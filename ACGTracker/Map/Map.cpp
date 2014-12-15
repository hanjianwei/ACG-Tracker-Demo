/*===========================================================================*\
 *                                                                           *
 *  ACG Tracker from the ECCV'14 Paper                                       *
 *  Scalable 6-DOF Localization on Mobile Devices                            *
 *  Copyright (C) 2014 by Computer Graphics Group, RWTH Aachen               *
 *  Author: Sven Middelberg <middelberg@cs.rwth-aachen.de>                   *
 *  www.rwth.graphics                                                        *
 *                                                                           *
 *---------------------------------------------------------------------------*
 *  This file is part of ACG Tracker                                         *
 *                                                                           *
 *  ACG Tracker is free software: you can redistribute it and/or modify      *
 *  it under the terms of the GNU General Public License as published by     *
 *  the Free Software Foundation, either version 3 of the License, or        *
 *  (at your option) any later version.                                      *
 *                                                                           *
 *  ACG Tracker is distributed in the hope that it will be useful,           *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
 *  GNU General Public License for more details.                             *
 *                                                                           *
 *  You should have received a copy of the GNU General Public License        *
 *  along with ACG Tracker. If not, see <http://www.gnu.org/licenses/>.      *
 *                                                                           *
\*===========================================================================*/

#include <cassert>
#include <limits>
#include <algorithm>
#include <dispatch/dispatch.h>

#include "Settings/Settings.h"
#include "Features/Feature.h"
#include "Features/PatchDescription/PatchDescription.h"
#include "Features/FeatureTracking.h"
#include "Features/TwoViewMatching.h"
#include "Utils/Metrics.h"
#include "Types/BGRAImage.h"
#include "Types/Intrinsics.h"
#include "Types/Point3d.h"
#include "Utils/Triangulation.h"
#include "SfM/SfM.h"
#include "SfM/BundleAdjustment.h"
#include "Map.h"
#include "Camera.h"
#include "MapPoint.h"
#include "PNP/PNP.h"

namespace ACGT
{
    Map::Map(const BGRAImage& img1,
             const Intrinsics& intrinsics1,
             const Eigen::Vector3d& gravity1,
             const BGRAImage& img2,
             const Intrinsics& intrinsics2,
             const Eigen::Vector3d& gravity2)
    {
        _semaphore = dispatch_semaphore_create(1);
        
        // Create cameras from color images
        Camera keyframe1(img1,intrinsics1,true);
        Camera keyframe2(img2,intrinsics2,true);
        keyframe1.gravity() = gravity1;
        keyframe2.gravity() = gravity2;
        const long keyframe1ID = keyframe1.id();
        const long keyframe2ID = keyframe2.id();
        
        // Add to keyframes array
        _keyframes[keyframe1ID] = std::move(keyframe1);
        _keyframes[keyframe2ID] = std::move(keyframe2);
        
        // Detect features
        _extractKeyframeFeatures(keyframe1ID,true);
        _extractKeyframeFeatures(keyframe2ID,true);
        
        // SfM
        _points = reconstructFromStereo(_keyframes[keyframe1ID], _keyframes[keyframe2ID]);
        
        // Bundle Adjustment
        bundleAdjustment(_keyframes, _points);
        for (auto& p: _points)
            p.second.coordinates() = p.second.coordinatesOpt();
        for (auto& k: _keyframes)
            k.second.pose() = k.second.poseOpt();
        
        // Set point colors
        for (auto& p : _points)
            _setPointColor(p.first);
        
        // Initialize LSH tables
        _resetLSHTables();
        
        _lastFramePositions.push_front(_keyframes[keyframe1ID].pose().C());
        _lastFramePositions.push_front(_keyframes[keyframe2ID].pose().C());
        
        // Set minimal keyframe distance to half the distance of the first two keyframes
        _minKeyframeDistance = 0.2*(_keyframes[keyframe2ID].pose().C()-_keyframes[keyframe1ID].pose().C()).norm();
        
        _trackCount = 0;
        
        _frameCount = 2;
        
        _updating = false;
        _updateQueue = dispatch_queue_create("Mapping Queue", NULL);
        dispatch_set_target_queue(dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_HIGH, 0), _updateQueue);
    }
    
    Map::~Map()
    {
        ////
        // Wait for the update queue to finish if it is not empty
        dispatch_semaphore_wait(_semaphore, DISPATCH_TIME_FOREVER);
        _cancelUpdate = true;
        dispatch_semaphore_signal(_semaphore);
        dispatch_sync(_updateQueue, ^(){});
        dispatch_release(_semaphore);
    }
    
    void Map::_extractKeyframeFeatures(const Camera::ID& keyframeID,const bool& describe)
    {
        assert(_keyframes.count(keyframeID));
        
        // Detect with minimal detection threshold
        _keyframes[keyframeID].extractFeatures(Settings::MinDetectionThreshold);
        
        if (_keyframes[keyframeID].features().size() <= Settings::TargetFeatures)
        {
            dispatch_semaphore_wait(_semaphore, DISPATCH_TIME_FOREVER);
            _detectionThreshold = (int)Settings::MinDetectionThreshold;
            dispatch_semaphore_signal(_semaphore);
        } else
        {
            // Find detection threshold that detects at least Settings::TargetFeatures features
            std::nth_element(std::begin(_keyframes[keyframeID].features()),
                             std::begin(_keyframes[keyframeID].features())+Settings::TargetFeatures-1,
                             std::end(_keyframes[keyframeID].features()),
                             [](const Feature& f1, const Feature& f2) -> bool
                             {
                                 return f1.score() > f2.score();
                             });
            
            dispatch_semaphore_wait(_semaphore, DISPATCH_TIME_FOREVER);
            _detectionThreshold = _keyframes[keyframeID].features()[Settings::TargetFeatures-1].score();
            dispatch_semaphore_signal(_semaphore);
            
            // Erase all features with a smaller detection threshold
            _keyframes[keyframeID].features().erase(std::remove_if(std::begin(_keyframes[keyframeID].features()),
                                                                   std::end(_keyframes[keyframeID].features()),
                                                                   [&](const Feature& f)
                                                                   {
                                                                       return f.score() < _detectionThreshold;
                                                                   }),
                                                    std::end(_keyframes[keyframeID].features()));
        }
        
        if (describe)
            _keyframes[keyframeID].extractDescriptors();
    }
    
    void Map::_setPointColor(const MapPoint::ID& pointID)
    {
        assert(_points.count(pointID));
        
        MapPoint& p = _points[pointID];
        p.color() = Color(0.0f,0.0f,0.0f);
        int numColors = 0;
        for (const auto& k : p.keyframeReferences())
        {
            if (_keyframes[k.keyframeID()].hasColorImage())
            {
                const Feature& f = _keyframes[k.keyframeID()].features()[k.featureIdx()];
                const Color fColor = _keyframes[k.keyframeID()].color(f.x(), f.y());
                p.color().r() += fColor.r();
                p.color().g() += fColor.g();
                p.color().b() += fColor.b();
                numColors++;
            }
        }
        if (numColors)
        {
            p.color().r() = std::min(p.color().r()/(float)numColors, 1.0f);
            p.color().g() = std::min(p.color().g()/(float)numColors, 1.0f);
            p.color().b() = std::min(p.color().b()/(float)numColors, 1.0f);
        }
    }
    
    std::vector<Match> Map::_matchLSH(const std::vector<BRISKDescriptor<Settings::DescriptorSize>>& descriptors) const
    {
        __block std::vector<Match> matches;
        
        // Compute matches for 16 subsets of the descriptors simultaneously
        const int rangeSize = (int)ceil((float)descriptors.size()/16.0);
        dispatch_semaphore_t semaphore = dispatch_semaphore_create(1);
        
        dispatch_apply(16, dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_HIGH, 0), ^(const size_t n)
                       {
                           const int startIdx = n*rangeSize;
                           const int endIdx = std::min((n+1)*rangeSize, descriptors.size());
                           
                           // compute matches:
                           for (int idx1=startIdx; idx1<endIdx; ++idx1)
                           {
                               const BRISKDescriptor<Settings::DescriptorSize>& d1 = descriptors[idx1];
                               Match nn;
                               for (int h=0; h<Settings::NumLSHTables; ++h)
                               {
                                   const uint8_t d1Byte = d1[h];
                                   for (const MapPoint::ID& pID : _LSHTables[h][d1Byte])
                                   {
                                       for (const KeyframeReference& kRef : _points.find(pID)->second.keyframeReferences())
                                       {
                                           assert(_keyframes.find(kRef.keyframeID()) != _keyframes.end());
                                           assert(_keyframes.find(kRef.keyframeID())->second.descriptors().size() > kRef.featureIdx() && kRef.featureIdx() >= 0);
                                           const BRISKDescriptor<Settings::DescriptorSize>& d2 = _keyframes.find(kRef.keyframeID())->second.descriptors()[kRef.featureIdx()];
                                           if (d1Byte == d2[h])
                                           {
                                               const float distance = d1.distance(d2);
                                               if (distance < nn.distance())
                                                   nn = Match(idx1,pID,distance);
                                           }
                                       }
                                   }
                               }
                               if (nn.distance() < Settings::MatchingThreshold)
                               {
                                   dispatch_semaphore_wait(semaphore, DISPATCH_TIME_FOREVER);
                                   matches.push_back(nn);
                                   dispatch_semaphore_signal(semaphore);
                               }
                           }
                       });
                       
        return matches;
    }
    
    void Map::_addPointKeyframeAssociation(const MapPoint::ID& pointID, Camera& keyframe, const int featureIdx)
    {
        dispatch_semaphore_wait(_semaphore, DISPATCH_TIME_FOREVER);
        _points[pointID].addKeyframeReference(keyframe, featureIdx);
        keyframe.addPointReference(featureIdx, pointID);
        for (int h=0; h<Settings::NumLSHTables; ++h)
        {
            const uint8_t byte = keyframe.descriptors()[featureIdx][h];
            _LSHTables[h][byte].insert(pointID);
        }
        _setPointColor(pointID);
        dispatch_semaphore_signal(_semaphore);
    }
    
    void Map::_removePointKeyframeAssociation(const MapPoint::ID& pointID, Camera& keyframe)
    {
        dispatch_semaphore_wait(_semaphore, DISPATCH_TIME_FOREVER);
        const auto& it = std::find_if(std::begin(_points[pointID].keyframeReferences()), std::end(_points[pointID].keyframeReferences()), [&](const KeyframeReference& ref)
                                       {
                                           return ref.keyframeID() == keyframe.id();
                                       });
        const int featureIdx = it->featureIdx();
        for (int h=0; h<Settings::NumLSHTables; ++h)
        {
            const uint8_t byte = keyframe.descriptors()[featureIdx][h];
            bool unique = std::count_if(std::begin(_points[pointID].keyframeReferences()), std::end(_points[pointID].keyframeReferences()), [&](const KeyframeReference& ref)
                                        {
                                            return byte == _keyframes[ref.keyframeID()].descriptors()[ref.featureIdx()][h];
                                        }) == 1;
            if (unique)
                _LSHTables[h][byte].erase(pointID);
        }
        _points[pointID].removeKeyframeReference(keyframe.id());
        keyframe.removePointReference(featureIdx);
        dispatch_semaphore_signal(_semaphore);
    }
    
    void Map::_addPoint(MapPoint& point)
    {
        const MapPoint::ID pointID = point.id();
        dispatch_semaphore_wait(_semaphore, DISPATCH_TIME_FOREVER);
        for (const KeyframeReference& k : point.keyframeReferences())
        {
            for (int h=0; h<Settings::NumLSHTables; ++h)
            {
                const uint8_t byte = _keyframes[k.keyframeID()].descriptors()[k.featureIdx()][h];
                _LSHTables[h][byte].insert(pointID);
            }
            _keyframes[k.keyframeID()].addPointReference(k.featureIdx(), pointID);
        }
        _points[pointID] = std::move(point);
        _setPointColor(pointID);
        dispatch_semaphore_signal(_semaphore);
    }
    
    void Map::_removePoint(const MapPoint::ID pointID)
    {
        assert(_points.find(pointID) != std::end(_points));
        
        MapPoint& point = _points[pointID];
        
        dispatch_semaphore_wait(_semaphore, DISPATCH_TIME_FOREVER);
        assert(std::count_if(std::begin(_featureTracks), std::end(_featureTracks), [&](const FeatureTrack<PatchDescriptor,Settings::TrackDescriptorCount,Settings::TrackRetainInit>& track)
                             {
                                 return track.trackedObjectID() == pointID;
                             }) <= 1);
        auto it = std::find_if(std::begin(_featureTracks), std::end(_featureTracks), [&](const FeatureTrack<PatchDescriptor,Settings::TrackDescriptorCount,Settings::TrackRetainInit>& track)
                               {
                                   return track.trackedObjectID() == pointID;
                               });
        if (it != std::end(_featureTracks))
            _featureTracks.erase(it);
        assert(std::count_if(std::begin(_featureTracks), std::end(_featureTracks), [&](const FeatureTrack<PatchDescriptor,Settings::TrackDescriptorCount,Settings::TrackRetainInit>& track)
                             {
                                 return track.trackedObjectID() == pointID;
                             }) == 0);
        for (const KeyframeReference& k : point.keyframeReferences())
        {
            for (int h=0; h<Settings::NumLSHTables; ++h)
            {
                const uint8_t byte = _keyframes[k.keyframeID()].descriptors()[k.featureIdx()][h];
                _LSHTables[h][byte].erase(pointID);
            }
            _keyframes[k.keyframeID()].removePointReference(k.featureIdx());
            
        }
        _points.erase(pointID);
        dispatch_semaphore_signal(_semaphore);
    }
    
    void Map::_resetLSHTables()
    {
        dispatch_semaphore_wait(_semaphore, DISPATCH_TIME_FOREVER);
        for (auto t : _LSHTables)
        {
            for (auto b : t)
            {
                b.clear();
            }
        }
        for (auto& p : _points)
        {
            for (const KeyframeReference& k : p.second.keyframeReferences())
            {
                for (int h=0; h<Settings::NumLSHTables; ++h)
                {
                    const uint8_t byte = _keyframes[k.keyframeID()].descriptors()[k.featureIdx()][h];
                    _LSHTables[h][byte].insert(p.first);
                }
            }
        }
        dispatch_semaphore_signal(_semaphore);
    }
    
    std::vector<Match> Map::_localize(Camera& frame)
    {
        // Compute frames features and descriptors if not done yet
        if (frame.features().size() == 0)
            frame.extractFeatures(_detectionThreshold,true);
        if (frame.descriptors().size() == 0)
            frame.extractDescriptors();
        
        // Compute matches
        std::vector<Match> matches = _matchLSH(frame.descriptors());
        
        // Increment point's match counts
        for (auto mIt = std::begin(matches); mIt<std::end(matches); ++mIt)
        {
            MapPoint::ID matchedPointID = mIt->idx2();
            if (!std::count_if(std::begin(matches), mIt, [&](const Match& m)
                               {
                                   return matchedPointID == m.idx2();
                               }))
                _points[matchedPointID].matchCount()++;
        }
        
        // Estimate pose
        std::vector<Eigen::Vector3d> NDCs;
        NDCs.reserve(matches.size());
        std::vector<Eigen::Vector3d> points;
        points.reserve(matches.size());
        for (const Match& m : matches)
        {
            NDCs.push_back(frame.features()[m.idx1()].NDCs(frame));
            points.push_back(_points.find(m.idx2())->second.coordinates());
        }
        
        std::vector<int> inliers = PNP::PNP(frame.pose(), NDCs, points, frame.intrinsics(), Settings::ReprojectionErrorInlierThreshold);
        
        // Increment point's inlier counts
        for (auto mIt = std::begin(matches); mIt<std::end(matches); ++mIt)
        {
            MapPoint::ID matchedPointID = mIt->idx2();
            if (!std::count_if(std::begin(matches), mIt, [&](const Match& m)
                               {
                                   return matchedPointID == m.idx2();
                               }))
                _points[matchedPointID].inlierCount()++;
        }

        for (int i=0; i<inliers.size(); ++i)
            matches[i] = matches[inliers[i]];
        matches.resize(inliers.size());
        
        return matches;
    }
    
    bool Map::localize(const BGRAImage& img,
                       const Intrinsics& intrinsics,
                       SE3& pose)
    {
        ////
        // Create camera from color image
        Camera frame(img,intrinsics,false);

        ////
        // Compute pose
        frame.extractFeatures(_detectionThreshold);
        std::vector<Match> inliers = _trackPose(frame);
        
        
        if (inliers.empty())
        {
            frame.extractDescriptors();
            dispatch_semaphore_wait(_semaphore, DISPATCH_TIME_FOREVER);
            inliers = _localize(frame);
            _resetFeatureTracks(frame, inliers);
            dispatch_semaphore_signal(_semaphore);
        }
        
        if (inliers.size() < Settings::MinPoseInliers)
            return false;
        
        pose = frame.pose();
        
        dispatch_semaphore_wait(_semaphore, DISPATCH_TIME_FOREVER);
        _frameCount++;
        dispatch_semaphore_signal(_semaphore);
        
        ////
        // Update
        if (_isKeyframe(frame, inliers))
            _update(img, intrinsics, pose);
        
        return true;
    }
    
    std::vector<Match> Map::_trackPose(Camera& frame)
    {
        std::vector<Match> matches;
   
        // Compute frames features if not done yet
        if (frame.features().size() == 0)
            frame.extractFeatures(_detectionThreshold,false);
        
        if (_trackCount == 0 || _featureTracks.size() < Settings::MinFeatureTracks)
        {
            // Maximum number of consecutive frame-to-frame trackings reached or not enough tracks
            return matches;
        }
        
        // Sort features according to x-coordinate for feature tracking
        std::sort(std::begin(frame.features()), std::end(frame.features()), [](const Feature& f1, const Feature& f2)
                  {
                      return f1.x() < f2.x();
                  });
        
        _trackCount--;
        
        ////
        // Describe features
        
        PatchDescription description;
        std::vector<PatchDescriptor> descriptors = description.extract(frame.image(),frame.features());
        
        dispatch_semaphore_wait(_semaphore, DISPATCH_TIME_FOREVER);
        matches = FeatureTracking::track<PatchDescriptor, Settings::TrackDescriptorCount, Settings::TrackRetainInit>(frame.features(),
                                                                                                                     descriptors,
                                                                                                                     _featureTracks,
                                                                                                                     Settings::MaxTrackingPixelDistance,
                                                                                                                     Settings::PatchMatchingThreshold);
        
        // Increment point's match counts
        for (auto mIt = std::begin(matches); mIt<std::end(matches); ++mIt)
        {
            MapPoint::ID trackedPointID = _featureTracks[mIt->idx2()].trackedObjectID();
            if (!std::count_if(std::begin(matches), mIt, [&](const Match& m)
                               {
                                   return trackedPointID == _featureTracks[m.idx2()].trackedObjectID();
                               }))
                _points[trackedPointID].matchCount()++;
        }

        // Estimate pose
        std::vector<Eigen::Vector3d> NDCs;
        NDCs.reserve(matches.size());
        std::vector<Eigen::Vector3d> points;
        points.reserve(matches.size());
        for (const Match& m : matches)
        {
            NDCs.push_back(frame.features()[m.idx1()].NDCs(frame));
            points.push_back(_points.find(_featureTracks[m.idx2()].trackedObjectID())->second.coordinates());
        }
     
        std::vector<int> inliers = PNP::PNP(frame.pose(), NDCs, points, frame.intrinsics(), Settings::ReprojectionErrorInlierThreshold);

        for (int i=0; i<inliers.size(); ++i)
            matches[i] = matches[inliers[i]];
        matches.resize(inliers.size());
        
        // Update tracks and predicts
        for (int tIdx=0; tIdx<_featureTracks.size(); ++tIdx)
        {
            auto& track = _featureTracks[tIdx];
            const auto m = std::find_if(std::begin(matches), std::end(matches), [&](const Match& m)
            {
                return tIdx == m.idx2();
            });
            if (m != std::end(matches))
            {
                track.trackedSuccessfully(descriptors[m->idx1()]);
                track.predict() = frame.features()[m->idx1()].coordinates();
            } else
            {
                track.lost();
                Eigen::Vector2d reprojection = _points.find(track.trackedObjectID())->second.reproject(frame.pose(), frame.intrinsics());
                Eigen::Vector2f predict(reprojection.x(),frame.image().height()-1-reprojection.y());
                track.predict() = predict;
            }
        }
        
        // Increment point's inlier counts
        for (auto mIt = std::begin(matches); mIt<std::end(matches); ++mIt)
        {
            MapPoint::ID trackedPointID = _featureTracks[mIt->idx2()].trackedObjectID();
            if (!std::count_if(std::begin(matches), mIt, [&](const Match& m)
                               {
                                   return trackedPointID == _featureTracks[m.idx2()].trackedObjectID();
                               }))
                _points[trackedPointID].inlierCount()++;
        }
        
        dispatch_semaphore_signal(_semaphore);
        
        // Delete tracks with a zero retain counter
        int numLeft = 0;
        for (int tIdx=0; tIdx<_featureTracks.size(); ++tIdx)
        {
            if (_featureTracks[tIdx].retainCounter() > 0)
            {
                if (tIdx != numLeft)
                    _featureTracks[numLeft] = std::move(_featureTracks[tIdx]);
                numLeft++;
            }
        }
        _featureTracks.resize(numLeft);
        
        if (matches.size() < Settings::MinTrackingInliers)
            _trackCount = 0;
        
        return matches;
    }
    
    void Map::_resetFeatureTracks(const Camera& frame, const std::vector<Match>& matches)
    {
        _featureTracks.clear();
        
        // Detect duplicates
        std::vector<int> duplicates(matches.size(),0);
        for (int mIdx=0; mIdx<matches.size(); ++mIdx)
        {
            if (std::count_if(std::begin(matches), std::end(matches), [&](const Match& m)
                              {
                                  return m.idx2() == matches[mIdx].idx2();
                              }) > 1)
                duplicates[mIdx] = 1;
        }
        
        PatchDescription description;
        
        for (int m=0; m<matches.size(); ++m)
        {
            if (duplicates[m]) {
                continue;
            }
            
            const Feature& feature = frame.features()[matches[m].idx1()];
            if (!description.descriptable(frame.image(), feature))
                continue;
            
            FeatureTrack<PatchDescriptor, Settings::TrackDescriptorCount, Settings::TrackRetainInit> track;
            track.trackedObjectID() = matches[m].idx2();
            track.predict() = feature.coordinates();
            track.trackedSuccessfully(description.extract(frame.image(), feature));
            _featureTracks.push_back(track);
        }
        
        _trackCount = Settings::MaxConsecutiveTrackings;
    }
    
    
    std::vector<SE3> Map::keyframePoses() const
    {
        std::vector<SE3> poses;
        dispatch_semaphore_wait(_semaphore, DISPATCH_TIME_FOREVER);
        poses.reserve(_keyframes.size());
        for (const auto& k : _keyframes)
            poses.push_back(k.second.pose());
        dispatch_semaphore_signal(_semaphore);
        return poses;
    }
    
    std::vector<Point3d> Map::points() const
    {
        std::vector<Point3d> points3d;
        dispatch_semaphore_wait(_semaphore, DISPATCH_TIME_FOREVER);
        points3d.reserve(_points.size());
        for (const auto& p : _points)
            points3d.push_back(Point3d(p.second.coordinates(),p.second.color()));
        dispatch_semaphore_signal(_semaphore);
        return points3d;
    }
    
    size_t Map::keyframeCount() const
    {
        dispatch_semaphore_wait(_semaphore, DISPATCH_TIME_FOREVER);
        const size_t count = _keyframes.size();
        dispatch_semaphore_signal(_semaphore);
        return count;
    }
    
    size_t Map::pointCount() const
    {
        dispatch_semaphore_wait(_semaphore, DISPATCH_TIME_FOREVER);
        const size_t count = _points.size();
        dispatch_semaphore_signal(_semaphore);
        return count;
    }
    
    ////
    // Map Update
    
    bool Map::_updateCanceled() const
    {
        dispatch_semaphore_wait(_semaphore, DISPATCH_TIME_FOREVER);
        bool canceled = _cancelUpdate;
        dispatch_semaphore_signal(_semaphore);
        return canceled;
    }
    
    bool Map::_isKeyframe(const Camera& frame, const std::vector<Match>& inlierMatches)
    {
        
        const Eigen::Vector3d fPosition = frame.pose().C();
        
        ////
        // Don't update during another update!
        
        dispatch_semaphore_wait(_semaphore, DISPATCH_TIME_FOREVER);
        bool updating = _updating;
        dispatch_semaphore_signal(_semaphore);
        
        if (updating)
        {
            _lastFramePositions.push_front(fPosition);
            if (_lastFramePositions.size() > 3)
                _lastFramePositions.pop_back();
            
            return false;
        }
        
        bool isKeyframe = true;
        _lastFramePositions.push_front(fPosition);
        
        const double minKeyframeDistanceSquared = _minKeyframeDistance*_minKeyframeDistance;
        
        ////
        // Ceck if estimated position is near the positions of the last three frames to ensure the pose estimation didn't fail
        
        if (_lastFramePositions.size() <= 3)
            return false;

        for (const Eigen::Vector3d& p : _lastFramePositions)
        {
            if ((p-fPosition).squaredNorm() > minKeyframeDistanceSquared)
            {
                isKeyframe = false;
                break;
            }
        }
        
        _lastFramePositions.pop_back();
        
        ////
        // Check if frame has a sufficient distance to all other keyframes
        
        for (const auto& keyframe : _keyframes)
        {
            double squaredDistance = (fPosition-keyframe.second.pose().C()).squaredNorm();
            if (squaredDistance < minKeyframeDistanceSquared)
            {
                isKeyframe = false;
                break;
            }
        }
        if (isKeyframe)
            return true;
        
        if (inlierMatches.size() > 200)
            return false;
        
        std::vector<int> rectInliers(16,0);
        
        for (const auto& m : inlierMatches)
        {
            const int xRect = frame.features()[m.idx1()].x()/(frame.image().width()/4);
            const int yRect = frame.features()[m.idx1()].y()/(frame.image().height()/4);
            rectInliers[4*yRect+xRect]++;
        }
        
        // Upper image half
        int halfInlierCount = 0;
        for (int i=0; i<8; ++i)
            halfInlierCount += rectInliers[i];

        if (halfInlierCount < inlierMatches.size()/8)
            return true;
        
        // Lower image half
        halfInlierCount = 0;
        for (int i=8; i<16; ++i)
            halfInlierCount += rectInliers[i];

        if (halfInlierCount < inlierMatches.size()/8)
            return true;
        
        // Left image half
        halfInlierCount = 0;
        for (int i=0; i<16; i+=4)
        {
            halfInlierCount += rectInliers[i];
            halfInlierCount += rectInliers[i+1];
        }
        if (halfInlierCount < inlierMatches.size()/8)
            return true;
        
        // Right image half
        halfInlierCount = 0;
        for (int i=2; i<16; i+=4)
        {
            halfInlierCount += rectInliers[i];
            halfInlierCount += rectInliers[i+1];
        }
        if (halfInlierCount < inlierMatches.size()/8)
            return true;
        
        return false;
    }
    
    void Map::_update(const BGRAImage& image, const Intrinsics& intrinsics, const SE3& pose)
    {
        Camera keyframe(image,intrinsics,true);
        keyframe.pose() = pose;
        const Camera::ID keyframeID = keyframe.id();
        _keyframes[keyframeID] = std::move(keyframe);
   
        _updating = true;
        
        _cancelUpdate = false;
        
        _removeUninformativePoints();
        
        dispatch_async(_updateQueue, ^(void)
        {
            if (_updateCanceled())
                return;
            _extractKeyframeFeatures(keyframeID,true);
            if (_updateCanceled())
                return;
            _matchKeyframeWithPosePredict(_keyframes[keyframeID]);
        });
        
        _matchToKeyframes2D2D(_keyframes[keyframeID]);
        
        _localizeKeyframe(_keyframes[keyframeID]);
        
        _addPointKeyframeAssociations(_keyframes[keyframeID]);
        
        _triangulateNewPoints(_keyframes[keyframeID]);
        
        _removeKeyframe(_keyframes[keyframeID]);
        
        _bundleAdjustment();
        
        _finalizeUpdate(_keyframes[keyframeID]);
    }
    
    void Map::_removeUninformativePoints()
    {
        dispatch_async(_updateQueue, ^(void)
        {
            if (_updateCanceled())
                return;
            
            dispatch_semaphore_wait(_semaphore, DISPATCH_TIME_FOREVER);
            const int currentFrameCount = _frameCount;
            dispatch_semaphore_signal(_semaphore);
            
            std::vector<MapPoint::ID> pointsToDelete;
            for (const auto& p : _points)
            {
                const int lifetime = currentFrameCount-p.second.framestamp();
                if ((lifetime > 100 && (float)p.second.matchCount()/(float)lifetime < 0.1) ||
                    (p.second.matchCount() > 100 && (float)p.second.inlierCount()/(float)p.second.matchCount() < 0.1))
                {
                    // Uninformative map point detected
                    pointsToDelete.push_back(p.first);
                }
            }
            for(const MapPoint::ID& pID : pointsToDelete)
                _removePoint(pID);
        });
    }
    
    void Map::_matchKeyframeWithPosePredict(const Camera& frame)
    {
        _keyframeMatches2D3D.reserve(_points.size());
        for (const auto& p : _points)
        {
            // Reproject point onto frame
            Eigen::Vector2d reprojection = p.second.reproject(frame.pose(), frame.intrinsics());
            reprojection.y() = frame.image().height()-1.0-reprojection.y();
            
            // Search for matches in a 10x10 patch surrounding the reprojection
            const float xLeft = reprojection.x()-5.0;
            const float xRight = reprojection.x()+5.0;
            const float yTop = reprojection.y()-5.0;
            const float yBottom = reprojection.y()+5.0;
            
            Match nn;
            for (int fIdx=0; fIdx<frame.features().size(); ++fIdx)
            {
                const Feature& f = frame.features()[fIdx];
                if (f.x() < xLeft || f.x() > xRight || f.y() < yTop || f.y() > yBottom)
                    continue;
                
                for (const auto& k : p.second.keyframeReferences())
                {
                    float distance = _keyframes[k.keyframeID()].descriptors()[k.featureIdx()].distance(frame.descriptors()[fIdx]);
                    if (distance < nn.distance())
                        nn = Match(fIdx,p.second.id(),distance);
                }
            }
            if (nn.distance() < std::numeric_limits<float>::max())
                _keyframeMatches2D3D.push_back(nn);
        }
    }
    
    void Map::_matchToKeyframes2D2D(const Camera& newKeyframe)
    {
        for (auto& oldKeyframe : _keyframes)
        {
            if (oldKeyframe.first == newKeyframe.id())
                continue;
            dispatch_async(_updateQueue, ^(void)
            {
                if (_updateCanceled())
                    return;
                
                _keyframeMatches2D2D[oldKeyframe.first] = TwoViewMatching::matchNNMutualDR(oldKeyframe.second.descriptors(), newKeyframe.descriptors(), Settings::MatchingThreshold, 0.8);
            });
        }
    }
    
    void Map::_localizeKeyframe(Camera& keyframe)
    {
        dispatch_async(_updateQueue, ^(void)
        {
            if (_updateCanceled())
                return;
            
            // Estimate pose
            std::vector<Eigen::Vector3d> NDCs;
            NDCs.reserve(_keyframeMatches2D3D.size());
            std::vector<Eigen::Vector3d> points;
            points.reserve(_keyframeMatches2D3D.size());
            for (const Match& m : _keyframeMatches2D3D)
            {
                NDCs.push_back(keyframe.features()[m.idx1()].NDCs(keyframe));
                points.push_back(_points.find(m.idx2())->second.coordinates());
            }
            
            std::vector<int> inliers = PNP::PNP(keyframe.pose(), NDCs, points, keyframe.intrinsics(), Settings::ReprojectionErrorInlierThreshold);

            if (inliers.size()<Settings::MinKeyframeInliers)
            {
                dispatch_semaphore_wait(_semaphore, DISPATCH_TIME_FOREVER);
                _cancelUpdate = true;
                dispatch_semaphore_signal(_semaphore);
                return;
            }
            
            for (int i=0; i<inliers.size(); ++i)
                _keyframeMatches2D3D[i] = _keyframeMatches2D3D[inliers[i]];
            _keyframeMatches2D3D.resize(inliers.size());
            
        });
    }
    
    void Map::_addPointKeyframeAssociations(ACGT::Camera& keyframe)
    {
        dispatch_async(_updateQueue, ^(void)
        {
            if (_updateCanceled())
                return;
            
            // Remove 2D-3D inliers that are not unique among the 2D features
            std::vector<int> duplicates(_keyframeMatches2D3D.size(),0);
            for (int mIdx=0; mIdx<_keyframeMatches2D3D.size(); ++mIdx)
            {
                if (std::count_if(std::begin(_keyframeMatches2D3D), std::end(_keyframeMatches2D3D), [&](const Match& m)
                                  {
                                      return m.idx1() == _keyframeMatches2D3D[mIdx].idx1();
                                  }) > 1)
                    duplicates[mIdx] = 1;
            }
            for (int mIdx = _keyframeMatches2D3D.size()-1; mIdx>=0; --mIdx)
            {
                if (duplicates[mIdx])
                {
                    // One feature corresponds to multiple points, don't trust these points
                    _removePoint(_keyframeMatches2D3D[mIdx].idx2());
                    _keyframeMatches2D3D.erase(std::begin(_keyframeMatches2D3D)+mIdx);
                } else
                    _addPointKeyframeAssociation(_keyframeMatches2D3D[mIdx].idx2(), keyframe, _keyframeMatches2D3D[mIdx].idx1());
            }
        });
    }
    
    void Map::_triangulateNewPoints(Camera& newKf)
    {
        const float squaredRepErrThreshold = Settings::ReprojectionErrorInlierThreshold*Settings::ReprojectionErrorInlierThreshold;
        for (auto& oldKf : _keyframes)
        {
            if (oldKf.first == newKf.id())
                continue;
            dispatch_async(_updateQueue, ^(void)
            {
                if (_updateCanceled())
                    return;
        
                const std::vector<Match>& matches = _keyframeMatches2D2D[oldKf.first];
                for (const Match& m : matches)
                {
                    const MapPoint::ID& pRefOld = oldKf.second.pointReferences()[m.idx1()];
                    const MapPoint::ID& pRefNew = newKf.pointReferences()[m.idx2()];
                    if (pRefOld && pRefNew)
                    {
                        if (pRefOld == pRefNew)
                        {
                            ////
                            // Both features already belong to the same map point or were deleted. Nothing to do!
                        } else
                        {
                            ////
                            // The features belong to different map points or one of them was deleted. We don't trust any of them, so delete both.
                            if (pRefOld > 0)
                                _removePoint(pRefOld);
                            if (pRefNew > 0)
                                _removePoint(pRefNew);
                        }
                    } else if (pRefOld)
                    {
                        if (pRefOld > 0)
                        {
                            ////
                            // Check if the new keyframe's feature should be added to the point
                            
                            MapPoint& p = _points[pRefOld];
                            const Feature& f = newKf.features()[m.idx2()];
                            if (Metrics::squaredReprojectionError(newKf.pose().R()*p.coordinates()+newKf.pose().t(), f.NDCs(newKf), newKf.intrinsics()) < squaredRepErrThreshold)
                            {
                                ////
                                // Reprojection error small enough. Check if point has already indirectly established a correspondence to the new keyframe.
                                // If so, erase point. If not, add feature to point.
                                if (std::find_if(std::begin(p.keyframeReferences()), std::end(p.keyframeReferences()), [&](const KeyframeReference& ref)
                                                 {
                                                     return ref.keyframeID() == newKf.id();
                                                 }) != std::end(p.keyframeReferences()))
                                {
                                    _removePoint(pRefOld);
                                    newKf.markUnstable(m.idx2());
                                } else
                                    _addPointKeyframeAssociation(pRefOld, newKf, m.idx2());
                            }
                        }
                    } else if (pRefNew)
                    {
                        if (pRefNew > 0)
                        {
                            ////
                            // Check if the old keyframe's feature should be added to the point
                            
                            MapPoint& p = _points[pRefNew];
                            const Feature& f = oldKf.second.features()[m.idx1()];
                            if (Metrics::squaredReprojectionError(oldKf.second.pose().R()*p.coordinates()+oldKf.second.pose().t(), f.NDCs(oldKf.second), oldKf.second.intrinsics()) < squaredRepErrThreshold)
                            {
                                ////
                                // Reprojection error small enough. Check if point has already indirectly established a correspondence to the old keyframe.
                                // If so, erase point. If not, add feature to point.
                                if (std::find_if(std::begin(p.keyframeReferences()), std::end(p.keyframeReferences()), [&](const KeyframeReference& ref)
                                                 {
                                                     return ref.keyframeID() == oldKf.second.id();
                                                 }) != std::end(p.keyframeReferences()))
                                {
                                    _removePoint(pRefNew);
                                    oldKf.second.markUnstable(m.idx1());
                                } else
                                    _addPointKeyframeAssociation(pRefNew, oldKf.second, m.idx1());
                            }
                        }
                    } else
                    {
                        ////
                        // None of both features is in the map. Check if the triangulation angle is big enough and if the match can be verified geometrically.
                        const Feature& fOld = oldKf.second.features()[m.idx1()];
                        const Feature& fNew = newKf.features()[m.idx2()];
                        Eigen::Vector3d NDCsOld = fOld.NDCs(oldKf.second);
                        NDCsOld.normalize();
                        Eigen::Vector3d NDCsNew = fNew.NDCs(newKf);
                        NDCsNew.normalize();
                        if (acos((oldKf.second.pose().R().transpose()*NDCsOld).dot(newKf.pose().R().transpose()*NDCsNew)) < Settings::MinTriangulationAngle)
                            continue;
                        
                        ////
                        // Angle big enough, triangulate map point
                        MapPoint newPoint;
                        std::vector<Eigen::Vector3d> NDCs(2);
                        NDCs[0] = NDCsOld;
                        NDCs[1] = NDCsNew;
                        std::vector<SE3> poses(2);
                        poses[0] = oldKf.second.pose();
                        poses[1] = newKf.pose();
                        newPoint.coordinates() = Triangulation::triangulate(NDCs, poses);
                        
                        ////
                        // Check cheirality constraint for both keyframes
                        Eigen::Vector3d newPOld = oldKf.second.pose().R()*newPoint.coordinates()+oldKf.second.pose().t();
                        if (newPOld.z() >= 0.0)
                            continue;
                        Eigen::Vector3d newPNew = newKf.pose().R()*newPoint.coordinates()+newKf.pose().t();
                        if (newPNew.z() >= 0.0)
                            continue;
                        
                        ////
                        // Verify geometrically
                        if (Metrics::squaredReprojectionError(NDCsOld, newPOld, oldKf.second.intrinsics()) > squaredRepErrThreshold)
                            continue;
                        if (Metrics::squaredReprojectionError(NDCsNew, newPNew, newKf.intrinsics()) > squaredRepErrThreshold)
                            continue;
                        
                        ////
                        // Match seems to be ok: insert!
                        newPoint.addKeyframeReference(oldKf.second, m.idx1());
                        newPoint.addKeyframeReference(newKf, m.idx2());
                        
                        _addPoint(newPoint);
                    }
                }
            });
        }
    }
    
    void Map::_removeKeyframe(const Camera& newKeyframe)
    {
        dispatch_async(_updateQueue, ^(void)
        {
            if (_updateCanceled())
                return;
            
            if(_keyframes.size() <= Settings::MaxKeyframes)
                return;
            
            float maxDistance = 0.0;
            Camera::ID keyframeToDelete = 0;
            for (const auto& kf : _keyframes)
            {
                const float distance = (kf.second.pose().C()-newKeyframe.pose().C()).squaredNorm();
                if (distance > maxDistance)
                {
                    maxDistance = distance;
                    keyframeToDelete = kf.first;
                }
            }
            
            for (const MapPoint::ID& pID : _keyframes[keyframeToDelete].pointReferences())
            {
                if (pID > 0)
                {
                    _removePointKeyframeAssociation(pID, _keyframes[keyframeToDelete]);
                    if (_points[pID].keyframeReferences().size() < 2)
                        _removePoint(pID);
                }
            }
            _keyframes.erase(keyframeToDelete);
        });
    }
    
    void Map::_bundleAdjustment()
    {
        dispatch_async(_updateQueue, ^(void)
                       {
                           if (_updateCanceled())
                               return;
                           
                           bundleAdjustment(_keyframes, _points);
                           dispatch_semaphore_wait(_semaphore, DISPATCH_TIME_FOREVER);
                           for (auto& p: _points)
                               p.second.coordinates() = p.second.coordinatesOpt();
                           for (auto& k: _keyframes)
                               k.second.pose() = k.second.poseOpt();
                           dispatch_semaphore_signal(_semaphore);
                       });
    }
    
    void Map::_finalizeUpdate(const Camera& keyframe)
    {
        dispatch_async(_updateQueue, ^(void)
        {
            if (_updateCanceled())
                _keyframes.erase(keyframe.id());
            _keyframeMatches2D3D.clear();
            _keyframeMatches2D2D.clear();
            dispatch_semaphore_wait(_semaphore, DISPATCH_TIME_FOREVER);
            _updating = false;
            dispatch_semaphore_signal(_semaphore);
        });
        
        
    }
}