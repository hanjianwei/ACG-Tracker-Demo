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

#ifndef __ACG_Tracker_Demo__Map__
#define __ACG_Tracker_Demo__Map__

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <array>
#include <list>
#include <dispatch/dispatch.h>
#include <Eigen/Core>

#include "Settings/Settings.h"
#include "Features/Match.h"
#include "Features/BRISK/BRISK.h"
#include "Features/FeatureTrack.h"
#include "Features/PatchDescription/PatchDescription.h"
#include "Types/BGRAImage.h"
#include "Types/SE3.h"
#include "Camera.h"
#include "MapPoint.h"

namespace ACGT
{
    class Map
    {
        
    public:
        
        // Constructor
        Map(const BGRAImage& img1,
            const Intrinsics& intrinsics1,
            const Eigen::Vector3d& gravity1,
            const BGRAImage& img2,
            const Intrinsics& intrinsics2,
            const Eigen::Vector3d& gravity2);
        
        // Desctructor
        ~Map();
        
        // Getter
        std::vector<SE3> keyframePoses() const;
        std::vector<Point3d> points() const;
        size_t keyframeCount() const;
        size_t pointCount() const;
        
        // Localization
        bool localize(const BGRAImage& img,
                      const Intrinsics& intrinsics,
                      SE3& pose);
        
        
    private:
        
        void _extractKeyframeFeatures(const Camera::ID& keyframeID,const bool& describe = false);
        void _setPointColor(const MapPoint::ID& pointID);
        std::vector<Match> _matchLSH(const std::vector<BRISKDescriptor<Settings::DescriptorSize>>& descriptors) const;
        std::vector<Match> _localize(Camera& frame);
        std::vector<Match> _trackPose(Camera& frame);
        void _resetLSHTables();
        void _resetFeatureTracks(const Camera& frame, const std::vector<Match>& matches);
        void _addPointKeyframeAssociation(const MapPoint::ID& pointID, Camera& keyframe, const int featureIdx);
        void _removePointKeyframeAssociation(const MapPoint::ID& pointID, Camera& keyframe);
        void _addPoint(MapPoint& point);
        void _removePoint(const MapPoint::ID pointID);
        bool _updateCanceled() const;
        bool _isKeyframe(const Camera& frame, const std::vector<Match>& inlierMatches);
        void _update(const BGRAImage& img,
                     const Intrinsics& intrinsics,
                     const SE3& pose);
        void _removeUninformativePoints();
        void _matchKeyframeWithPosePredict(const Camera& frame);
        void _matchToKeyframes2D2D(const Camera& keyframe);
        void _localizeKeyframe(Camera& keyframe);
        void _addPointKeyframeAssociations(ACGT::Camera& keyframe);
        void _triangulateNewPoints(Camera& newKf);
        void _removeKeyframe(const Camera& newKeyframe);
        void _bundleAdjustment();
        void _finalizeUpdate(const Camera& keyframe);
        
        std::unordered_map<long, Camera> _keyframes;
        std::unordered_map<long, MapPoint> _points;
        
        unsigned int _frameCount;
        
        uint8_t _detectionThreshold;
    
        std::list<Eigen::Vector3d> _lastFramePositions;
        double _minKeyframeDistance;
        
        std::array<std::array<std::unordered_set<MapPoint::ID>,256>,Settings::NumLSHTables> _LSHTables;
        
        dispatch_semaphore_t _semaphore;
        
        
        std::vector<FeatureTrack<PatchDescriptor,Settings::TrackDescriptorCount,Settings::TrackRetainInit>> _featureTracks;
        unsigned int _trackCount;
        
        ////
        // Update
        
        bool _updating;
        bool _cancelUpdate;
        dispatch_queue_t _updateQueue;
        std::vector<Match> _keyframeMatches2D3D;
        std::unordered_map<Camera::ID,std::vector<Match>> _keyframeMatches2D2D;
    };
}

#endif /* defined(__ACG_Tracker_Demo__Map__) */
