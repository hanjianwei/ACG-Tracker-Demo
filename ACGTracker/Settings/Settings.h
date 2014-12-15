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

#ifndef __ACG_Tracker_Demo__Settings__
#define __ACG_Tracker_Demo__Settings__

#include <cstdlib>
#include <cmath>

namespace ACGT
{
    namespace Settings
    {
        constexpr size_t DescriptorSize = 2; // Size of the BRISK descriptor in bytes divided by 16
        constexpr unsigned int DetectorOctaves = 0; // Number of octaves of the BRISK detector
        constexpr uint8_t MinDetectionThreshold = 30; // Minimal detection threshold used for BRISK detection
        constexpr unsigned int MatchingThreshold = 55; // Threshold for matching of two BRISK descriptors. Should be in the interval [0,DescriptorSize*128].
        constexpr unsigned int TargetFeatures = 1500; // The number of features that should be detected in the images
        constexpr double SampsonusErrorInlierThreshold = 0.00001; // Inlier threshold for the Sampsonus error used in the 3Pt essential matrix estimation
        constexpr unsigned int NumLSHTables = 6; // The number of hash tables that are used by the map for locality sensitive hashing
        constexpr unsigned int TrackDescriptorCount = 3; // The number of descriptors stored by each feature track
        constexpr unsigned int TrackRetainInit = 10; // The number of consecutive frames in which a feature track cannot be tracked until it is removed
        constexpr unsigned int RansacP3PMinIterations = 100; // The minimum number of RANSAC iterations in P3P pose estimation
        constexpr unsigned int RansacP3PMaxIterations = 1000; // The maximum number of RANSAC iterations in P3P pose estimation
        constexpr float ReprojectionErrorInlierThreshold = 5.0; // The inlier threshold for RANSAC P3P pose estimation
        constexpr unsigned int PNPRefineNonLinearMaxIterations = 50; // The maximum number of iterations in the non-linear optimization of the camera poses during PNP computation
        constexpr unsigned int MinPoseInliers = 12; // The minimum number of 2D-3D inliers for accepting a computed pose
        constexpr unsigned int MaxConsecutiveTrackings = 30; // The maximum number of consecutive frame-to-frame feature trackings before the map resorts to global LSH matching.
        constexpr unsigned int MinTrackingInliers = 30; // The minimum number of inliers for frame-to-frame tracking. If this number is not reached, the map resorts to global LSH matching.
        constexpr float MaxTrackingPixelDistance = 24.0; // The search radius for feature matches in frame-to-frame tracking.
        constexpr unsigned int PatchMatchingThreshold = 1024; // The threshold for matching of patch descriptors.
        constexpr unsigned int MinFeatureTracks = 50; // The minimum number of feature tracks for which frame-to-frame tracking is applied.
        constexpr unsigned int MinKeyframeInliers = 20; // The minimum number of 2D-3D inliers we expect from a keyframe.
        constexpr double MinTriangulationAngle = 5.0*M_PI/180.0; // The minimum angle required for triangulation of two features.
        constexpr unsigned int MaxKeyframes = 15; // The maximum number of keyframes in the map.
    }
}

#endif /* defined(__ACG_Tracker_Demo__Settings__) */
