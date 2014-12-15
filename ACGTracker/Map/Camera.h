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

#ifndef __pose_estimation_bmvc__Camera__
#define __pose_estimation_bmvc__Camera__

#include <vector>
#include <memory>
#include <Eigen/Core>

#include "Settings/Settings.h"
#include "Types/SE3.h"
#include "Types/Intrinsics.h"
#include "Types/GrayscaleImage.h"
#include "Types/BGRAImage.h"
#include "Types/Color.h"
#include "Features/BRISK/BRISK.h"

namespace ACGT
{
    class Camera {
        
    public:
        
        typedef long ID;
        
        Camera();
        
        Camera(Camera&& other);
        Camera& operator=(Camera&& other);
        
        // Constructor that performs a deep copy of image
        Camera(const GrayscaleImage& image, const Intrinsics& intrinsics);
        
        // Move constructor that takes ownership from image
        Camera(GrayscaleImage&& image, const Intrinsics& intrinsics);
        
        // Constructor that creates a grayscale image from an BGRAImage.
        // If storeColorImage is true, the Camera makes a deep copy of the BGRAImage, which allows to query the pixel's colour values.
        // If storeColorImage is false, the BGRAImage is neither copied or stored.
        Camera(const BGRAImage& image, const Intrinsics& intrinsics, const bool& storeColorImage = false);
        
        const long& id() const;
        const GrayscaleImage& image() const;
        const Intrinsics& intrinsics() const;
        SE3& pose();
        const SE3& pose() const;
        SE3& poseOpt();
        const SE3& poseOpt() const;
        Eigen::Vector3d& gravity();
        const std::vector<Feature>& features() const;
        const std::vector<BRISKDescriptor<Settings::DescriptorSize>>& descriptors() const;
        std::vector<Feature>& features();
        std::vector<BRISKDescriptor<Settings::DescriptorSize>>& descriptors();
        
        const std::vector<long>& pointReferences() const;
        void addPointReference(const int& featureIdx, const long& pointID);
        void removePointReference(const int& featureIdx);
        void markUnstable(const int& featureIdx);
        
        void extractFeatures(const uint8_t& threshold, const bool& describe = false);
        void extractDescriptors();
    
        bool hasColorImage() const;
        Color color(const float& x, const float& y) const;
        
    private:
        
        ID _id;
        GrayscaleImage _image;
        Intrinsics _intrinsics;
        SE3 _pose;
        SE3 _poseOpt;
        Eigen::Vector3d _gravity;
        std::shared_ptr<const BGRAImage> _colorImage;
        
        std::vector<Feature> _features;
        std::vector<BRISKDescriptor<Settings::DescriptorSize>> _descriptors;
        
        std::vector<long> _pointReferences;
        
        static long _idCreater;
    };
}

#endif /* defined(__pose_estimation_bmvc__Camera__) */
