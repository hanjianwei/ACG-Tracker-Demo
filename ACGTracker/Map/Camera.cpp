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

#include <utility>
#include <vector>

#include "Camera.h"
#include "Types/SE3.h"
#include "Features/BRISK/BRISK.h"

namespace ACGT
{
    long Camera::_idCreater = 1;
    
    // Default constructor, initializes empty camera (useful for class members of type Camera)
    Camera::Camera() : _intrinsics(Intrinsics(0.0f, 0.0f, 0.0f))
    {
        _id = _idCreater++;
        _image = GrayscaleImage();
    }
    
    Camera::Camera(Camera&& other) : _intrinsics(other.intrinsics())
    {
        std::swap(_id, other._id);
        std::swap(_image, other._image);
        std::swap(_pose, other._pose);
        std::swap(_gravity, other._gravity);
        std::swap(_features, other._features);
        std::swap(_descriptors, other._descriptors);
        std::swap(_colorImage, other._colorImage);
        std::swap(_pointReferences, other._pointReferences);
    }
    
    Camera& Camera::operator=(Camera&& other)
    {
        std::swap(_id, other._id);
        std::swap(_image, other._image);
        std::swap(_intrinsics,other._intrinsics);
        std::swap(_pose, other._pose);
        std::swap(_gravity, other._gravity);
        std::swap(_features, other._features);
        std::swap(_descriptors, other._descriptors);
        std::swap(_colorImage, other._colorImage);
        std::swap(_pointReferences, other._pointReferences);
        return *this;
    }
    
    Camera::Camera(const GrayscaleImage& image, const Intrinsics& intrinsics) : _intrinsics(intrinsics)
    {
        _id = _idCreater++;
        _image = image;
    }
    
    Camera::Camera(GrayscaleImage&& image, const Intrinsics& intrinsics) : _intrinsics(intrinsics)
    {
        _id = _idCreater++;
        _image = std::move(image);
    }
    
    Camera::Camera(const BGRAImage& image, const Intrinsics& intrinsics, const bool& storeColorImage) : _intrinsics(intrinsics)
    {
        _id = _idCreater++;
        _image = image.grayscaleImage();
        if (storeColorImage)
            _colorImage.reset(new BGRAImage(image));
    }
    
    const long& Camera::id() const
    {
        return _id;
    }
    
    const GrayscaleImage& Camera::image() const
    {
        return _image;
    }
    
    const Intrinsics& Camera::intrinsics() const
    {
        return _intrinsics;
    }
    
    SE3& Camera::pose()
    {
        return _pose;
    }
    
    const SE3& Camera::pose() const
    {
        return _pose;
    }
    
    SE3& Camera::poseOpt()
    {
        return _poseOpt;
    }
    
    const SE3& Camera::poseOpt() const
    {
        return _poseOpt;
    }
    
    Eigen::Vector3d& Camera::gravity()
    {
        return _gravity;
    }
    
    const std::vector<Feature>& Camera::features() const
    {
        return _features;
    }
    
    const std::vector<BRISKDescriptor<Settings::DescriptorSize>>& Camera::descriptors() const
    {
        return _descriptors;
    }
    
    std::vector<Feature>& Camera::features()
    {
        return _features;
    }
    
    std::vector<BRISKDescriptor<Settings::DescriptorSize>>& Camera::descriptors()
    {
        return _descriptors;
    }
    
    void Camera::extractFeatures(const uint8_t& threshold, const bool& describe)
    {
        _features.clear();
        _descriptors.clear();
        
        BRISKDetection detector(Settings::DetectorOctaves,threshold);
        detector.constructScaleSpace(_image);
        _features = detector.detect();
        if (describe)
        {
            BRISKDescription<Settings::DescriptorSize> descriptor;
            _descriptors = descriptor.extract(_image, _features);
        }
        _pointReferences.resize(_features.size(), 0); 
    }
    
    void Camera::extractDescriptors()
    {
        _descriptors.clear();
        BRISKDescription<Settings::DescriptorSize> descriptor;
        _descriptors = descriptor.extract(_image, _features);
        _pointReferences.resize(_features.size(), 0);
    }
    
    bool Camera::hasColorImage() const
    {
        return _colorImage != nullptr;
    }
    
    Color Camera::color(const float& x, const float& y) const
    {
        assert(_colorImage != nullptr);
        
        const int xUpperLeft = (int)x;
        const int yUpperLeft = (int)y;
        
        const int xUpperRight = (int)x+1;
        const int yUpperRight = (int)y;
        
        const int xLowerLeft = (int)x;
        const int yLowerLeft = (int)y+1;
        
        const int xLowerRight = (int)x+1;
        const int yLowerRight = (int)y+1;
        
        const float weightLeft = ceil(x)-x;
        const float weightRight = 1.0f-weightLeft;
        const float weightUpper = ceil(y)-y;
        const float weightLower = 1.0f-weightUpper;
        
        Color color;
        
        color.r() += weightLeft*weightUpper*(float)(*_colorImage)[yUpperLeft][xUpperLeft].r();
        color.r() += weightRight*weightUpper*(float)(*_colorImage)[yUpperRight][xUpperRight].r();
        color.r() += weightLeft*weightLower*(float)(*_colorImage)[yLowerLeft][xLowerLeft].r();
        color.r() += weightRight*weightLower*(float)(*_colorImage)[yLowerRight][xLowerRight].r();
        color.r() = std::min(color.r()/255.0f, 1.0f);
        
        color.g() += weightLeft*weightUpper*(float)(*_colorImage)[yUpperLeft][xUpperLeft].g();
        color.g() += weightRight*weightUpper*(float)(*_colorImage)[yUpperRight][xUpperRight].g();
        color.g() += weightLeft*weightLower*(float)(*_colorImage)[yLowerLeft][xLowerLeft].g();
        color.g() += weightRight*weightLower*(float)(*_colorImage)[yLowerRight][xLowerRight].g();
        color.g() = std::min(color.g()/255.0f, 1.0f);
        
        color.b() += weightLeft*weightUpper*(float)(*_colorImage)[yUpperLeft][xUpperLeft].b();
        color.b() += weightRight*weightUpper*(float)(*_colorImage)[yUpperRight][xUpperRight].b();
        color.b() += weightLeft*weightLower*(float)(*_colorImage)[yLowerLeft][xLowerLeft].b();
        color.b() += weightRight*weightLower*(float)(*_colorImage)[yLowerRight][xLowerRight].b();
        color.b() = std::min(color.b()/255.0f, 1.0f);
        
        return color;
    }
    
    const std::vector<long>& Camera::pointReferences() const
    {
        return _pointReferences;
    }
    
    void Camera::addPointReference(const int& featureIdx, const long& pointID)
    {
        assert(_pointReferences[featureIdx] == 0);
        _pointReferences[featureIdx] = pointID;
    }
    
    void Camera::removePointReference(const int& featureIdx)
    {
        assert(_pointReferences[featureIdx]);
        _pointReferences[featureIdx] = -1;
    }
    
    void Camera::markUnstable(const int& featureIdx)
    {
        assert(_pointReferences[featureIdx] <= 0);
        _pointReferences[featureIdx] = -1;
    }
}