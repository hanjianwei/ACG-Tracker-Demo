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

#include "NormalizationTransform.h"

namespace ACGT
{
    NormalizationTransform::NormalizationTransform(const std::vector<Eigen::Vector3d>& data)
    {
        if (data.size() < 3)
        {
            _transform.linear() = Eigen::Matrix3d::Identity();
            _transform.translation() = Eigen::Vector3d::Zero();
            return;
        }
        
        // Compute center of gravity of map points
        Eigen::Vector3d cog = Eigen::Vector3d::Zero();
        for (const auto& it : data)
            cog += it;
        cog /= (double)data.size();
        
        // Compute average scale of map points
        double scale = 0.0;
        for (const auto& it : data)
            scale += (it-cog).norm();
        scale /= (double)data.size();
        
        _transform.linear() = sqrt(2.0)/scale*Eigen::Matrix3d::Identity();
        _transform.translation() = -sqrt(2.0)/scale*cog;
        
        _inverse = _transform.inverse();
    }
    
    Eigen::Vector3d NormalizationTransform::normalize(const Eigen::Vector3d& point) const
    {
        return _transform*point;
    }
    
    SE3 NormalizationTransform::normalize(const SE3& pose) const
    {
        Eigen::Affine3d normalizedAffine = _transform.linear()*pose.affine()*_inverse;
        SE3 normalizedPose;
        normalizedPose.R() = normalizedAffine.linear();
        normalizedPose.t() = normalizedAffine.translation();
        return normalizedPose;
    }
    
    Eigen::Vector3d NormalizationTransform::denormalize(const Eigen::Vector3d& point) const
    {
        return _inverse*point;
    }
    
    SE3 NormalizationTransform::denormalize(const SE3& pose) const
    {
        Eigen::Affine3d denormalizedAffine = _transform.linear().inverse()*pose.affine()*_transform;
        SE3 denormalizedPose;
        denormalizedPose.R() = denormalizedAffine.linear();
        denormalizedPose.t() = denormalizedAffine.translation();
        return denormalizedPose;
    }
}
