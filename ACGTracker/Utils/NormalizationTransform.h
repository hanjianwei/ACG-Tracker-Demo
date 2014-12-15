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

#ifndef __ACG_Tracker_Demo__NormalizationTransform__
#define __ACG_Tracker_Demo__NormalizationTransform__

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Types/SE3.h"

namespace ACGT
{
    class NormalizationTransform
    {
        
    public:
        
        NormalizationTransform(const std::vector<Eigen::Vector3d>& data);
        
        Eigen::Vector3d normalize(const Eigen::Vector3d& other) const;
        SE3 normalize(const SE3& other) const ;
        
        Eigen::Vector3d denormalize(const Eigen::Vector3d& other)const ;
        SE3 denormalize(const SE3& other) const;
        
    private:
        
        Eigen::Affine3d _transform;
        Eigen::Affine3d _inverse;
    };
}

#endif /* defined(__ACG_Tracker_Demo__NormalizationTransform__) */
