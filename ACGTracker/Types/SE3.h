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

#ifndef pose_estimation_bmvc_SE3_h
#define pose_estimation_bmvc_SE3_h

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ACGT
{
    class SE3
    {
        
    public:
        
        Eigen::Matrix3d& R(); // rotation
        Eigen::Vector3d& t(); // translation
        
        const Eigen::Matrix3d& R() const; // rotation
        const Eigen::Vector3d& t() const; // translation
        
        Eigen::Affine3d affine() const;
        
        Eigen::Vector3d C() const; // center of projection
        
    private:
        
        Eigen::Matrix3d _R;
        Eigen::Vector3d _t;
    };
}

#endif
