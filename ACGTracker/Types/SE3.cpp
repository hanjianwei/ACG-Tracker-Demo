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

#include "SE3.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ACGT
{
    Eigen::Matrix3d& SE3::R()
    {
        return _R;
    }
    
    Eigen::Vector3d& SE3::t()
    {
        return _t;
    }
    
    const Eigen::Matrix3d& SE3::R() const
    {
        return _R;
    }
    
    const Eigen::Vector3d& SE3::t() const
    {
        return _t;
    }
    
    Eigen::Affine3d SE3::affine() const
    {
        Eigen::Affine3d affine3d;
        affine3d.linear() = _R;
        affine3d.translation() = _t;
        return affine3d;
    }
    
    Eigen::Vector3d SE3::C() const
    {
        return -_R.transpose()*_t;
    }
}
