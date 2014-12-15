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

#include "Point3d.h"

namespace ACGT
{
    Eigen::Vector3d& Point3d::coordinates()
    {
        return _coordinates;
    }
    
    const Eigen::Vector3d& Point3d::coordinates() const
    {
        return _coordinates;
    }
    
    Color& Point3d::color()
    {
        return _color;
    }
    
    const Color& Point3d::color() const
    {
        return _color;
    }
    
    Eigen::Vector3d Point3d::reproject(const SE3& pose) const
    {
        Eigen::Vector3d NDCs = pose.R()*_coordinates+pose.t();
        NDCs /= NDCs.z();
        return NDCs;
    }
    
    Eigen::Vector2d Point3d::reproject(const SE3& pose, const Intrinsics& intrinsics) const
    {
        Eigen::Vector3d NDCs = pose.R()*_coordinates+pose.t();
        NDCs /= -NDCs.z();
        Eigen::Vector2d reprojection;
        reprojection.x() = NDCs.x()*intrinsics.f()+intrinsics.pX();
        reprojection.y() = NDCs.y()*intrinsics.f()+intrinsics.pY();
        return reprojection;
    }
}