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

#ifndef __ACG_Tracker_Demo__Point3d__
#define __ACG_Tracker_Demo__Point3d__

#include <Eigen/Core>

#include "Intrinsics.h"
#include "Color.h"
#include "SE3.h"

namespace ACGT
{
    class Point3d
    {
        
    public:
        
        Point3d() : _coordinates(Eigen::Vector3d::Zero()) {};
        Point3d(const Eigen::Vector3d& coordinates, const Color& color = Color()) : _coordinates(coordinates), _color(color) {};
        
        Eigen::Vector3d& coordinates();
        const Eigen::Vector3d& coordinates() const;
        
        Color& color();
        const Color& color() const;
        
        ////
        // Returns 3D normaliced device coordinates of the point
        Eigen::Vector3d reproject(const SE3& pose) const;
        
        ////
        // Returns 2D coordinates of the point reprojected onto the image
        Eigen::Vector2d reproject(const SE3& pose, const Intrinsics& intrinsics) const;
        
    protected:
        
        Eigen::Vector3d _coordinates;
        Color _color;
        
    };
}

#endif /* defined(__ACG_Tracker_Demo__Point3d__) */
