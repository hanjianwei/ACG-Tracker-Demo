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

#include "Feature.h"
#include "Map/Camera.h"

namespace ACGT
{
    float& Feature::x()
    {
        return _x;
    }
    
    float& Feature::y()
    {
        return _y;
    }
    
    int& Feature::layer()
    {
        return _layer;
    }
    
    float& Feature::response()
    {
        return _response;
    }
    
    int& Feature::score()
    {
        return _score;
    }
    
    float& Feature::scale()
    {
        return _scale;
    }
    
    float& Feature::angle()
    {
        return _angle;
    }
    
    const float& Feature::x() const
    {
        return _x;
    }
    
    const float& Feature::y() const
    {
        return _y;
    }
    
    const int& Feature::layer() const
    {
        return _layer;
    }
    
    const float& Feature::response() const
    {
        return _response;
    }
    
    const int& Feature::score() const
    {
        return _score;
    }
    
    const float& Feature::scale() const
    {
        return _scale;
    }
    
    const float& Feature::angle() const
    {
        return _angle;
    }
    
    Eigen::Vector2f Feature::coordinates() const
    {
        return Eigen::Vector2f(_x,_y);
    }
    
    Eigen::Vector3d Feature::NDCs(const Camera& camera) const
    {
        Eigen::Vector3d NDCs;
        NDCs << (_x-camera.intrinsics().pX())/camera.intrinsics().f(),
                ((double)camera.image().height()-1.0-_y-camera.intrinsics().pY())/camera.intrinsics().f(),
                -1.0;
        return NDCs;
    }
}