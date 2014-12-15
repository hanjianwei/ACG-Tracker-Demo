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

#include <cfloat>

#include "EulerAngles.h"

namespace ACGT
{
    EulerAngles::EulerAngles(const Eigen::Matrix3d R)
    {
        _pitch = atan2(-R(2,1),sqrt(R(1,1)*R(1,1)+R(0,1)*R(0,1)));
        
        // check for singularities:
        if (fabs((fabs(_pitch)-M_PI/2.0)) < DBL_EPSILON)
        {
            // Gimbal Locks
            _roll = 0.0;
            if (_pitch > 0.0) {
                _yaw = atan2(R(2,0), -R(2,1));
            } else {
                _yaw = atan2(-R(2,0), R(2,1));
            }
        } else
        {
            // no singularities
            _yaw = atan2(-R(2,0)/cos(_pitch), R(2,2)/cos(_pitch));
            _roll = atan2(R(0,1)/cos(_pitch), R(1,1)/cos(_pitch));
        }
    }
    
    EulerAngles::EulerAngles(const Eigen::Vector3d gravity)
    {
        // _yaw cannot be computed from gravity, set to zero
        _yaw = 0.0;
        
        // Compute _roll and _pitch
        _pitch = atan2(gravity.z(), sqrt(gravity.x()*gravity.x()+gravity.y()*gravity.y()));
        const double cosP = cos(_pitch);
        if (cosP == 0) {
            // gimbal lock
            _roll = 0.0;
        } else {
            _roll = atan2(-gravity.x()/cosP, -gravity.y()/cosP);
        }
    }
    
    double& EulerAngles::roll()
    {
        return _roll;
    }
    
    const double& EulerAngles::roll() const
    {
        return _roll;
    }
    
    double& EulerAngles::pitch()
    {
        return _pitch;
    }
    
    const double& EulerAngles::pitch() const
    {
        return _pitch;
    }
    
    double& EulerAngles::yaw()
    {
        return _yaw;
    }
    
    const double& EulerAngles::yaw() const
    {
        return _yaw;
    }
    
    Eigen::Matrix3d EulerAngles::matrix()
    {
        const double cosR = cos(_roll);
        const double sinR = sin(_roll);
        const double cosP = cos(_pitch);
        const double sinP = sin(_pitch);
        const double cosY = cos(_yaw);
        const double sinY = sin(_yaw);
        
        Eigen::Matrix3d R;
        R <<  cosR*cosY-sinP*sinR*sinY, cosP*sinR, cosR*sinY+cosY*sinP*sinR,
        -cosY*sinR-cosR*sinP*sinY, cosP*cosR, cosR*cosY*sinP-sinR*sinY,
        -cosP*sinY,     -sinP,                cosP*cosY;
        return R;
    }
}
