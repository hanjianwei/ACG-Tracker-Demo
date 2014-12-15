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

#ifndef ACGT_EulerAngles_h
#define ACGT_EulerAngles_h

#include <Eigen/Core>

namespace ACGT
{
    /**
     * Class for Euler Angles that factorize a rotation matrix R as follows:
     * 
     * R = Rr * Rp * Ry
     *
     * where:
     *
     * Rr : Rotation matrix that rotates around the z-axis (forward axis, roll)
     * Rp : Rotation matrix that rotates around the x-axis (right axis, pitch)
     * Ry : Rotation matrix that rotates around the y-axis (upward axis, yaw)
     *
     **/
    
    class EulerAngles {
        
    public:
        
        EulerAngles() : _roll(0.0), _pitch(0.0), _yaw(0.0) {};
        EulerAngles(const double roll, const double pitch, const double yaw) : _roll(roll), _pitch(pitch), _yaw(yaw) {};
        EulerAngles(const Eigen::Matrix3d R);
        EulerAngles(const Eigen::Vector3d gravity);
        
        double& roll();
        const double& roll() const;
        double& pitch();
        const double& pitch() const;
        double& yaw();
        const double& yaw() const;
        
        Eigen::Matrix3d matrix();
        
    private:
        
        double _roll;
        double _pitch;
        double _yaw;
    };
}

#endif
