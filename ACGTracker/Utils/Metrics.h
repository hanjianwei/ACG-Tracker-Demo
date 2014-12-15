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

#ifndef ACG_Tracker_Demo_Metrics_h
#define ACG_Tracker_Demo_Metrics_h

#include <Eigen/Core>

#include "Types/Intrinsics.h"

namespace ACGT
{
    namespace Metrics
    {
        // Computes the Sampsonus error of corresponding NDCs according to an essential matrix e
        static inline double sampsonusError(const Eigen::Vector3d& P, const Eigen::Vector3d& PDash, const Eigen::Matrix3d& E)
        {
            const Eigen::Vector3d EP = E*P;
            const Eigen::Vector3d PDashE = PDashE;
            const double PDashEP = PDash.dot(EP);
            
            return (PDashEP*PDashEP)/(EP.x()*EP.x()+EP.y()*EP.y()+PDashE.x()*PDashE.x()+PDashE.y()*PDashE.y());
        }
        
        // Computes the reprojection error in pixels of two points defined with respect to the same camera
        static inline float squaredReprojectionError(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2, const Intrinsics& intrinsics)
        {
            const float dX = intrinsics.f()*(P1.x()/P1.z()-P2.x()/P2.z());
            const float dY = intrinsics.f()*(P1.y()/P1.z()-P2.y()/P2.z());
            return dX*dX+dY*dY;
        }
    }
}

#endif
