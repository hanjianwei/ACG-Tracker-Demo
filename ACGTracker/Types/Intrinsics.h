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

#ifndef __ACG_Tracker_Demo__Intrinsics__
#define __ACG_Tracker_Demo__Intrinsics__

namespace ACGT
{
    class Intrinsics
    {
    public:
        
        Intrinsics(const Intrinsics& other) : _f(other._f), _pX(other._pX), _pY(other._pY) {};
        
        Intrinsics(double f, double pX, double pY) : _f(f), _pX(pX), _pY(pY) {};
        
        const double& f() const;
        const double& pX() const;
        const double& pY() const;
        
    private:
        
        double _f;  // focal length
        double _pX; // x coordinate of principal point
        double _pY; // y coordinate of principal point
    };
}

#endif /* defined(__ACG_Tracker_Demo__Intrinsics__) */
