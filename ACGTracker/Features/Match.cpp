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

#include "Match.h"

namespace ACGT
{
    unsigned int& Match::idx1()
    {
        return _idx1;
    }
    
    unsigned int& Match::idx2()
    {
        return _idx2;
    }
    
    float& Match::distance()
    {
        return _distance;
    }
    
    const unsigned int& Match::idx1() const
    {
        return _idx1;
    }
    
    const unsigned int& Match::idx2() const
    {
        return _idx2;
    }
    
    const float& Match::distance() const
    {
        return _distance;
    }
    
    bool Match::operator<(const Match& m ) const
    {
        return _distance < m._distance;
    }
    
    bool Match::operator<=(const Match& m ) const
    {
        return _distance <= m._distance;
    }
    
    bool Match::operator==(const Match& m ) const
    {
        return _distance == m._distance;
    }
}

