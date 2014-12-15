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

#ifndef __ACG_Tracker_Demo__Match__
#define __ACG_Tracker_Demo__Match__

#include <limits>
#include <iostream>

namespace ACGT
{
    class Match
    {
    public:
        
        Match() : _idx1(0), _idx2(0), _distance(std::numeric_limits<float>::infinity()) {};
        Match(const unsigned int& idx1, const unsigned int& idx2, const float& distance) : _idx1(idx1), _idx2(idx2), _distance(distance) {};
        
        unsigned int& idx1();
        unsigned int& idx2();
        float& distance();
        
        const unsigned int& idx1() const;
        const unsigned int& idx2() const;
        const float& distance() const;
        
        // Comparison of matches, the less the better
        bool operator<(const Match& m ) const;
        bool operator<=(const Match& m ) const;
        bool operator==(const Match& m ) const;
        
    private:
        
        unsigned int _idx1;
        unsigned int _idx2;
        float _distance;
    };
}

// Overload left shift operator for printing
inline std::ostream& operator<<(std::ostream& os, const ACGT::Match& match)
{
    os << match.idx1() << " " << match.idx2() << " " << match.distance() << " ";
    return os;
}


#endif /* defined(__ACG_Tracker_Demo__Match__) */
