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

#ifndef ACG_Tracker_Demo_Color_h
#define ACG_Tracker_Demo_Color_h

#include <array>

class Color
{
public:
    
    Color() : _rgb(std::array<float,3>()) {};
    Color(const Color& other) : _rgb(other._rgb) {};
    Color(const float& r,const float& g,const float& b) : _rgb(std::array<float,3>{r,g,b}) {};
    
    const float& r() const
    {
        return _rgb[0];
    }
    
    float& r()
    {
        return _rgb[0];
    }
    
    const float& g() const
    {
        return _rgb[1];
    }
    
    float& g()
    {
        return _rgb[1];
    }
    
    const float& b() const
    {
        return _rgb[2];
    }
    
    float& b()
    {
        return _rgb[2];
    }
    
    const std::array<float,3>& rgb() const
    {
        return _rgb;
    }
    
    std::array<float,3>& rgb()
    {
        return _rgb;
    }
    
private:
    
    std::array<float,3> _rgb;
};

#endif
