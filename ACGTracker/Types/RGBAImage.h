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

#ifndef __ACG_Tracker_Demo__RGBAImage__
#define __ACG_Tracker_Demo__RGBAImage__

#include <array>

#include "Image.h"
#include "GrayscaleImage.h"

namespace ACGT
{
    class RGBAPixel
    {
        
    public:
        
        RGBAPixel()
        {
            _data[0] = 0;
            _data[1] = 0;
            _data[2] = 0;
            _data[3] = 0;
        }
        
        RGBAPixel(const uint8_t r, const uint8_t g, const uint8_t b, const uint8_t a = 255)
        {
            _data[0] = r;
            _data[1] = g;
            _data[2] = b;
            _data[3] = a;
        }
        
        uint8_t& operator[](int&& idx)
        {
            return _data[idx];
        }
        
        const uint8_t& operator[](int&& idx) const
        {
            return _data[idx];
        }
        
        uint8_t& r()
        {
            return _data[0];
        }
        
        const uint8_t& r() const
        {
            return _data[0];
        }
        
        uint8_t& g()
        {
            return _data[1];
        }
        
        const uint8_t& g() const
        {
            return _data[1];
        }
        
        uint8_t& b()
        {
            return _data[2];
        }
        
        const uint8_t& b() const
        {
            return _data[2];
        }
        
        uint8_t& a()
        {
            return _data[3];
        }
        
        const uint8_t& a() const
        {
            return _data[3];
        }
        
    private:
        
        std::array<uint8_t,4> _data;
    };
    
    class RGBAImage : public Image<RGBAPixel>
    {
        
    public:
        
        RGBAImage() : Image<RGBAPixel>() {};
        RGBAImage(const RGBAImage& other) : Image<RGBAPixel>(other) {};
        RGBAImage(RGBAImage&& other) : Image<RGBAPixel>(std::move(other)) {};
        RGBAImage(RGBAPixel *data, const int& width, const int& height) : Image<RGBAPixel>(data,width,height) {};
        RGBAImage(const int& width, const int& height) : Image<RGBAPixel>(width,height) {};
        RGBAImage(const GrayscaleImage& grayscaleImage);
        
        RGBAImage& operator=(const RGBAImage& other)
        {
            Image<RGBAPixel>::operator=(other);
            return *this;
        }
        RGBAImage& operator=(RGBAImage&& other)
        {
            Image<RGBAPixel>::operator=(std::move(other));
            return *this;
        }
        
        GrayscaleImage grayscaleImage() const;
    };
}


#endif /* defined(__ACG_Tracker_Demo__RGBAImage__) */
