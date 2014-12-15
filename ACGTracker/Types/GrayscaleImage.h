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

#ifndef ACG_Tracker_Demo_GrayscaleImage_h
#define ACG_Tracker_Demo_GrayscaleImage_h

#include <memory>

#include "Image.h"

namespace ACGT
{
    class GrayscaleImage : public Image<uint8_t>
    {
        
    public:
        
        GrayscaleImage() : Image<uint8_t>() {};
        GrayscaleImage(const GrayscaleImage& other) : Image<uint8_t>(other) {};
        GrayscaleImage(GrayscaleImage&& other) : Image<uint8_t>(std::move(other)) {};
        GrayscaleImage(uint8_t *data, const int& width, const int& height) : Image<uint8_t>(data,width,height) {};
        GrayscaleImage(const int& width, const int& height) : Image<uint8_t>(width,height) {};
        
        GrayscaleImage& operator=(const GrayscaleImage& other)
        {
            Image<uint8_t>::operator=(other);
            return *this;
        }
        GrayscaleImage& operator=(GrayscaleImage&& other)
        {
            Image<uint8_t>::operator=(std::move(other));
            return *this;
        }
        
        Image<int> integralImage() const;
    };
}

#endif
