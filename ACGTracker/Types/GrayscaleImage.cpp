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

#include "GrayscaleImage.h"
#include "RGBImage.h"
#include "RGBAImage.h"
#include "BGRAImage.h"

namespace ACGT
{
    Image<int> GrayscaleImage::integralImage() const
    {
        Image<int> integralImage(_width,_height);
        
        const uint8_t *grayscaleIterator = _data;
        int *integralIterator = integralImage[0];
        
        int rowIntegral = 0;
        
        // the first row
        for (int x=0; x<_width; x++) {
            rowIntegral += grayscaleIterator[x];
            integralIterator[x] = rowIntegral;
        }
        grayscaleIterator += _width;
        integralIterator += _width;
        
        // the other rows
        for (int y=1; y<_height; y++) {
            rowIntegral = 0;
            for (int x=0; x<_width; x++) {
                rowIntegral += grayscaleIterator[x];
                integralIterator[x] = integralIterator[x-_width]+rowIntegral;
            }
            grayscaleIterator += _width;
            integralIterator += _width;
        }
        
        return integralImage;
    }
}