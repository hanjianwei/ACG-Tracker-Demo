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

#include "BGRAImage.h"
#include "GrayscaleImage.h"

#include <vector>
#include <dispatch/dispatch.h>

namespace ACGT
{
    
    BGRAImage::BGRAImage(const GrayscaleImage& grayscaleImage)
    {
        _data = new BGRAPixel[grayscaleImage.width()*grayscaleImage.height()];
        _width = grayscaleImage.width();
        _height = grayscaleImage.height();
        _owning = true;
        for (int row=0; row<_height; ++row)
        {
            for (int col=0; col<_width; ++col)
            {
                (*this)[row][col] = BGRAPixel(grayscaleImage[row][col], grayscaleImage[row][col], grayscaleImage[row][col]);
            }
        }
    }
    
    GrayscaleImage BGRAImage::grayscaleImage() const
    {
        const int numPixels = _width*_height;
        std::vector<uint8_t*> bgraPointers(24);
        bgraPointers[9] = (uint8_t*)_data;
        bgraPointers[18] = (uint8_t*)_data+2*numPixels;
        
        GrayscaleImage grayscaleImage(_width,_height);
        std::vector<uint8_t*> gsPointers(24);
        gsPointers[9] = grayscaleImage[0];
        gsPointers[18] = grayscaleImage[0]+numPixels/2;
        std::vector<int> numPixels_(24);
        numPixels_[9] = numPixels/2;
        numPixels_[18] = numPixels/2;
        
        dispatch_apply(2, dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_HIGH, 0), ^(size_t i) {
            
            asm volatile("lsr          %2, %2, #3      \n"
                         "# build the three constants: \n"
                         "mov         r4, #28          \n" // Blue channel multiplier
                         "mov         r5, #151         \n" // Green channel multiplier
                         "mov         r6, #77          \n" // Red channel multiplier
                         "vdup.8      d4, r4           \n"
                         "vdup.8      d5, r5           \n"
                         "vdup.8      d6, r6           \n"
                         "0:                           \n"
                         "# load 8 pixels:             \n"
                         "vld4.8      {d0-d3}, [%1]!   \n"
                         "# do the weight average:     \n"
                         "vmull.u8    q7, d0, d4       \n"
                         "vmlal.u8    q7, d1, d5       \n"
                         "vmlal.u8    q7, d2, d6       \n"
                         "# shift and store:           \n"
                         "vshrn.u16   d7, q7, #8       \n" // Divide q3 by 256 and store in the d7
                         "vst1.8      {d7}, [%0]!      \n"
                         "subs        %2, %2, #1       \n" // Decrement iteration count
                         "bne         0b               \n" // Repeat unil iteration count is not zero
                         :
                         : "r"(gsPointers[9*i+9]), "r"(bgraPointers[9*i+9]), "r"(numPixels_[9*i+9])
                         : "r4", "r5", "r6"
                         );
        });
        return grayscaleImage;
    }
}