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

#include "PatchDescription.h"

namespace ACGT
{
    const uint8_t& PatchDescriptor::operator[](const int& idx) const
    {
        return _data[idx];
    }
    
    uint8_t& PatchDescriptor::operator[](const int& idx)
    {
        return _data[idx];
    }
    
    std::vector<PatchDescriptor> PatchDescription::extract(const GrayscaleImage& image, std::vector<Feature>& features) const
    {
        // Remove features that are too close to the border
        int numFilteredFeatures = 0;
        const float minXY = 4.5;
        const float maxX = image.width()-minXY-1.0;
        const float maxY = image.height()-minXY-1.0;
        for (const auto& f : features)
        {
            if (f.x() >= minXY && f.y() >= minXY && f.x() <= maxX && f.y() <= maxY)
                features[numFilteredFeatures++] = f;
        }
        features.resize(numFilteredFeatures);
        
        // allocate memory for descriptors:
        __block std::vector<PatchDescriptor> descriptors(numFilteredFeatures);
        
        const int numSkipPixels = image.width() - 8;
        
        // compute descriptors:
        for (int i=0; i<features.size(); ++i)
        {
            const int upperLeftX = (int)features[i].x()-4.0;
            const int upperLeftY = (int)features[i].y()-4.0;
            const int weightLeft = upperLeftX-features[i].x()+5.0;
            const int weightRight = 1.0-weightLeft;
            const int weightUpper = upperLeftY-features[i].y()+5.0;
            const int weightLower = 1.0-weightUpper;
            const int wUL = weightUpper*weightLeft;
            const int wUR = weightUpper*weightRight;
            const int wLL = weightLower*weightLeft;
            const int wLR = weightLower*weightRight;
            const uint8_t *pUL = image[upperLeftY]+upperLeftX;
            const uint8_t *pUR = image[upperLeftY]+upperLeftX+1;
            const uint8_t *pLL = image[upperLeftY+1]+upperLeftX;
            const uint8_t *pLR = image[upperLeftY+1]+upperLeftX+1;
            std::array<int, 16> patchSums{};
            for (int i=0; i<8; i++)
            {
                for (int j=0; j<8; j++)
                    patchSums[4*(i/2)+(j/2)] += wUL*(*pUL++)+wUR*(*pUR++)+wLL*(*pLL++)+wLR*(*pLR++);
                pUL += numSkipPixels;
                pUR += numSkipPixels;
                pLL += numSkipPixels;
                pLR += numSkipPixels;
            }
            for (int j=0; j<16; j++)
                descriptors[i][j] = 0.25*patchSums[j];
        }
        
        return descriptors;
    }
    
    bool PatchDescription::descriptable(const GrayscaleImage& image, const Feature& f) const
    {
        const float minXY = 4.5;
        const float maxX = image.width()-minXY-1.0;
        const float maxY = image.height()-minXY-1.0;
        return f.x() >= minXY && f.y() >= minXY && f.x() <= maxX && f.y() <= maxY;
    }
    
    PatchDescriptor PatchDescription::extract(const GrayscaleImage& image, const Feature& f) const
    {
        PatchDescriptor descriptor;
        
        const int numSkipPixels = image.width() - 8;
        
        // compute descriptor
        const int upperLeftX = (int)f.x()-4.0;
        const int upperLeftY = (int)f.y()-4.0;
        const int weightLeft = upperLeftX-f.x()+5.0;
        const int weightRight = 1.0-weightLeft;
        const int weightUpper = upperLeftY-f.y()+5.0;
        const int weightLower = 1.0-weightUpper;
        const int wUL = weightUpper*weightLeft;
        const int wUR = weightUpper*weightRight;
        const int wLL = weightLower*weightLeft;
        const int wLR = weightLower*weightRight;
        const uint8_t *pUL = image[upperLeftY]+upperLeftX;
        const uint8_t *pUR = image[upperLeftY]+upperLeftX+1;
        const uint8_t *pLL = image[upperLeftY+1]+upperLeftX;
        const uint8_t *pLR = image[upperLeftY+1]+upperLeftX+1;
        std::array<int, 16> patchSums{};
        for (int i=0; i<8; i++)
        {
            for (int j=0; j<8; j++)
                patchSums[4*(i/2)+(j/2)] += wUL*(*pUL++)+wUR*(*pUR++)+wLL*(*pLL++)+wLR*(*pLR++);
            pUL += numSkipPixels;
            pUR += numSkipPixels;
            pLL += numSkipPixels;
            pLR += numSkipPixels;
        }
        for (int j=0; j<16; j++)
            descriptor[j] = 0.25*patchSums[j];
        
        return descriptor;
    }
}