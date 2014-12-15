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

#ifndef __ACG_Tracker_Demo__OASTDetection9_16__
#define __ACG_Tracker_Demo__OASTDetection9_16__

#include "Features/FeatureDetection.h"
#include "Features/Feature.h"
#include "Types/GrayscaleImage.h"

namespace ACGT
{
    class OASTDetection9_16 : FeatureDetection
    {
        
    public:
        
        OASTDetection9_16();
        OASTDetection9_16(GrayscaleImage& image, const uint8_t& threshold = 40);
        
        void setImage(GrayscaleImage& image);
        
        uint8_t& threshold();
        const uint8_t& threshold() const;
        
        std::vector<Feature> detect();
        std::vector<Feature> detect(const ImageRect& rect);
        uint8_t cornerScore(const int& x, const int& y);
        
    private:
        
        GrayscaleImage _image;
        uint8_t _threshold;
        
        int_fast16_t _offset0;
        int_fast16_t _offset1;
        int_fast16_t _offset2;
        int_fast16_t _offset3;
        int_fast16_t _offset4;
        int_fast16_t _offset5;
        int_fast16_t _offset6;
        int_fast16_t _offset7;
        int_fast16_t _offset8;
        int_fast16_t _offset9;
        int_fast16_t _offset10;
        int_fast16_t _offset11;
        int_fast16_t _offset12;
        int_fast16_t _offset13;
        int_fast16_t _offset14;
        int_fast16_t _offset15;
    };
}

#endif /* defined(__ACG_Tracker_Demo__OASTDetection9_16__) */
