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

#ifndef __ACG_Tracker_Demo__BRISKLayer__
#define __ACG_Tracker_Demo__BRISKLayer__

#include "Features/FeatureDetection.h"
#include "Features/Feature.h"
#include "Types/GrayscaleImage.h"
#include "Features/AGAST/AGAST.h"

namespace ACGT
{
    typedef enum SampleModeEnum
    {
        Halfsample,
        Twothirdsample
    } SampleMode;
    
    class BRISKLayer : public GrayscaleImage
    {
        
    public:
        
        BRISKLayer() = delete;
        BRISKLayer(const BRISKLayer& other) = delete;
        BRISKLayer(BRISKLayer&& other);
        BRISKLayer(const int& width, const int& height) = delete;
        
        BRISKLayer& operator=(const BRISKLayer& other) = delete;
        BRISKLayer& operator=(BRISKLayer&& other) = delete;
        
        BRISKLayer(uint8_t *data, const int& width, const int& height);
        BRISKLayer(GrayscaleImage& image);
        BRISKLayer(GrayscaleImage&& image);
        
        BRISKLayer(const BRISKLayer& layer, SampleMode mode);
        
        const GrayscaleImage& scores() const;
        const float& scale() const;
        const float& offset() const;
        
        std::vector<Feature> getAgastCorners(const uint8_t& threshold);
        std::vector<Feature> getAgastCorners(const uint8_t& threshold, const ImageRect& rect);
        uint8_t score(const int& x, const int& y, const uint8_t& threshold);
        uint8_t score5_8(const int& x, const int& y, const uint8_t& threshold);
        uint8_t score(const float& xf, const float& yf, const uint8_t& threshold, const float& targetScale);
        
    private:
        
        uint8_t _smoothedScore(const float& xf, const float& yf, const float& targetScale) const;
        
        void _halfsampleFromLayer(const BRISKLayer& layer);
        void _twothirdsampleFromLayer(const BRISKLayer& layer);
        
        GrayscaleImage _scores;
        float _scale;
        float _offset;
        OASTDetection9_16 _oastDetection9_16;
        AGASTDetection5_8 _agastDetection5_8;
    };
}

#endif /* defined(__ACG_Tracker_Demo__BRISKLayer__) */
