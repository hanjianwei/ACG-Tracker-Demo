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

#ifndef __ACG_Tracker_Demo__BRISKDetection__
#define __ACG_Tracker_Demo__BRISKDetection__

#include <vector>

#include "Types/GrayscaleImage.h"
#include "Features/Feature.h"
#include "Features/FeatureDetection.h"
#include "BRISKLayer.h"

namespace ACGT
{
    class BRISKDetection : FeatureDetection
    {
        
    public:
        
        BRISKDetection() = delete;
        BRISKDetection(const BRISKDetection& other) = delete;
        BRISKDetection(BRISKDetection&& other) = delete;
        
        BRISKDetection& operator=(const BRISKDetection& other) = delete;
        BRISKDetection& operator=(BRISKDetection&& other) = delete;
        
        BRISKDetection(const int& numOctaves, const uint8_t& threshold = 40);
        
        uint8_t& threshold();
        const uint8_t& threshold() const;
        
        void constructScaleSpace(GrayscaleImage& image);
        void constructScaleSpace(GrayscaleImage&& image);
    
        std::vector<Feature> detect();
        std::vector<Feature> detect(const int& l);
        std::vector<Feature> detect(const int& l, ImageRect rect);
        
        static constexpr int maxOctaves = 4;
        
    private:
        
        bool _isMaxOnLayer(const int& l, const int& x, const int& y);
        bool _isMaxOnLayerAbove(const int& l, const int& x, const int& y, const uint8_t& score);
        bool _isMaxOnLayerBelow(const int& l, const int& x, const int& y, const uint8_t& score);
        float _maxScoreOnLayerAbove(const int& l, const int& x, const int& y, const uint8_t& threshold, bool& isMaximum, float& deltaX, float& deltaY);
        float _maxScoreOnLayerBelow(const int& l, const int& x, const int& y, const uint8_t& threshold, bool& isMaximum, float& deltaX, float& deltaY);
        float _maxScoreRefined2D(const int& l, const int& x, const int& y, float& deltaX, float& deltaY);
        float _maxScoreRefinedScaleOnOriginalImage(float& scaleRefined, const float& score1, const float& score0_75, const float& score1_5);
        float _maxScoreRefinedScaleOnOctave(float& scaleRefined, const float& score1, const float& score0_75, const float& score1_5);
        float _maxScoreRefinedScaleOnIntraOctave(float& scaleRefined, const float& score1, const float& score0_75, const float& score1_5);
        float _maxScoreRefined3D(const int& layer, const int& x, const int& y, bool& isMaximum, float& xRefined, float& yRefined, float& scaleRefined);
        
        uint8_t _threshold;
        int _numLayers;
        std::vector<BRISKLayer> _scaleSpace;
    };
}

#endif /* defined(__ACG_Tracker_Demo__BRISKDetection__) */
