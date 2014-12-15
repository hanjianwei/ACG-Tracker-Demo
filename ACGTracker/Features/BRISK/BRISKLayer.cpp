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

#include <vector>
#include <memory>
#include <arm_neon.h>

#include "BRISKLayer.h"
#include "Types/GrayscaleImage.h"
#include "Utils/NeonExtensions.h"
#include "Features/AGAST/AGAST.h"

namespace ACGT
{
    /**
     * Constructs BRISKLayer from raw image data.
     * The caller is responsible for freeing the data.
     */
    BRISKLayer::BRISKLayer(uint8_t *data, const int& width, const int& height) : GrayscaleImage(data,width,height)
    {
        _scores = GrayscaleImage(_width,_height);
        _scale = 1.0;
        _offset = 0.0;
        
        _oastDetection9_16.setImage(*this);
        _agastDetection5_8.setImage(*this);
    }
    
    /** 
     * Constructs a BRISKLayer from a grayscale image.
     * The layer performs a shallow copy of the grayscale image, which keeps ownership.
     */
    BRISKLayer::BRISKLayer(GrayscaleImage& image) : GrayscaleImage(image[0],image.width(),image.height())
    {
        _scores = GrayscaleImage(_width,_height);
        _scale = 1.0;
        _offset = 0.0;
        
        _oastDetection9_16.setImage(*this);
        _agastDetection5_8.setImage(*this);
    }
    
    /**
     * Constructs a BRISKLayer from a grayscale image rValue reference.
     * The layer performs a shallow copy of the grayscale image and takes its ownership.
     */
    BRISKLayer::BRISKLayer(GrayscaleImage&& image) : GrayscaleImage(std::move(image))
    {
        _scores = GrayscaleImage(_width,_height);
        _scale = 1.0;
        _offset = 0.0;
        
        _oastDetection9_16.setImage(*this);
        _agastDetection5_8.setImage(*this);
    }
    
    /**
     * Derive a layer from an existing layer (halfsampled or twothirdsampled)
     */
    BRISKLayer::BRISKLayer(const BRISKLayer& layer, SampleMode mode)
    {
        if (mode == Halfsample)
        {
            _halfsampleFromLayer(layer);
            _scale = layer.scale()*2.0;
            _offset = 0.5*_scale-0.5;
        }
        else if (mode == Twothirdsample)
        {
            _twothirdsampleFromLayer(layer);
            _scale = layer.scale()*1.5;
            _offset = 0.5*_scale-0.5;
        }
        _scores = GrayscaleImage(_width,_height);
      
        _oastDetection9_16.setImage(*this);
        _agastDetection5_8.setImage(*this);
    }
    
    BRISKLayer::BRISKLayer(BRISKLayer&& other) : GrayscaleImage(std::move(other))
    {
        _scores = std::move(other.scores());
        _scale = std::move(other.scale());
        _offset = std::move(other.offset());
        
        _oastDetection9_16.setImage(*this);
        _agastDetection5_8.setImage(*this);
    }
    
    const GrayscaleImage& BRISKLayer::scores() const
    {
        return _scores;
    }
    
    const float& BRISKLayer::scale() const
    {
        return _scale;
    }
    
    const float& BRISKLayer::offset() const
    {
        return _offset;
    }
    
    // Fast/Agast
    // wraps the agast class
    std::vector<Feature> BRISKLayer::getAgastCorners(const uint8_t& threshold)
    {
        _oastDetection9_16.threshold() = threshold;
        std::vector<Feature> features = _oastDetection9_16.detect();
        
        // also write scores
        for(Feature& feature : features)
        {
            _scores[(int)feature.y()][(int)feature.x()] = _oastDetection9_16.cornerScore(feature.x(), feature.y());
        }
        
        return features;
    }
    
    std::vector<Feature> BRISKLayer::getAgastCorners(const uint8_t& threshold, const ImageRect& rect)
    {
        _oastDetection9_16.threshold() = threshold;
        std::vector<Feature> features = _oastDetection9_16.detect(rect);
        
        // also write scores
        for(Feature& feature : features)
        {
            _scores[(int)feature.y()][(int)feature.x()] = _oastDetection9_16.cornerScore(feature.x(), feature.y());
        }
        
        return features;
    }
    
    uint8_t BRISKLayer::score(const int& x, const int& y, const uint8_t& threshold)
    {
        if( x<3 || y<3 )
            return 0;
        if( x >= _width-3 || y >= _height-3)
            return 0;
        uint8_t score = _scores[y][x];
        if(score>2)
            return score;
        
        _oastDetection9_16.threshold() = threshold-1;
        score = _oastDetection9_16.cornerScore(x, y);
        if (score<threshold)
            score = 0;
        
        _scores[y][x] = score;
        return score;
    }
    
    uint8_t BRISKLayer::score5_8(const int& x, const int& y, const uint8_t& threshold)
    {
        if( x<2 || y<2 ) 
            return 0;
        if( x >= _width-2 || y >= _height-2)
            return 0;
        
        _agastDetection5_8.threshold() = threshold-1;
        uint8_t score = _agastDetection5_8.cornerScore(x, y);
        if (score<threshold) 
            score = 0;
        
        return score;
    }
    
    uint8_t BRISKLayer::score(const float& xf, const float& yf, const uint8_t& threshold, const float& targetScale)
    {
        if(targetScale<=1.0)
        {
            // just do a bilinear interpolation inside the layer
            const int x = (int)xf;
            const int y = (int)yf;
        
            const float xOffset=(xf-(float)x);
            const float yOffset=(yf-(float)y);
            
            // get agast scores of the four neighboring pixels
            const uint8_t scoreUpperLeft = score(x, y, threshold);
            const uint8_t scoreUpperRight = score(x+1, y, threshold);
            const uint8_t scoreLowerLeft = score(x, y+1, threshold);
            const uint8_t scoreLowerRight = score(x+1, y+1, threshold);
            
            // interpolate score linear between upper pixels
            const int scoreUpper = scoreUpperLeft + xOffset * (scoreUpperRight-scoreUpperLeft);
            
            // interpolate score linear between lower pixels
            const int scoreLower = scoreLowerLeft + xOffset * (scoreLowerRight-scoreLowerLeft);
            
            // interpolate linear between upper and lower scores and round
            return scoreUpper + yOffset * (scoreLower-scoreUpper);
            
        } else
        {
            // this means we smooth the score over the quadratic smoothing area (floating point boundaries) surrounding (xf,yf) with a side length of imgScale/2
            const float sigmaHalf = targetScale/2.0;
            // the center of a pixel (x,y) is at (x+0.5,y+0.5), this has to be considered in the computation
            // compute the scores of all pixels in the smoothing area:
            for(int x = xf+0.5-sigmaHalf; x <= xf+0.5+sigmaHalf; x++)
            {
                for(int y = yf+0.5-sigmaHalf; y <= yf+0.5+sigmaHalf; y++)
                    score(x, y, threshold);
    
            }
            // get the smoothed value
            return _smoothedScore(xf, yf, targetScale);
        }
    }
    
    // access values of subpixel coordinates with scale-dependent floating point boundary mean box smoothing filter
    uint8_t BRISKLayer::_smoothedScore(const float& xf, const float& yf, const float& targetScale) const
    {
        // box area smaller than 1 pixel:
        if(targetScale <= 1.0)
        {
            // just do a bilinear interpolation inside the layer
            const int x = (int)xf;
            const int y = (int)yf;
            //multiply offsets with 1024 so that we can use integer arithmetic
            const int xOffset=(xf-x)*1024;
            const int yOffset=(yf-y)*1024;
            // get values of the four neighboring pixels
            const uint8_t *scoreData = &_scores[y][x];
            const int valueUpperLeft = (int)*(scoreData++);
            const int valueUpperRight = (int)*scoreData;
            scoreData +=_width;
            const int valueLowerRight = (int)*(scoreData--);
            const int valueLowerLeft = (int)*scoreData;
            // interpolate value linear between upper pixels
            const int valueUpper = valueUpperLeft + xOffset * (valueUpperRight-valueUpperLeft);
            // interpolate value linear between lower pixels
            const int valueLower = valueLowerLeft + xOffset *  (valueLowerRight-valueLowerLeft);
            // interpolate linear between upper and lower values and round
            return 0xFF&((valueUpper + yOffset * (valueLower-valueUpper) + 512)/1024/1024);
        }
        
        // box area greater than one pixel:
        
        // compute sigmaHalf (half of the box side length):
        const float sigmaHalf=targetScale/2.0;
        
        //const int scaling2=float(scaling)*area/1024.0;
        
        // the center of a pixel (x,y) is at (x+0.5,y+0.5), this has to be considered in the computation!
        // calculate borders of box area
        const float xLeft = xf+0.5-sigmaHalf;
        const float xRight = xf+0.5+sigmaHalf;
        const float yTop = yf+0.5-sigmaHalf;
        const float yBottom = yf+0.5+sigmaHalf;
        
        // compute float boundary weights:
        // (fraction of the boundary pixels that belongs to the box area)
        const float weightXLeftF = 1.0 - xLeft + (int)xLeft;
        const float weightXRightF = xRight - (int)xRight;
        const float weightYTopF = 1.0 - yTop + (int)yTop;
        const float weightYBottomF = yBottom - (int)yBottom;
        
        // area size of box smooting kernel
        const float area=targetScale*targetScale;
        // scaling (1024*1024) to use integer arithmetic and normalization over box area:
        const int scaling = (int)(1048576.0/area);
        
        // scale boundary weights to int:
        const int weightXLeft = ((int)weightXLeftF)*scaling;
        const int weightXRight = ((int)weightXRightF)*scaling;
        const int weightYTop = ((int)weightYTopF)*scaling;
        const int weightYBottom = ((int)weightYBottomF)*scaling;
        
        //compute weights of the boundary corners:
        const int weightTopLeft = (weightXLeftF*weightYTopF)*scaling;
        const int weightTopRight = (weightXRightF*weightYTopF)*scaling;
        const int weightBottomLeft = (weightXLeftF*weightYBottomF)*scaling;
        const int weightBottomRight = (weightXRightF*weightYBottomF)*scaling;
        
        // compute average of the values in box area:
        const uint8_t *scoreData = &_scores[(int)yTop][(int)xLeft];
        const int dX = (int)xRight-(int)xLeft-1;
        const int dY = (int)yBottom-(int)yTop-1;
        int tempSum = 0;
        
        // top row:
        int average = (int)*(scoreData++) * weightTopLeft;
        const uint8_t *endTop = scoreData + dX;
        while (scoreData < endTop)
            tempSum += (int)*(scoreData++);
        
        average += tempSum * weightYTop;
        average += (int)*scoreData * weightTopRight;
        scoreData += _width - dX - 1;
        
        // middle rows:
        tempSum = 0;
        const uint8_t *endMidY = scoreData + dY * _width;
        while (scoreData < endMidY) {
            average += (int)*(scoreData++) * weightXLeft;
            const uint8_t *endMidX = scoreData + dX;
            while (scoreData < endMidX) {
                tempSum += (int)*(scoreData++);
            }
            average += (int)*scoreData * weightXRight;
            scoreData += _width - dX - 1;
        }
        average += tempSum * scaling;
        
        // bottom row:
        tempSum = 0;
        average += (int)*(scoreData++) * weightBottomLeft;
        const uint8_t *endBottom = scoreData + dX;
        while (scoreData < endBottom) {
            tempSum += (int)*(scoreData++);
        }
        average += tempSum * weightYBottom;
        average += (int)*scoreData * weightBottomRight;
        
        // rescale, round and return
        return 0xFF&((average+524288)/1048576);
    }
    
    void BRISKLayer::_halfsampleFromLayer(const BRISKLayer& layer)
    {
        // compute width and height of subsampled image
        _width = layer.width()/2;
        _height = layer.height()/2;
        
        // allocate memory for halfsampled image
        _data = new uint8_t[_width*_height];
        
        _owning = true;
        
        // we want to process 32 pixels at a time with ARM Neon SIMD instructions
        // check if we have left over columns
        const unsigned short leftoverCols = ((layer.width()%16)/2);// take care with border...
        const bool noLeftover = (layer.width()%16)==0; // note: leftoverCols can be zero but this still false...
        
        // data pointers
        uint8_t const *layerData = layer[0];
        uint8x16_t *pUpper = (uint8x16_t *)layerData;
        uint8x16_t *pLower = (uint8x16_t *)(layerData+layer.width());
        uint8x16_t *pDest = (uint8x16_t *)_data;
        uint8_t *pDestChar;
        
        // size in quadwords:
        const unsigned int size = (layer.width()*layer.height())/16;
        const unsigned int hSize = layer.width()/16; // horizontal size
        uint8x16_t *pEnd = pUpper + size;
        
        unsigned int row=0;
        const unsigned int end=hSize/2;
        bool quadwordLeftover;
        if(hSize%2==0)
            quadwordLeftover=false;
        else
            quadwordLeftover=true;
        
        while(pLower<pEnd)
        {
            for(unsigned int i=0; i<end;i++)
            {
                
                // compute pairwise horizontal averages of upper 32 bytes
                uint8x16x2_t upper = vld2q_u8((uint8_t *)pUpper);
                uint8x16_t resultUpper = NEONExtensions::vavgq_u8(upper.val[0], upper.val[1]);
                
                // compute pairwise horizontal averages of lower 32 bytes
                uint8x16x2_t lower = vld2q_u8((uint8_t *)pLower);
                uint8x16_t resultLower = NEONExtensions::vavgq_u8(lower.val[0], lower.val[1]);
                
                // compute pairwise vertical averages
                uint8x16_t result = NEONExtensions::vavgq_u8(resultUpper, resultLower);
                
                // store result in destination image
                vst1q_u8((uint8_t *)pDest, result);
                
                // increment pointers
                pUpper += 2;
                pLower += 2;
                pDest++;
                
            }
            // if we are not at the end of the row, do the rest:
            if(quadwordLeftover)
            {
                // compute pairwise horizontal averages of upper 16 bytes
                uint8x8x2_t upper = vld2_u8((uint8_t *)pUpper);
                uint8x8_t resultUpper = NEONExtensions::vavg_u8(upper.val[0], upper.val[1]);
                
                // compute pairwise horizontal averages of lower 16 bytes
                uint8x8x2_t lower = vld2_u8((uint8_t *)pLower);
                uint8x8_t resultLower = NEONExtensions::vavg_u8(lower.val[0], lower.val[1]);
                
                // compute pairwise vertical averages
                uint8x8_t result = NEONExtensions::vavg_u8(resultUpper, resultLower);
                
                // store result in destination array
                vst1_u8((uint8_t *)pDest, result);
                
                // increment pointers
                pUpper++;
                pLower++;
                pDestChar = (unsigned char *)pDest + 8;
                
            } else
                pDestChar = (unsigned char *)pDest;
            if(noLeftover)
            {
                row++;
                pDest = (uint8x16_t *)(_data + row*(layer.width()/2));
                pUpper = (uint8x16_t *)(layerData+2*row*layer.width());
                pLower = pUpper+hSize;
            } else
            {
                const uint8_t *pUpperChar=(uint8_t *)(pUpper);
                const uint8_t *pLowerChar=(uint8_t *)(pLower);
                for(unsigned int k=0; k<leftoverCols; k++){
                    unsigned short tmp = pUpperChar[2*k] + pUpperChar[2*k+1] +
                    pLowerChar[2*k] + pLowerChar[2*k+1];
                    *(pDestChar++) = (unsigned char)(tmp/4);
                }
                // done with the two rows:
                row++;
                pDest = (uint8x16_t *)(_data + row*(layer.width()/2));
                pUpper = (uint8x16_t *)(layerData+2*row*layer.width());
                pLower = (uint8x16_t *)(layerData+(2*row+1)*layer.width());
            }
        }
    }
    
    void BRISKLayer::_twothirdsampleFromLayer(const BRISKLayer& layer)
    {
        
        // compute width and height of subsampled image
        _width = (layer.width()/3)*2;
        _height = (layer.height()/3)*2;
        
        // allocate memory for halfsampled image
        _data = new uint8_t[_width*_height];
        
        _owning = true;
        
        // we want to process 32 pixels at a time with ARM Neon SIMD instructions
        // check if we have left over columns
        const unsigned short leftoverCols = ((layer.width()%24)/3);// take care with border...
        const bool noLeftover = (layer.width()%24)==0; // note: leftoverCols can be zero but this still false...
        
        // width of destination image
        const unsigned int destWidth = (layer.width()/3)*2;
        
        // data pointers
        uint8_t const *layerData = layer[0];
        uint8x16_t *pUpper = (uint8x16_t *)layerData;
        uint8x16_t *pMid = (uint8x16_t *)(layerData+layer.width());
        uint8x16_t *pLower = (uint8x16_t *)(layerData+2*layer.width());
        uint8x16_t *pDestUpper = (uint8x16_t *)_data;
        uint8x16_t *pDestLower = (uint8x16_t *)_data + layer.width();
        uint8_t *pUpperChar;
        uint8_t *pMidChar;
        uint8_t *pLowerChar;
        
        // size in 3*16 bytes:
        const unsigned int rowItCount = layer.width()/48; // number of iterations per row
        const unsigned int colItCount = layer.height()/3; // number of column iterations
        
        bool threeDWordsLeftover;
        if ((layer.width()/24)%2 == 0)
            threeDWordsLeftover = false;
        else
            threeDWordsLeftover = true;
        
        for (int j=0; j<colItCount; j++)
        {
            pDestUpper = (uint8x16_t *)(_data + 2*j*destWidth);
            pDestLower = (uint8x16_t *)(_data + (2*j+1)*destWidth);
            pUpper = (uint8x16_t *)(layerData+3*j*layer.width());
            pMid = (uint8x16_t *)(layerData+(3*j+1)*layer.width());
            pLower = (uint8x16_t *)(layerData+(3*j+2)*layer.width());
            for (int i=0; i<rowItCount; i++)
            {
                // compute horizontal averages of upper 48 bytes
                uint8x16x3_t upper = vld3q_u8((uint8_t *)pUpper);
                uint8x16_t resultLeftUpper = NEONExtensions::vavgq_u8(NEONExtensions::vavgq_u8(upper.val[0], upper.val[1]), upper.val[0]);
                uint8x16_t resultRightUpper = NEONExtensions::vavgq_u8(NEONExtensions::vavgq_u8(upper.val[2], upper.val[1]), upper.val[2]);
                
                // compute horizontal averages of mid 48 bytes
                uint8x16x3_t mid = vld3q_u8((uint8_t *)pMid);
                uint8x16_t resultLeftMid = NEONExtensions::vavgq_u8(NEONExtensions::vavgq_u8(mid.val[0], mid.val[1]), mid.val[0]);
                uint8x16_t resultRightMid = NEONExtensions::vavgq_u8(NEONExtensions::vavgq_u8(mid.val[2], mid.val[1]), mid.val[2]);
                
                // compute horizontal averages of lower 48 bytes
                uint8x16x3_t lower = vld3q_u8((uint8_t *)pLower);
                uint8x16_t resultLeftLower = NEONExtensions::vavgq_u8(NEONExtensions::vavgq_u8(lower.val[0], lower.val[1]), lower.val[0]);
                uint8x16_t resultRightLower = NEONExtensions::vavgq_u8(NEONExtensions::vavgq_u8(lower.val[2], lower.val[1]), lower.val[2]);
                
                // compute averages of uppers and mid
                uint8x16x2_t resultMidUppers =
                {
                    {
                        NEONExtensions::vavgq_u8(NEONExtensions::vavgq_u8(resultLeftUpper, resultLeftMid), resultLeftUpper),
                        NEONExtensions::vavgq_u8(NEONExtensions::vavgq_u8(resultRightUpper, resultRightMid), resultRightUpper)
                    }
                };
                
                // compute averages of lowers and mid
                uint8x16x2_t resultMidLowers =
                {
                    {
                        NEONExtensions::vavgq_u8(NEONExtensions::vavgq_u8(resultLeftLower, resultLeftMid), resultLeftLower),
                        NEONExtensions::vavgq_u8(NEONExtensions::vavgq_u8(resultRightLower, resultRightMid), resultRightLower)
                    }
                };
                
                // store results in destination image and increment destination image pointers
                vst2q_u8((uint8_t *)pDestUpper, resultMidUppers);
                vst2q_u8((uint8_t *)pDestLower, resultMidLowers);
                
                pDestUpper += 2;
                pDestLower += 2;
                
                // increment source image pointers
                pUpper += 3;
                pMid += 3;
                pLower += 3;
            }
            // if we are not at the end of the row, do the rest:
            if (threeDWordsLeftover)
            {
                // compute horizontal averages of upper 24 bytes
                uint8x8x3_t upper = vld3_u8((uint8_t *)pUpper);
                uint8x8_t resultLeftUpper = NEONExtensions::vavg_u8(NEONExtensions::vavg_u8(upper.val[0], upper.val[1]), upper.val[0]);
                uint8x8_t resultRightUpper = NEONExtensions::vavg_u8(NEONExtensions::vavg_u8(upper.val[2], upper.val[1]), upper.val[2]);
                
                // compute horizontal averages of mid 24 bytes
                uint8x8x3_t mid = vld3_u8((uint8_t *)pMid);
                uint8x8_t resultLeftMid = NEONExtensions::vavg_u8(NEONExtensions::vavg_u8(mid.val[0], mid.val[1]), mid.val[0]);
                uint8x8_t resultRightMid = NEONExtensions::vavg_u8(NEONExtensions::vavg_u8(mid.val[2], mid.val[1]), mid.val[2]);
                
                // compute horizontal averages of lower 24 bytes
                uint8x8x3_t lower = vld3_u8((uint8_t *)pLower);
                uint8x8_t resultLeftLower = NEONExtensions::vavg_u8(NEONExtensions::vavg_u8(lower.val[0], lower.val[1]), lower.val[0]);
                uint8x8_t resultRightLower = NEONExtensions::vavg_u8(NEONExtensions::vavg_u8(lower.val[2], lower.val[1]), lower.val[2]);
                
                // compute averages of upper and mid
                uint8x8x2_t resultMidUppers =
                {
                    {
                        NEONExtensions::vavg_u8(NEONExtensions::vavg_u8(resultLeftUpper, resultLeftMid), resultLeftUpper),
                        NEONExtensions::vavg_u8(NEONExtensions::vavg_u8(resultRightUpper, resultRightMid), resultRightUpper)
                    }
                };
                
                // compute averages of upper and mid
                uint8x8x2_t resultMidLowers =
                {
                    {
                        NEONExtensions::vavg_u8(NEONExtensions::vavg_u8(resultLeftLower, resultLeftMid), resultLeftLower),
                        NEONExtensions::vavg_u8(NEONExtensions::vavg_u8(resultRightLower, resultRightMid), resultRightLower)
                    }
                };
                
                // store results in destination image and increment destination image pointers
                vst2_u8((uint8_t *)pDestUpper, resultMidUppers);
                vst2_u8((uint8_t *)pDestLower, resultMidLowers);
                pDestUpper++;
                pDestLower++;
                
                // increment source image pointers
                pUpperChar = (uint8_t *)pUpper+24;
                pMidChar = (uint8_t *)pMid+24;
                pLowerChar = (uint8_t *)pLower+24;
            } else
            {
                // increment source image pointers
                pUpperChar = (uint8_t *)pUpper;
                pMidChar = (uint8_t *)pMid;
                pLowerChar = (uint8_t *)pLower;
            }
            if(!noLeftover)
            {
                uint8_t *pDestUpperChar = (uint8_t *)(pDestUpper);
                uint8_t *pDestLowerChar = (uint8_t *)(pDestLower);
                
                for(unsigned int k=0; k<leftoverCols; k++){
                    const uint16_t U0 = *(pUpperChar++);
                    const uint16_t U1 = *(pUpperChar++);
                    const uint16_t U2 = *(pUpperChar++);
                    const uint16_t M0 = *(pMidChar++);
                    const uint16_t M1 = *(pMidChar++);
                    const uint16_t M2 = *(pMidChar++);
                    const uint16_t L0 = *(pLowerChar++);
                    const uint16_t L1 = *(pLowerChar++);
                    const uint16_t L2 = *(pLowerChar++);
                    *(pDestUpperChar++) = (4*U0+2*(U1+M0)+M1)/9;
                    *(pDestUpperChar++) = (4*U2+2*(U1+M2)+M1)/9;
                    *(pDestLowerChar++) = (4*L0+2*(L1+M0)+M1)/9;
                    *(pDestLowerChar++) = (4*L2+2*(L1+M2)+M1)/9;
                }
            }
        }
    }
}