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
#include <unordered_map>
#include <algorithm>
#include <Eigen/Core>
#include <dispatch/dispatch.h>

#include "BRISKDetection.h"
#include "BRISKLayer.h"

namespace ACGT
{
    BRISKDetection::BRISKDetection(const int& numOctaves, const uint8_t& threshold) : _threshold(threshold)
    {
        if (numOctaves == 0)
            _numLayers = 1;
        else if (numOctaves > maxOctaves)
            _numLayers = 2*maxOctaves;
        else
            _numLayers = 2*numOctaves;
    }
    
    uint8_t& BRISKDetection::threshold()
    {
        return _threshold;
    }
    
    const uint8_t& BRISKDetection::threshold() const
    {
        return _threshold;
    }
    
    void BRISKDetection::constructScaleSpace(GrayscaleImage& image)
    {
        _scaleSpace.push_back(BRISKLayer(image));
        
        if (_numLayers > 1) {
            _scaleSpace.push_back(BRISKLayer(_scaleSpace[0], SampleMode::Twothirdsample));
        }
        
        for (int i=2; i<_numLayers; i+=2)
        {
            _scaleSpace.push_back(BRISKLayer(_scaleSpace[i-2], SampleMode::Halfsample));
            _scaleSpace.push_back(BRISKLayer(_scaleSpace[i-1], SampleMode::Halfsample));
        }
    }
    
    void BRISKDetection::constructScaleSpace(GrayscaleImage&& image)
    {
        _scaleSpace.push_back(BRISKLayer(std::move(image)));
        
        if (_numLayers > 1) {
            _scaleSpace.push_back(BRISKLayer(_scaleSpace[0], SampleMode::Twothirdsample));
        }
        
        for (int i=2; i<_numLayers; i+=2)
        {
            _scaleSpace.push_back(BRISKLayer(_scaleSpace[i-2], SampleMode::Halfsample));
            _scaleSpace.push_back(BRISKLayer(_scaleSpace[i-1], SampleMode::Halfsample));
        }
    }
    
    std::vector<Feature> BRISKDetection::detect()
    {
        __block std::vector<Feature> features;
        __block std::unordered_map<BRISKLayer*, std::vector<std::vector<Feature>>> agastCorners;
        
        dispatch_semaphore_t semaphore = dispatch_semaphore_create(1);
        
        // go through the layers and compute AGAST corners and corresponding scores
        for (BRISKLayer& layer : _scaleSpace)
        {
            agastCorners[&layer] = std::vector<std::vector<Feature>>();
            dispatch_apply(16, dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_HIGH, 0), ^(size_t rectIdx)
                           {
                               std::vector<Feature> agastCornersRect = layer.getAgastCorners(_threshold,layer.rectangle(rectIdx));
                               dispatch_semaphore_wait(semaphore, DISPATCH_TIME_FOREVER);
                               agastCorners[&layer].push_back(std::move(agastCornersRect));
                               dispatch_semaphore_signal(semaphore);
                           });
        }
        
        if (_numLayers==1)
        {
            // just do a simple 2D refinement on the single layer
            BRISKLayer& layer = _scaleSpace[0];
            void (^detectionBlock)(size_t) = ^void (size_t rectIdx)
            {
                for (const Feature& corner : agastCorners[&layer][rectIdx])
                {
                    // check if its a 2D maximum:
                    if (!_isMaxOnLayer(0, corner.x(), corner.y()))
                        continue;
                    
                    
                    // refine maximum position
                    float deltaX, deltaY;
                    const float refinedScore = _maxScoreRefined2D(0, corner.x(), corner.y(), deltaX, deltaY);
                    
                    Feature feature;
                    feature.x() = corner.x()+deltaX;
                    feature.y() = corner.y()+deltaY;
                    feature.layer() = 0;
                    feature.response() = refinedScore;
                    feature.score() = layer.scores()[(int)corner.y()][(int)corner.x()];
                    feature.scale() = 1.0;
                    dispatch_semaphore_wait(semaphore, DISPATCH_TIME_FOREVER);
                    features.push_back(feature);
                    dispatch_semaphore_signal(semaphore);
                }
            };
            // Detect only in non-adjacent rectangles
            dispatch_apply(4, dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_HIGH, 0), ^(size_t rectIdx)
                           {
                               detectionBlock(2*rectIdx+(rectIdx/2)*4);
                           });
            dispatch_apply(4, dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_HIGH, 0), ^(size_t rectIdx)
                           {
                               detectionBlock(1+2*rectIdx+(rectIdx/2)*4);
                           });
            dispatch_apply(4, dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_HIGH, 0), ^(size_t rectIdx)
                           {
                               detectionBlock(4+2*rectIdx+(rectIdx/2)*4);
                           });
            dispatch_apply(4, dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_HIGH, 0), ^(size_t rectIdx)
                           {
                               detectionBlock(5+2*rectIdx+(rectIdx/2)*4);
                           });
        } else
        {
            for (int j=0; j<_numLayers; j++)
            {
                BRISKLayer& layer = _scaleSpace[j];
                if (j == _numLayers-1)
                {
                    // j is the last layer => no layer above exists
                    void (^detectionBlock)(size_t) = ^void (size_t rectIdx)
                    {
                        for (const Feature& corner : agastCorners[&layer][rectIdx])
                        {
                            // check if its a 2D maximum
                            if (!_isMaxOnLayer(j, corner.x(), corner.y()))
                                continue;
                            
                            // check if its also a maximum on the layer below
                            if (!_isMaxOnLayerBelow(j, corner.x(), corner.y(), layer.score(corner.x(), corner.y(), _threshold)))
                                continue;
                            
                            // refine maximum position
                            float deltaX, deltaY;
                            const float refinedScore = _maxScoreRefined2D(j, corner.x(), corner.y(), deltaX, deltaY);
                            
                            // store feature
                            Feature feature;
                            feature.x() = (corner.x()+deltaX)*layer.scale()+layer.offset();
                            feature.y() = (corner.y()+deltaY)*layer.scale()+layer.offset();
                            feature.layer() = j;
                            feature.response() = refinedScore;
                            feature.score() = layer.scores()[(int)corner.y()][(int)corner.x()];
                            feature.scale() = layer.scale();
                            dispatch_semaphore_wait(semaphore, DISPATCH_TIME_FOREVER);
                            features.push_back(feature);
                            dispatch_semaphore_signal(semaphore);
                        }
                    };
                    // Detect only in non-adjacent rectangles
                    dispatch_apply(4, dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_HIGH, 0), ^(size_t rectIdx)
                                   {
                                       detectionBlock(2*rectIdx+(rectIdx/2)*4);
                                   });
                    dispatch_apply(4, dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_HIGH, 0), ^(size_t rectIdx)
                                   {
                                       detectionBlock(1+2*rectIdx+(rectIdx/2)*4);
                                   });
                    dispatch_apply(4, dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_HIGH, 0), ^(size_t rectIdx)
                                   {
                                       detectionBlock(4+2*rectIdx+(rectIdx/2)*4);
                                   });
                    dispatch_apply(4, dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_HIGH, 0), ^(size_t rectIdx)
                                   {
                                       detectionBlock(5+2*rectIdx+(rectIdx/2)*4);
                                   });
                    
                } else
                {
                    void (^detectionBlock)(size_t) = ^void (size_t rectIdx)
                    {
                        for (const Feature& corner : agastCorners[&layer][rectIdx])
                        {
                            // check if its a 2D maximum
                            if (!_isMaxOnLayer(j, corner.x(), corner.y()))
                                continue;
                            
                            float refinedCornerX,refinedCornerY,refinedScale;
                            bool isMaximum;
                            const float refinedScore = _maxScoreRefined3D(j, corner.x(), corner.y(), isMaximum, refinedCornerX, refinedCornerY, refinedScale);
                            
                            if (!isMaximum)
                                continue;
                            
                            if (refinedScore > (float)_threshold)
                            {
                                // store feature
                                Feature feature;
                                feature.x() = refinedCornerX;
                                feature.y() = refinedCornerY;
                                feature.layer() = j;
                                feature.response() = refinedScore;
                                feature.score() = layer.scores()[(int)corner.y()][(int)corner.x()];
                                feature.scale() = refinedScale;
                                dispatch_semaphore_wait(semaphore, DISPATCH_TIME_FOREVER);
                                features.push_back(feature);
                                dispatch_semaphore_signal(semaphore);
                            }
                        }
                    };
                    // Detect only in non-adjacent rectangles
                    dispatch_apply(4, dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_HIGH, 0), ^(size_t rectIdx)
                                   {
                                       detectionBlock(2*rectIdx+(rectIdx/2)*4);
                                   });
                    dispatch_apply(4, dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_HIGH, 0), ^(size_t rectIdx)
                                   {
                                       detectionBlock(1+2*rectIdx+(rectIdx/2)*4);
                                   });
                    dispatch_apply(4, dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_HIGH, 0), ^(size_t rectIdx)
                                   {
                                       detectionBlock(4+2*rectIdx+(rectIdx/2)*4);
                                   });
                    dispatch_apply(4, dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_HIGH, 0), ^(size_t rectIdx)
                                   {
                                       detectionBlock(5+2*rectIdx+(rectIdx/2)*4);
                                   });
                }
            }
        }
        
        dispatch_release(semaphore);
        return features;
    }
    
    std::vector<Feature> BRISKDetection::detect(const int& l)
    {
        std::vector<Feature> features;
        
        if (l < 0 || l > _numLayers)
            return features;
        
        BRISKLayer& layer = _scaleSpace[l];
        
        // compute AGAST corners and corresponding scores
        std::vector<Feature> agastCorners = layer.getAgastCorners(_threshold);

        if (_numLayers==1)
        {
            // just do a simple 2D refinement on the single layer
            const BRISKLayer& layer = _scaleSpace[0];
            
            for (const Feature& corner : agastCorners)
            {
                // check if its a 2D maximum:
                if (!_isMaxOnLayer(0, corner.x(), corner.y()))
                    continue;
                
                
                // refine maximum position
                float deltaX, deltaY;
                const float refinedScore = _maxScoreRefined2D(0, corner.x(), corner.y(), deltaX, deltaY);
                
                Feature feature;
                feature.x() = corner.x()+deltaX;
                feature.y() = corner.y()+deltaY;
                feature.layer() = 0;
                feature.response() = refinedScore;
                feature.score() = layer.scores()[(int)corner.y()][(int)corner.x()];
                feature.scale() = 1.0;
                features.push_back(feature);
            }
        } else if (l == _numLayers-1)
        {
            // j is the last layer => no layer above exists
            for (const Feature& corner : agastCorners)
            {
                // check if its a 2D maximum
                if (!_isMaxOnLayer(l, corner.x(), corner.y()))
                    continue;
                
                // check if its also a maximum on the layer below
                if (!_isMaxOnLayerBelow(l, corner.x(), corner.y(), layer.score(corner.x(), corner.y(), _threshold)))
                    continue;
                
                // refine maximum position
                float deltaX, deltaY;
                const float refinedScore = _maxScoreRefined2D(l, corner.x(), corner.y(), deltaX, deltaY);
                
                // store feature
                Feature feature;
                feature.x() = (corner.x()+deltaX)*layer.scale()+layer.offset();
                feature.y() = (corner.y()+deltaY)*layer.scale()+layer.offset();
                feature.layer() = l;
                feature.response() = refinedScore;
                feature.score() = layer.scores()[(int)corner.y()][(int)corner.x()];
                feature.scale() = layer.scale();
                features.push_back(feature);
            }
        } else
        {
            for (const Feature& corner : agastCorners)
            {
                // check if its a 2D maximum
                if (!_isMaxOnLayer(l, corner.x(), corner.y()))
                    continue;
                
                float refinedCornerX,refinedCornerY,refinedScale;
                bool isMaximum;
                const float refinedScore = _maxScoreRefined3D(l, corner.x(), corner.y(), isMaximum, refinedCornerX, refinedCornerY, refinedScale);
                
                if (!isMaximum)
                    continue;
                
                if (refinedScore > (float)_threshold)
                {
                    // store feature
                    Feature feature;
                    feature.x() = refinedCornerX;
                    feature.y() = refinedCornerY;
                    feature.layer() = l;
                    feature.response() = refinedScore;
                    feature.score() = layer.scores()[(int)corner.y()][(int)corner.x()];
                    feature.scale() = refinedScale;
                    features.push_back(feature);
                }
                
            }
        }
    
        return features;
    }

    std::vector<Feature> BRISKDetection::detect(const int& l, ImageRect rect)
    {
        std::vector<Feature> features;
        
        if (l < 0 || l > _numLayers)
            return features;
        
        BRISKLayer& layer = _scaleSpace[l];
        
        // downsample rect to layer scale
        if (l%2 == 1)
        {
            rect.origin().x() = (rect.origin().x()/3)*2;
            rect.origin().y() = (rect.origin().y()/3)*2;
            rect.width() = (rect.width()/3)*2;
            rect.height() = (rect.height()/3)*2;
        }
        for (int i=l%2; i<l; i+=2)
        {
            rect.origin().x() = rect.origin().x()/2;
            rect.origin().y() = rect.origin().y()/2;
            rect.width() = rect.width()/2;
            rect.height() = rect.height()/2;
        }
        
        // compute AGAST corners and corresponding scores
        std::vector<Feature> agastCorners = layer.getAgastCorners(_threshold,rect);
        
        if (_numLayers==1)
        {
            // just do a simple 2D refinement on the single layer
            const BRISKLayer& layer = _scaleSpace[0];
            
            for (const Feature& corner : agastCorners)
            {
                // check if its a 2D maximum:
                if (!_isMaxOnLayer(0, corner.x(), corner.y()))
                    continue;
                
                
                // refine maximum position
                float deltaX, deltaY;
                const float refinedScore = _maxScoreRefined2D(0, corner.x(), corner.y(), deltaX, deltaY);
                
                Feature feature;
                feature.x() = corner.x()+deltaX;
                feature.y() = corner.y()+deltaY;
                feature.layer() = 0;
                feature.response() = refinedScore;
                feature.score() = layer.scores()[(int)corner.y()][(int)corner.x()];
                feature.scale() = 1.0;
                features.push_back(feature);
            }
        } else if (l == _numLayers-1)
        {
            // j is the last layer => no layer above exists
            for (const Feature& corner : agastCorners)
            {
                // check if its a 2D maximum
                if (!_isMaxOnLayer(l, corner.x(), corner.y()))
                    continue;
                
                // check if its also a maximum on the layer below
                if (!_isMaxOnLayerBelow(l, corner.x(), corner.y(), layer.score(corner.x(), corner.y(), _threshold)))
                    continue;
                
                // refine maximum position
                float deltaX, deltaY;
                const float refinedScore = _maxScoreRefined2D(l, corner.x(), corner.y(), deltaX, deltaY);
                
                // store feature
                Feature feature;
                feature.x() = (corner.x()+deltaX)*layer.scale()+layer.offset();
                feature.y() = (corner.y()+deltaY)*layer.scale()+layer.offset();
                feature.layer() = l;
                feature.response() = refinedScore;
                feature.score() = layer.scores()[(int)corner.y()][(int)corner.x()];
                feature.scale() = layer.scale();
                features.push_back(feature);
            }
        } else
        {
            for (const Feature& corner : agastCorners)
            {
                // check if its a 2D maximum
                if (!_isMaxOnLayer(l, corner.x(), corner.y()))
                    continue;
                
                float refinedCornerX,refinedCornerY,refinedScale;
                bool isMaximum;
                const float refinedScore = _maxScoreRefined3D(l, corner.x(), corner.y(), isMaximum, refinedCornerX, refinedCornerY, refinedScale);
                
                if (!isMaximum)
                    continue;
                
                if (refinedScore > (float)_threshold)
                {
                    // store feature
                    Feature feature;
                    feature.x() = refinedCornerX;
                    feature.y() = refinedCornerY;
                    feature.layer() = l;
                    feature.response() = refinedScore;
                    feature.score() = layer.scores()[(int)corner.y()][(int)corner.x()];
                    feature.scale() = refinedScale;
                    features.push_back(feature);
                }
                
            }
        }
        
        return features;
    }

    bool BRISKDetection::_isMaxOnLayer(const int& l, const int& x, const int& y)
    {
        const GrayscaleImage& scores = _scaleSpace[l].scores();

        const uint8_t *data = &scores[y][x];
        
        // decision tree
        uint8_t s00 = *data;
        data--;
        const uint8_t s_10=*data;
        if(s00<s_10) return false;
        data+=2;
        const uint8_t s10=*data;
        if(s00<s10) return false;
        data-=(scores.width()+1);
        const uint8_t s0_1=*data;
        if(s00<s0_1) return false;
        data+=2*scores.width();
        const uint8_t s01=*data;
        if(s00<s01) return false;
        data--;
        const uint8_t s_11=*data;
        if(s00<s_11) return false;
        data+=2;
        const uint8_t s11=*data;
        if(s00<s11) return false;
        data-=2*scores.width();
        const uint8_t s1_1=*data;
        if(s00<s1_1) return false;
        data-=2;
        const uint8_t s_1_1=*data;
        if(s00<s_1_1) return false;
        
        // check for equal scores in patch neighborhood
        Eigen::Vector2i deltas[8];
        int deltasSize = 0;
        if (s00==s_1_1) {
            deltas[deltasSize].x() = -1;
            deltas[deltasSize].y() = -1;
            deltasSize++;
        }
        if (s00==s0_1) {
            deltas[deltasSize].x() = 0;
            deltas[deltasSize].y() = -1;
            deltasSize++;
        }
        if (s00==s1_1) {
            deltas[deltasSize].x() = 1;
            deltas[deltasSize].y() = -1;
            deltasSize++;
        }
        if (s00==s_10) {
            deltas[deltasSize].x() = -1;
            deltas[deltasSize].y() = 0;
            deltasSize++;
        }
        if (s00==s10) {
            deltas[deltasSize].x() = 1;
            deltas[deltasSize].y() = 0;
            deltasSize++;
        }
        if (s00==s_11) {
            deltas[deltasSize].x() = -1;
            deltas[deltasSize].y() = 1;
            deltasSize++;
        }
        if (s00==s01) {
            deltas[deltasSize].x() = 0;
            deltas[deltasSize].y() = 1;
            deltasSize++;
        }
        if (s00==s11) {
            deltas[deltasSize].x() = 1;
            deltas[deltasSize].y() = 1;
            deltasSize++;
        }
        
        if (deltasSize != 0) {
            // multiple maximum candidates
            // apply gaussian smoothing to each candidate and decide according to the smoothed scores
            const int smoothedS00 = 4*s00+2*(s_10+s10+s0_1+s01)+s_1_1+s1_1+s_11+s11;
            for (int i=0; i<deltasSize; i++) {
                data = &scores[y+deltas[i].y()-1][x+deltas[i].x()-1];
                int smoothedCandidate = *data;
                data++;
                smoothedCandidate += (*data)<<1;
                data++;
                smoothedCandidate += *data;
                data += scores.width();
                smoothedCandidate += (*data)<<1;
                data--;
                smoothedCandidate += (*data)<<2;
                data--;
                smoothedCandidate += (*data)<<1;
                data += scores.width();
                smoothedCandidate += *data;
                data++;
                smoothedCandidate += (*data)<<1;
                data++;
                smoothedCandidate += *data;
                if (smoothedCandidate>smoothedS00) 
                {
                    return false;
                }
            }
        }
        return true;
    }
    
    /**
     * Project the surrounding 1x1 patch of (x,y) on the layer above and check if the score at (x,y) is bigger or equal than all scores in the projected patch
     **/
    bool BRISKDetection::_isMaxOnLayerAbove(const int& l, const int& x, const int& y, const uint8_t& score)
    {
        BRISKLayer& layerAbove = _scaleSpace[l+1];
        
        // floating point boundaries of the projected 1x1 patch in the layer above:
        float xAboveLeft;
        float xAboveRight;
        float yAboveTop;
        float yAboveBottom;
        
        if (l%2==0)
        {
            // on octave
            xAboveLeft = (float)(4*x-3)/6.0;
            xAboveRight = (float)(4*x+1)/6.0;
            yAboveTop = (float)(4*y-3)/6.0;
            yAboveBottom = (float)(4*y+1)/6.0;
        } else
        {
            // on intra-octave
            xAboveLeft = (float)(6*x-4)/8.0;
            xAboveRight = (float)(6*x+2)/8.0;
            yAboveTop = (float)(6*y-4)/8.0;
            yAboveBottom = (float)(6*y+2)/8.0;
        }
        
        // integer boundaries of the projected 1x1 patch in the layer above:
        const int xAL = (int)xAboveLeft+1;
        const int xAR = (int)xAboveRight+1;
        const int yAT = (int)yAboveTop+1;
        const int yAB = (int)yAboveBottom+1;
        
        // check if all scores in the projected patch are smaller than score:
        
        // first row:
        if (score < layerAbove.score(xAboveLeft, yAboveTop, 1, 1.0))
            return false;
    
        for (int i=xAL; i<xAR; i++)
        {
            if (score < layerAbove.score((float)i, yAboveTop, 1, 1.0))
                return false;
        }
        if (score < layerAbove.score(xAboveRight, yAboveTop, 1, 1.0))
            return false;
        
        // middle rows:
        for (int j=yAT; j<yAB; j++) {
            if (score < layerAbove.score(xAboveLeft, (float)j, 1, 1.0))
                return false;
            for (int i=xAL; i<xAR; i++)
            {
                if (score < layerAbove.score(i, j, 1))
                    return false;
            }
            if (score < layerAbove.score(xAboveRight, (float)j, 1, 1.0))
                return false;
        }
        
        // bottom row:
        if (score < layerAbove.score(xAboveLeft, yAboveBottom, 1, 1.0))
            return false;
        for (int i=xAL; i<xAR; i++) {
            if (score < layerAbove.score((float)i, yAboveBottom, 1, 1.0))
                return false;
        }
        if (score < layerAbove.score(xAboveRight, yAboveBottom, 1, 1.0))
            return false;
        
        // we are done, all scores in the projected patch are smaller than or equal score, return true
        return true;

    }
    
    bool BRISKDetection::_isMaxOnLayerBelow(const int& l, const int& x, const int& y, const uint8_t& score)
    {
        BRISKLayer& layerBelow = _scaleSpace[l-1];
        
        // floating point boundaries of the projected 1x1 patch in the layer below:
        float xBelowLeft;
        float xBelowRight;
        float yBelowTop;
        float yBelowBottom;
        
        if (l%2==0)
        {
            // on octave
            xBelowLeft = (float)(8*x-3)/6.0;
            xBelowRight = (float)(8*x+5)/6.0;
            yBelowTop = (float)(8*y-3)/6.0;
            yBelowBottom = (float)(8*y+5)/6.0;
        } else
        {
            // on intra-octave
            xBelowLeft = (float)(6*x-2)/4.0;
            xBelowRight = (float)(6*x+4)/4.0;
            yBelowTop = (float)(6*y-2)/4.0;
            yBelowBottom = (float)(6*y+4)/4.0;
        }
        
        // integer boundaries of the projected 1x1 patch in the layer below:
        const int xBL = (int)xBelowLeft+1;
        const int xBR = (int)xBelowRight+1;
        const int yBT = (int)yBelowTop+1;
        const int yBB = (int)yBelowBottom+1;
        
        // check if all scores in the projected patch are smaller than score:
        
        // first row:
        if (score < layerBelow.score(xBelowLeft, yBelowTop, 1, 1.0))
            return false;
        for (int i=xBL; i<xBR; i++)
        {
            if (score < layerBelow.score((float)i, yBelowTop, 1, 1.0))
                return false;
        }
        if (score < layerBelow.score(xBelowRight, yBelowTop, 1, 1.0))
            return false;
        
        // middle rows:
        for (int j=yBT; j<yBB; j++)
        {
            if (score < layerBelow.score(xBelowLeft, (float)j, 1, 1.0))
                return false;
            for (int i=xBL; i<xBR; i++)
            {
                if (score < layerBelow.score(i, j, 1))
                    return false;
            }
            if (score < layerBelow.score(xBelowRight, (float)j, 1, 1.0))
                return false;
        }
        
        // bottom row:
        if (score < layerBelow.score(xBelowLeft, yBelowBottom, 1, 1.0))
            return false;
        for (int i=xBL; i<xBR; i++)
        {
            if (score < layerBelow.score((float)i, yBelowBottom, 1, 1.0))
                return false;
        }
        if (score < layerBelow.score(xBelowRight, yBelowBottom, 1, 1.0))
            return false;
        
        // we are done, all scores in the projected patch are smaller than or equal score, return true
        return true;
    }
    
    float BRISKDetection::_maxScoreOnLayerAbove(const int& l, const int& x, const int& y, const uint8_t& candidate, bool& isMaximum, float& deltaX, float& deltaY)
    {
        isMaximum = false;
        BRISKLayer& layerAbove = _scaleSpace[l+1];
        
        // floating point boundaries of the projected 1x1 patch in the layer above:
        float xAboveLeft;
        float xAboveRight;
        float yAboveTop;
        float yAboveBottom;
        
        if (l%2==0)
        {
            // on octave
            xAboveLeft = (float)(4*x-3)/6.0;
            xAboveRight = (float)(4*x+1)/6.0;
            yAboveTop = (float)(4*y-3)/6.0;
            yAboveBottom = (float)(4*y+1)/6.0;
        } else
        {
            // on intra-octave
            xAboveLeft = (float)(6*x-4)/8.0;
            xAboveRight = (float)(6*x+2)/8.0;
            yAboveTop = (float)(6*y-4)/8.0;
            yAboveBottom = (float)(6*y+2)/8.0;
        }
        
        // integer boundaries of the projected 1x1 patch in the layer above:
        const int xAL = (int)xAboveLeft+1;
        const int xAR = (int)xAboveRight+1;
        const int yAT = (int)yAboveTop+1;
        const int yAB = (int)yAboveBottom+1;
        
        // coordinates of the current maximum in the projected patch and the maximum score:
        int maxX = xAL;
        int maxY = yAT;
        
        // check if all scores in the projected patch are smaller than score:
        
        // first row:
        float score = layerAbove.score(xAboveLeft, yAboveTop, 1, 1.0);
        if (candidate < score)
            return 0.0;
        float maxValue = score;

        for (int i=xAL; i<xAR; i++)
        {
            score = layerAbove.score((float)i, yAboveTop, 1, 1.0);
            if (candidate < score)
                return 0.0;
            if (maxValue < score)
            {
                maxValue = score;
                maxX = i;
            }
        }
        score = layerAbove.score(xAboveRight, yAboveTop, 1, 1.0);
        if (candidate < score)
            return 0.0;
        if (maxValue < score)
        {
            maxValue = score;
            maxX = xAR-1;
        }
        
        // middle rows:
        for (int j=yAT; j<yAB; j++)
        {
            score = layerAbove.score(xAboveLeft, (float)j, 1, 1.0);
            if (candidate < score)
                return 0.0;
            if (maxValue < score)
            {
                maxValue = score;
                maxX = xAL;
                maxY = j;
            }
            
            for (int i=xAL; i<xAR; i++)
            {
                score = layerAbove.score(i, j, 1);
                if (candidate < score)
                    return 0.0;
                if (maxValue < score)
                {
                    maxValue = score;
                    maxX = i;
                    maxY = j;
                }
                
            }
            score = layerAbove.score(xAboveRight, (float)j, 1, 1.0);
            if (candidate < score)
                return 0.0;
            if (maxValue < score)
            {
                maxValue = score;
                maxX = xAR-1;
                maxY = j;
            }
        }
        
        // bottom row:
        score = layerAbove.score(xAboveLeft, yAboveBottom, 1, 1.0);
        if (candidate < score)
            return 0.0;
        if (maxValue < score)
        {
            maxValue = score;
            maxX = xAL;
            maxY = yAB-1;
        }
        for (int i=xAL; i<xAR; i++)
        {
            score = layerAbove.score((float)i, yAboveBottom, 1, 1.0);
            if (candidate < score)
                return 0.0;
            if (maxValue < score)
            {
                maxValue = score;
                maxX = i;
                maxY = yAB-1;
            }
        }
        score = layerAbove.score(xAboveRight, yAboveBottom, 1, 1.0);
        if (candidate < score)
            return 0.0;
        if (maxValue < score)
        {
            maxValue = score;
            maxX = xAR-1;
            maxY = yAB-1;
        }
        // all scores in the projected patch are smaller than or equal score
        isMaximum = true;
        
        // refine maximum of above layer:
        float dXMaxAbove;
        float dYMaxAbove;
        float refinedMax = _maxScoreRefined2D(l+1, maxX, maxY, dXMaxAbove, dYMaxAbove);
     
        // project refined maximum coordinates to current layer:
        if(l%2==0)
        {
            // on octave
            deltaX=(((float)maxX+(dXMaxAbove))*6.0+1.0)/4.0-(float)x;
            deltaY=(((float)maxY+(dYMaxAbove))*6.0+1.0)/4.0-(float)y;
        } else {
            // on intra-octave
            deltaX=(((float)maxX+(dXMaxAbove))*8.0+1.0)/6.0-(float)x;
            deltaY=(((float)maxY+(dYMaxAbove))*8.0+1.0)/6.0-(float)y;
        }
        
        // saturate:
        bool returnRefined = true;
        if (deltaX > 1.0)
        {
            deltaX = 1.0;
            returnRefined = false;
        }
        if (deltaX < -1.0)
        {
            deltaX = -1.0;
            returnRefined = false;
        }
        if (deltaY > 1.0)
        {
            deltaY = 1.0;
            returnRefined = false;
        }
        if (deltaY < -1.0)
        {
            deltaY = -1.0;
            returnRefined = false;
        }
        
        if (returnRefined)
            return maxValue < refinedMax ? refinedMax : maxValue;
        
        return maxValue;
    }
    
    float BRISKDetection::_maxScoreOnLayerBelow(const int& l, const int& x, const int& y, const uint8_t& candidate, bool& isMaximum, float& deltaX, float& deltaY)
    {
        isMaximum = false;
        BRISKLayer& layerBelow = _scaleSpace[l-1];
        
        // floating point boundaries of the projected 1x1 patch in the layer below:
        float xBelowLeft;
        float xBelowRight;
        float yBelowTop;
        float yBelowBottom;
        
        if (l%2==0)
        {
            // on octave
            xBelowLeft = (float)(8*x-3)/6.0;
            xBelowRight = (float)(8*x+5)/6.0;
            yBelowTop = (float)(8*y-3)/6.0;
            yBelowBottom = (float)(8*y+5)/6.0;
        } else
        {
            // on intra-octave
            xBelowLeft = (float)(6*x-2)/4.0;
            xBelowRight = (float)(6*x+4)/4.0;
            yBelowTop = (float)(6*y-2)/4.0;
            yBelowBottom = (float)(6*y+4)/4.0;
        }
        
        // integer boundaries of the projected 1x1 patch in the layer below:
        const int xBL = (int)xBelowLeft+1;
        const int xBR = (int)xBelowRight+1;
        const int yBT = (int)yBelowTop+1;
        const int yBB = (int)yBelowBottom+1;
        
        // coordinates of the current maximum in the projected patch and the maximum score:
        int maxX = xBL;
        int maxY = yBT;
        
        // check if all scores in the projected patch are smaller than score:
        
        // first row:
        float score = layerBelow.score(xBelowLeft, yBelowTop, 1, 1.0);
        if (candidate < score)
            return 0.0;
        float maxValue = score;
        for (int i=xBL; i<xBR; i++)
        {
            score = layerBelow.score((float)i, yBelowTop, 1, 1.0);
            if (candidate < score)
                return 0.0;
            if (maxValue < score)
            {
                maxValue = score;
                maxX = i;
            }
        }
        score = layerBelow.score(xBelowRight, yBelowTop, 1, 1.0);
        if (candidate < score)
            return 0.0;
        if (maxValue < score)
        {
            maxValue = score;
            maxX = xBR-1;
        }
        
        // middle rows:
        for (int j=yBT; j<yBB; j++)
        {
            score = layerBelow.score(xBelowLeft, (float)j, 1, 1.0);
            if (candidate < score)
                return 0.0;
            if (maxValue < score)
            {
                maxValue = score;
                maxX = xBL;
                maxY = j;
            }
            for (int i=xBL; i<xBR; i++)
            {
                score = layerBelow.score(i, j, 1);
                if (candidate < score)
                    return 0.0;
                if (maxValue < score)
                {
                    maxValue = score;
                    maxX = i;
                    maxY = j;
                }
                // Apply gaussian smoothing:
                if (maxValue == score) {
                    const int smoothedMax =  layerBelow.score(maxX-1, maxY-1, 1)
                                            +layerBelow.score(maxX+1, maxY-1, 1)
                                            +layerBelow.score(maxX-1, maxY+1, 1)
                                            +layerBelow.score(maxX+1, maxY+1, 1)
                                         +2*(layerBelow.score(maxX-1, maxY, 1)
                                            +layerBelow.score(maxX+1, maxY, 1)
                                            +layerBelow.score(maxX, maxY-1, 1)
                                            +layerBelow.score(maxX, maxY+1, 1));
                    const int smoothedCandidate =  layerBelow.score(i-1, j-1, 1)
                                                  +layerBelow.score(i+1, j-1, 1)
                                                  +layerBelow.score(i-1, j+1, 1)
                                                  +layerBelow.score(i+1, j+1, 1)
                                               +2*(layerBelow.score(i-1, j, 1)
                                                  +layerBelow.score(i+1, j, 1)
                                                  +layerBelow.score(i, j-1, 1)
                                                  +layerBelow.score(i, j+1, 1));
                    if (smoothedCandidate > smoothedMax)
                    {
                        maxX = i;
                        maxY = j;
                    }
                }
            }
            score = layerBelow.score(xBelowRight, (float)j, 1, 1.0);
            if (candidate < score)
                return 0.0;
            if (maxValue < score)
            {
                maxValue = score;
                maxX = xBR-1;
                maxY = j;
            }
        }
        
        // bottom row:
        score = layerBelow.score(xBelowLeft, yBelowBottom, 1, 1.0);
        if (candidate < score)
            return 0.0;
        if (maxValue < score)
        {
            maxValue = score;
            maxX = xBL;
            maxY = yBB-1;
        }
        for (int i=xBL; i<xBR; i++)
        {
            score = layerBelow.score((float)i, yBelowBottom, 1, 1.0);
            if (candidate < score)
                return 0.0;
            if (maxValue < score)
            {
                maxValue = score;
                maxX = i;
                maxY = yBB-1;
            }
        }
        score = layerBelow.score(xBelowRight, yBelowBottom, 1, 1.0);
        if (candidate < score)
            return 0.0;
        if (maxValue < score)
        {
            maxValue = score;
            maxX = xBR-1;
            maxY = yBB-1;
        }
        // all scores in the projected patch are smaller than or equal score
        isMaximum = true;
        
        // refine maximum of layer below:
        float dXMaxBelow;
        float dYMaxBelow;
        float refinedMax = _maxScoreRefined2D(l-1, maxX, maxY, dXMaxBelow, dYMaxBelow);
        
        
        // project refined maximum coordinates to current layer:
        if(l%2==0)
        {
            // on octave
            deltaX=(((float)maxX+dXMaxBelow)*6.0+1.0)/8.0-(float)x;
            deltaY=(((float)maxY+dYMaxBelow)*6.0+1.0)/8.0-(float)y;
        }
        else{
            // on intra-octave
            deltaX=(((float)maxX+dXMaxBelow)*4.0-1.0)/6.0-(float)x;
            deltaY=(((float)maxY+dYMaxBelow)*4.0-1.0)/6.0-(float)y;
        }
        
        // saturate:
        bool returnRefined=true;
        if (deltaX > 1.0)
        {
            deltaX = 1.0;
            returnRefined = false;
        }
        if (deltaX < -1.0)
        {
            deltaX = -1.0;
            returnRefined = false;
        }
        if (deltaY > 1.0)
        {
            deltaY = 1.0;
            returnRefined = false;
        }
        if (deltaY < -1.0)
        {
            deltaY = -1.0;
            returnRefined = false;
        }
        
        if (returnRefined)
            return maxValue < refinedMax ? refinedMax : maxValue;
        
        return maxValue;
    }
    
    /**
     * Refines a score maximum on the surrounding 3x3 patch.
     * Returns the maximum value of the refined maximum position.
     * The refined maximum position relative to the unrefined maximum position is (deltaX,deltaY).
     * A 2D paraboloid f(x,y) = a*x^2+b*x*y+c*y^2+d*x+e*y+f is fitted in the least squares sense to the patch to obtain the unknowns a,b and c.
     **/
    float BRISKDetection::_maxScoreRefined2D(const int& l, const int& x, const int& y, float& deltaX, float& deltaY)
    {
        float maxVirtual = 0.0;
        
        // Get scores of surrounding 3x3 patch
        int s_1_1, s0_1, s1_1;
        int  s_10,  s00,  s10;
        int  s_11,  s01,  s11;
        if (l == -1)
        {
            // Virtual intra octave below layer 0, guess the virtual intra-octave with the AGAST5_8 detection on layer 0
            
            BRISKLayer& layer = _scaleSpace[0];
            
            s_1_1 = layer.score5_8(x-1, y-1, 1);
            maxVirtual = (float)s_1_1;
            s0_1 = layer.score5_8(x, y-1, 1);
            if (s0_1>(int)maxVirtual)
                maxVirtual = (float)s0_1;
            s1_1 = layer.score5_8(x+1, y-1, 1);
            if (s1_1>(int)maxVirtual)
                maxVirtual = (float)s1_1;
            s_10 = layer.score5_8(x-1, y, 1);
            if (s_10>(int)maxVirtual)
                maxVirtual = (float)s_10;
            s00 = layer.score5_8(x, y, 1);
            if (s00>(int)maxVirtual)
                maxVirtual = (float)s00;
            s10 = layer.score5_8(x+1, y, 1);
            if (s10>(int)maxVirtual)
                maxVirtual = (float)s10;
            s_11 = layer.score5_8(x-1, y+1, 1);
            if (s_11>(int)maxVirtual)
                maxVirtual = (float)s_11;
            s01 = layer.score5_8(x, y+1, 1);
            if (s01>(int)maxVirtual)
                maxVirtual = (float)s01;
            s11 = layer.score5_8(x+1, y+1, 1);
            if (s11>(int)maxVirtual)
                maxVirtual = (float)s11;
            
        } else
        {
            BRISKLayer& layer = _scaleSpace[l];
            s_1_1 = layer.score(x-1, y-1, 1);
            s0_1 = layer.score(x, y-1, 1);
            s1_1 = layer.score(x+1, y-1, 1);
            s_10 = layer.score(x-1, y, 1);
            s00 = layer.score(x, y, 1);
            s10 = layer.score(x+1, y, 1);
            s_11 = layer.score(x-1, y+1, 1);
            s01 = layer.score(x, y+1, 1);
            s11 = layer.score(x+1, y+1, 1);
        }
        
        // auxiliary variables
        const int s_1_1x6 = 6*s_1_1;
        const int s0_1x6 = 6*s0_1;
        const int s1_1x6 = 6*s1_1;
        const int s_10x6 = 6*s_10;
        const int s00x6 = 6*s00;
        const int s10x6 = 6*s10;
        const int s_11x6 = 6*s_11;
        const int s01x6 = 6*s01;
        const int s11x6 = 6*s11;
        const int tmp1 = s_1_1x6 + s1_1x6 + s_11x6 + s11x6 - (s00x6<<1);
        const int tmp2 = s0_1x6 + s01x6;
        const int tmp3 = s_10x6 + s10x6;
        const int tmp4 = -s_1_1x6 + s11x6;
        const int tmp5 = s1_1x6 - s_11x6;
        
        // Compute paraboloid coefficients a,b,c,d,e,f (scaled with factor 36 to stick to integer arithmetic)
        const int a = tmp1-(tmp2<<1)+tmp3;
        const int b = 9*(s_1_1+s11-(s1_1+s_11));
        const int c = tmp1+tmp2-(tmp3<<1);
        const int d = tmp4+tmp5+6*(s10-s_10);
        const int e = tmp4-tmp5+6*(s01-s0_1);
        const int f = (-(s_1_1+s1_1+s_11+s11)+((s0_1+s_10+s10+s01)<<1)+5*s00)<<2;
        
        // Determinant of the hessian matrix
        const int HDet = 4*a*c-b*b;

        if (HDet == 0)
        {
            // H semidefinite => maximum cannot be found by Hessian test, so just take the patch center as refined maximum
            deltaX = 0.0;
            deltaY = 0.0;
            if (maxVirtual)
                return maxVirtual;
            else
                return (float)f/36.0;
        }
        
        if (HDet<0 || a>=0)
        {
            // H indefinite (HDet<0) or positive definite (HDet>0 && a>=0) according to Sylvester's criterion.
            // The maximum must lie on the patch boundary. For simplicity just check the patch corners.
            
            // lower right corner:
            int tempMax = b+d+e;
            deltaX = 1.0;
            deltaY = 1.0;
            
            // lower left corner:
            int tmp = -b-d+e;
            if (tmp > tempMax)
            {
                tempMax = tmp;
                deltaX = -1.0;
                deltaY = 1.0;
            }
            
            // upper left corner:
            tmp = b-d-e;
            if (tmp > tempMax)
            {
                tempMax = tmp;
                deltaX = -1.0;
                deltaY = -1.0;
            }
            
            // upper right corner:
            tmp = -b+d-e;
            if (tmp > tempMax)
            {
                tempMax = tmp;
                deltaX = 1.0;
                deltaY = -1.0;
            }
            
            if (maxVirtual)
                return maxVirtual;
            else
                return (float)(tempMax+a+c+f)/36.0;
        }
        
        // H is negative definite => it exists a global maximum
        float dX = -(float)(2*c*d - b*e)/(float)HDet;
        float dY = -(float)(2*a*e - b*d)/(float)HDet;
        
        // check if (deltaX,deltaY) is beyond the patch boundary and saturate // TODO: check if this is correct or/and can be done faster
        bool tX = dX > 1.0;
        bool t_X = dX < -1.0;
        bool tY = dY > 1.0;
        bool t_Y = dY < -1.0;
        int numExceedings = tX+t_X+tY+t_Y;
        
        if (numExceedings == 0)
        {
            // (deltaX,deltaY) is within patch bounds
            deltaX = dX;
            deltaY = dY;
            if (maxVirtual)
                return maxVirtual;
            else
                return (float)(a*dX*dX+b*dX*dY+c*dY*dY+d*dX+e*dY+f)/36.0;
        }
        
        if (numExceedings == 1)
        {
            // Either dX or dY is out of bounds, saturate:
            if (tX)
            {
                deltaX = 1.0;
                deltaY = -(float)(b+e)/(float)(2*c);
                if (maxVirtual)
                    return maxVirtual;
                else
                    return (float)(a+b*deltaY+c*deltaY*deltaY+d+e*deltaY+f)/36.0;
            }
            if (t_X)
            {
                deltaX = -1.0;
                deltaY = (float)(b-e)/(float)(2*c);
                if (maxVirtual)
                    return maxVirtual;
                else
                    return (float)(a-b*deltaY+c*deltaY*deltaY-d+e*deltaY+f)/36.0;
            }
            if (tY)
            {
                deltaX = -(float)(b+d)/(float)(2*a);
                deltaY = 1.0;
                if (maxVirtual)
                    return maxVirtual;
                else
                    return (float)(a*deltaX*deltaX+b*deltaX+c+d*deltaX+e+f)/36.0;
            }
            deltaX = (float)(b-d)/(float)(2*a);
            deltaY = -1.0;
            if (maxVirtual)
                return maxVirtual;
            else
                return (float)(a*deltaX*deltaX-b*deltaX+c+d*deltaX-e+f)/36.0;
        }
        
        // Both dX and dY are out of bounds
        if (tX && tY)
        {
            deltaX = 1.0;
            deltaY = 1.0;
            if (maxVirtual)
                return maxVirtual;
            else
                return (float)(a+b+c+d+e+f)/36.0;
        }
        if (t_X && tY)
        {
            deltaX = -1.0;
            deltaY = 1.0;
            if (maxVirtual)
                return maxVirtual;
            else
                return (float)(a-b+c-d+e+f)/36.0;
        }
        if (tX && t_Y)
        {
            deltaX = 1.0;
            deltaY = -1.0;
            if (maxVirtual)
                return maxVirtual;
            else
                return (float)(a-b+c+d-e+f)/36.0;
        }
        deltaX = -1.0;
        deltaY = -1.0;
        if (maxVirtual)
            return maxVirtual;
        else
            return (float)(a+b+c-d-e+f)/36.0;
    }
    
    /**
     * Refines a score maximum of the original image (scale=1) along the scale axis.
     * Returns the refined maximum.
     * score0_7 is the score of the virtual intra-octave below the unsampled image (scale 0.7 ≈ 1/√2)
     * score1 is the score of the unsampled image (scale 1.0)
     * score1_5 is the score of the intra-octave above the unsampled image (scale 1.5)
     * A 1D parabola f(x) = a*x^2+b*x+c is fitted in the least squares sense to (0.7,score0_7), (1.0,score1), (1.5,score1_5)
     * to obtain the unknowns a,b and c.
     **/
    float BRISKDetection::_maxScoreRefinedScaleOnOriginalImage(float& scaleRefined, const float& score1, const float& score0_7, const float& score1_5)
    {
        // convert scores to integer arithmetic and round
        int s0_7 = (int)(1024.0*score0_7+0.5);
        int s1 = (int)(1024.0*score1+0.5);
        int s1_5 = (int)(1024.0*score1_5+0.5);
        
        // parabola coefficients:
        // a = 18*score0_7 - 30*score1 + 12*score1_5
        // b = -45*score0_7 + 65*score1 - 20*score1_5
        // c = 27*score0_7 - 30*score1 + 8*score1_5
    
        int a = 2*s0_7 - 4*s1 + 2*s1_5;
        
        // second derivative f''(x) = 2*a must be negative for a maximum:
        if (a >= 0)
        {
            // just return the unrefined maximum
            if (score0_7 > score1 && score0_7 > score1_5)
            {
                scaleRefined = 0.7;
                return score0_7;
            }
            if (score1_5 > score0_7 && score1_5 > score1)
            {
                scaleRefined = 1.5;
                return score1_5;
            }
            scaleRefined = 1.0;
            return score1;
        }
        
        int b = -5*s0_7 + 8*s1 - 3*s1_5;
        int c = 3*s0_7 - 3*s1 + 1*s1_5;
        
        // compute maximal point -b/(2a)
        scaleRefined = -((float)b)/((float)(2*a));
        
        // saturate and return:
        if (scaleRefined < 0.7)
        {
            scaleRefined = 0.7;
            return score0_7;
        }
        if (scaleRefined > 1.5)
        {
            scaleRefined = 1.5;
            return score1_5;
        }
        
        return ((float)a*scaleRefined*scaleRefined + (float)b*scaleRefined + (float)c) / 1024.0;
    }
    
    /**
     * Refines a score maximum of an octave (scale=2^i, i>0) along the scale axis.
     * Returns the refined maximum.
     * score0_75 is the score of the intra-octave below the octave (relative scale 0.75)
     * score1 is the score of the octave (relative scale 1.0)
     * score1_5 is the score of the intra-octave above the octave (relative scale 1.5)
     * A 1D parabola f(x) = a*x^2+b*x+c is fitted in the least squares sense to (0.75,score0_75), (1.0,score1), (1.5,score1_5)
     * to obtain the unknowns a,b and c.
     **/
    float BRISKDetection::_maxScoreRefinedScaleOnOctave(float& scaleRefined, const float& score1, const float& score0_75, const float& score1_5)
    {
        // convert scores to integer arithmetic and round
        int s0_75 = (int)(1024.0*score0_75+0.5);
        int s1 = (int)(1024.0*score1+0.5);
        int s1_5 = (int)(1024.0*score1_5+0.5);
        
        // parabola coefficients:
        // a = (16*score0_75 - 24*score1 + 8*score1_5) / 3
        // b = (-40*score0_75 + 54*score1 - 14*score1_5) / 3
        // c = (24*score0_75 - 27*score1 + 6*score1_5) / 3
        int tripleA = 16*s0_75 - 24*s1 + 8*s1_5;
        
        // second derivative f''(x) = 2*a must be negative for a maximum:
        if (tripleA >= 0)
        {
            // just return the unrefined maximum
            if (score0_75 > score1 && score0_75 > score1_5)
            {
                scaleRefined = 0.75;
                return score0_75;
            }
            if (score1_5 > score0_75 && score1_5 > score1)
            {
                scaleRefined = 1.5;
                return score1_5;
            }
            scaleRefined = 1.0;
            return score1;
        }
        
        int tripleB = -40*s0_75 + 54*s1 - 14*s1_5;
        int tripleC = 24*s0_75 - 27*s1 + 6*s1_5;
        
        // compute maximal point -b/(2a)
        scaleRefined = -((float)tripleB)/((float)(2*tripleA));
        
        // saturate and return:
        if (scaleRefined < 0.75)
        {
            scaleRefined = 0.75;
            return score0_75;
        }
        if (scaleRefined > 1.5)
        {
            scaleRefined = 1.5;
            return score1_5;
        }
        
        return ((float)tripleA*scaleRefined*scaleRefined + (float)tripleB*scaleRefined + (float)tripleC) / 3072.0;
    }
    
    /**
     * Refines a score maximum of an intra-octave along the scale axis.
     * Returns the refined maximum.
     * score0_66 is the score of the octave below the intra-octave (relative scale 2/3)
     * score1 is the score of the intra-octave (relative scale 1.0)
     * score1_33 is the score of the octave above the intra-octave (relative scale 4/3)
     * A 1D parabola f(x) = a*x^2+b*x+c is fitted in the least squares sense to (2/3,score0_66), (1.0,score1), (4/3,score1_33)
     * to obtain the unknowns a,b and c.
     **/
    float BRISKDetection::_maxScoreRefinedScaleOnIntraOctave(float& scaleRefined, const float& score1, const float& score0_66, const float& score1_33)
    {
        // convert scores to integer arithmetic and round
        int s0_66 = (int)(1024.0*score0_66+0.5);
        int s1 = (int)(1024.0*score1+0.5);
        int s1_33 = (int)(1024.0*score1_33+0.5);
        
        // parabola coefficients:
        // a = (9*score0_66 - 18*score1 + 9*score1_33) / 2
        // b = (-21*score0_66 + 36*score1 - 15*score1_33) / 2
        // c = (12*score0_66 - 16*score1 + 6*score1_33) / 2
        int doubleA = 9*s0_66 - 18*s1 + 9*s1_33;
        
        // second derivative f''(x) = 2*a must be negative for a maximum:
        if (doubleA >= 0)
        {
            // just return the unrefined maximum
            if (score0_66 > score1 && score0_66 > score1_33)
            {
                scaleRefined = 0.6666666666666666666666666667;
                return score0_66;
            }
            if (score1_33 > score0_66 && score1_33 > score1)
            {
                scaleRefined = 1.3333333333333333333333333333;
                return score1_33;
            }
            scaleRefined = 1.0;
            return score1;
        }
        
        int doubleB = -21*s0_66 + 36*s1 - 15*s1_33;
        int doubleC = 12*s0_66 - 16*s1 + 6*s1_33;
     
        // compute maximal point -b/(2a)
        scaleRefined = -((float)doubleB)/((float)(2*doubleA));
        
        // saturate and return:
        if (scaleRefined < 0.6666666666666666666666666667)
        {
            scaleRefined = 0.6666666666666666666666666667;
            return score0_66;
        }
        if (scaleRefined > 1.3333333333333333333333333333)
        {
            scaleRefined = 1.3333333333333333333333333333;
            return score1_33;
        }
        
        return ((float)doubleA*scaleRefined*scaleRefined + (float)doubleB*scaleRefined + (float)doubleC) / 2048.0;
    }
    
    /**
     * Refines the 2D position and the scale of a maximum
     * Assumes that the point (x,y) is a 2D maximum and that there is a layer above "layer"
     * Returnes the refined maximum value
     * isMaximum states if the candidate (x,y) is a 3D maximum
     * The refined maximum position and scale is stored in (refinedX,refinedY,refinedScale)
     */
    float BRISKDetection::_maxScoreRefined3D(const int& l, const int& x, const int& y, bool& isMaximum, float& xRefined, float& yRefined, float& scaleRefined)
    {
        BRISKLayer& thisLayer = _scaleSpace[l];
        const int candidateScore = thisLayer.score(x, y, 1);
        float maxValue; // return value
        
        // check layer above:
        float deltaXAbove;
        float deltaYAbove;
        float maxAbove = _maxScoreOnLayerAbove(l, x, y, candidateScore, isMaximum, deltaXAbove, deltaYAbove);
    
        if (!isMaximum)
            return 0.0;
        
        if (l%2==0)
        {
            // on octave
            // check layer below:
            float deltaXBelow = 0.0;
            float deltaYBelow = 0.0;
            float maxBelow;
            if (l==0)
                _maxScoreRefined2D(-1, x, y, deltaYBelow, deltaYBelow);
            else
            {
                maxBelow = _maxScoreOnLayerBelow(l, x, y, candidateScore, isMaximum, deltaXBelow, deltaYBelow);
                if (!isMaximum)
                    return 0.0;
            }
            
            // refine maximum on this layer:
            float deltaXThis;
            float deltaYThis;
            float maxThis = _maxScoreRefined2D(l, x, y, deltaXThis, deltaYThis);
            
            // refine scale:
            
            if (l==0)
                maxValue = _maxScoreRefinedScaleOnOriginalImage(scaleRefined, std::max(maxThis, (float)candidateScore), maxBelow, maxAbove);
            else
                maxValue = _maxScoreRefinedScaleOnOctave(scaleRefined, std::max(maxThis, (float)candidateScore), maxBelow, maxAbove);
            
            // interpolate max position along scale axis:
            if (scaleRefined > 1.0)
            {
                // interpolate between this layer and the layer above:
                const float weightThis = 2.0*(1.5-scaleRefined);
                const float weightAbove = 1.0-weightThis;
                xRefined = thisLayer.offset() + thisLayer.scale()*((float)x+weightThis*deltaXThis+weightAbove*deltaXAbove);
                yRefined = thisLayer.offset() + thisLayer.scale()*((float)y+weightThis*deltaYThis+weightAbove*deltaYAbove);
            } else
            {
                // interpolate between this layer and the layer below:
                if (l==0)
                {
                    const float weightThis = 3.3333333333333333333333333333*(scaleRefined-0.7);
                    const float weightBelow = 1.0-weightThis;
                    xRefined = (float)x+weightThis*deltaXThis+weightBelow*deltaXBelow;
                    yRefined = (float)y+weightThis*deltaYThis+weightBelow*deltaYBelow;
                } else
                {
                    const float weightThis = 4.0*(scaleRefined-0.75);
                    const float weightBelow = 1.0-weightThis;
                    xRefined = thisLayer.offset() + thisLayer.scale()*((float)x+weightThis*deltaXThis+weightBelow*deltaXBelow);
                    yRefined = thisLayer.offset() + thisLayer.scale()*((float)y+weightThis*deltaYThis+weightBelow*deltaYBelow);
                }
            }
        } else
        {
            // on intra-octave
            
            // check layer below:
            float deltaXBelow;
            float deltaYBelow;
            float maxBelow;
            maxBelow = _maxScoreOnLayerBelow(l, x, y, candidateScore, isMaximum, deltaXBelow, deltaYBelow);
            if (!isMaximum)
                return 0.0;
            
            // refine maximum on this layer:
            float deltaXThis;
            float deltaYThis;
            float maxThis = _maxScoreRefined2D(l, x, y, deltaXThis, deltaYThis);

            // refine scale
            maxValue = _maxScoreRefinedScaleOnIntraOctave(scaleRefined, std::max(maxThis, (float)candidateScore), maxBelow, maxAbove);
            
            // interpolate max position along scale axis:
            if (scaleRefined > 1.0)
            {
                // interpolate between this layer and the layer above:
                const float weightThis = 4.0-3.0*scaleRefined;
                const float weightAbove = 1.0-weightThis;
                xRefined = thisLayer.offset() + thisLayer.scale()*((float)x+weightThis*deltaXThis+weightAbove*deltaXAbove);
                yRefined = thisLayer.offset() + thisLayer.scale()*((float)y+weightThis*deltaYThis+weightAbove*deltaYAbove);
            } else
            {
                // interpolate between this layer and the layer below:
                const float weightThis = 3.0*scaleRefined-2.0;
                const float weightBelow = 1.0-weightThis;
                xRefined = thisLayer.offset() + thisLayer.scale()*((float)x+weightThis*deltaXThis+weightBelow*deltaXBelow);
                yRefined = thisLayer.offset() + thisLayer.scale()*((float)y+weightThis*deltaYThis+weightBelow*deltaYBelow);
            }
        }
        
        scaleRefined *= thisLayer.scale();
        return maxValue;
    }
}