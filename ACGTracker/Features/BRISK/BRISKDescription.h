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

#ifndef ACG_Tracker_Demo_BRISKDescription_h
#define ACG_Tracker_Demo_BRISKDescription_h

#include <vector>
#include <array>
#include <iostream>
#include <dispatch/dispatch.h>
#include <typeinfo>
#include <cassert>
#include "Features/FeatureDescription.h"
#include "Types/GrayscaleImage.h"
#include "Types/Image.h"

namespace ACGT
{
    /**
     * Template BRISK descriptor.
     * The template parameter DescriptorSize is the size of the descriptor in 16 bytes.
     **/
    template < unsigned int DescriptorSize >
    class BRISKDescriptor
    {
        
    public:
        
        const uint8_t& operator[](const int& idx) const
        {
            return _data[idx];
        }
        
        uint8_t& operator[](const int& idx)
        {
            return _data[idx];
        }
        
        inline float distance(const BRISKDescriptor<DescriptorSize>& other) const
        {
            int distance;
            asm volatile("mov         r4, %0           \n"
                         "mov         r5, %1           \n"
                         "mov         r6, %2           \n"
                         "vmov.i8     q0, #0           \n" // initialize global popcount accumulator to zero
                         "0:                           \n" // beginning of loop
                         "vld1.8      {q1}, [r4]!      \n" // load 128 bit from first descriptor
                         "vld1.8      {q2}, [r5]!      \n" // load 128 bit from second descriptor
                         "veor        q1, q1, q2       \n" // q1 = q1^q2
                         "vcnt.i8     q1, q1           \n" // bytewise popcount
                         "vadd.i8     q0, q0, q1       \n" // add popcount to global accumulator
                         "subs        r6, r6, #1       \n" // decrease descriptorSize
                         "bne         0b               \n" // end of loop, repeat loop if desriptorSize != 0
                         "vpaddl.u8   q0, q0           \n" // add consecutive bytes of global accumulator and store in 8 consecutive shorts
                         "vpaddl.u16  q0, q0           \n" // add consecutive shorts of global accumulator and store in 4 consecutive ints
                         "vpaddl.u32  q0, q0           \n" // add consecutive ints of global accumulator and store in 2 consecutive longs
                         "vadd.i64    d0, d0, d1       \n" // finally add the remaining longs together and store the result in d0
                         "vst1.32     {d0[0]}, [%3]    \n" // store the result as an int in distance
                         :
                         : "r"(_data.data()), "r"(other._data.data()), "r"(DescriptorSize), "r"(&distance)
                         : "r4", "r5", "r6"
                         );
            return (float)distance;
        }
        
    private:

        std::array<uint8_t,DescriptorSize*16> _data;
    };
    
    typedef struct BRISKSamplePoint
    {
        float x;         // x coordinate relative to center
        float y;         // x coordinate relative to center
        float sigma;     // Gaussian smoothing sigma
    } BRISKSamplePoint;
    
    typedef struct BRISKShortDistancePair
    {
        int i;  // index of the first pattern point
        int j;  // index of other pattern point
        double squaredDist; // euclidean distance of the pair
    } BRISKShortDistancePair;
    
    typedef struct BRISKLongDistancePair
    {
        int i;  // index of the first pattern point
        int j;  // index of other pattern point
        int weighted_dx; // 2048*(p_ix-p_jx)/(||p_i-p_j||^2)
        int weighted_dy; // 2048*(p_iy-p_jy)/(||p_i-p_j||^2)
    } BRISKLongDistancePair;
    
    template < unsigned int DescriptorSize >
    class BRISKDescription : public FeatureDescription<BRISKDescriptor<DescriptorSize>>
    {
        
    public:
        
        BRISKDescription()
        {
            // list of radiuses of the rings of the decriptor kernel for smallest scale 0.7 ≈ 1/√2
            const std::array<float,_numKernelRings> radiuses = {0.0,2.9,4.9,7.4,10.8};
            
            // list of numbers of sampling locations per ring of the descriptor kernel
            const std::array<int,_numKernelRings> numSamplesPerRing = {1,10,14,15,20};
            
            _scaleInvariant = true;
            _rotationInvariant = false;
            
            if (_kernelLUT.size() == 0)
            {
                // Initialize static members
                
                _numSamplePoints = 0;
                for (int i=0; i<_numKernelRings; i++)
                    _numSamplePoints += numSamplesPerRing[i];
                
                const float deltaMax = 6.88; // ≈ 9.75/√2
                const float deltaMin = 9.67; // ≈ 13.67/√2
                
                _generateKernelLUT(radiuses, numSamplesPerRing, deltaMin, deltaMax);
                
                const float lbMaxScale = log(_maxScale)/log(2.0);
                const float lbSqrt2 = 0.5;
                _basicScaleIndex = std::min(std::max(0, (int)(_numScales/lbMaxScale*lbSqrt2+0.5)),_numScales-1);
            }
        }
        
        bool& scaleInvariant()
        {
            return _scaleInvariant;
        }
        
        bool& rotationInvariant()
        {
            return _rotationInvariant;
        }
        
        const bool& scaleInvariant() const
        {
            return _scaleInvariant;
        }
        
        const bool& rotationInvariant() const
        {
            return _rotationInvariant;
        }
        
        virtual std::vector<BRISKDescriptor<DescriptorSize>> extract(const GrayscaleImage& image, std::vector<Feature>& features) const
        {
            return extract(image,features,image.integralImage());
        }
        
        virtual std::vector<BRISKDescriptor<DescriptorSize>> extract(const GrayscaleImage& image, std::vector<Feature>& features, const Image<int>& integralImage) const
        {
            // remember scale indices of features in kernelLUT
            std::vector<int> featureScales(features.size());
            auto scalesIterator = featureScales.begin();
            
            static const float ln2 = 0.693147180559945;
            static const float sqrt2 = 1.414213562373095;
            const float lbMaxScale = log(_maxScale)/(ln2);
            const float indexMultiplier = (float)_numScales/(lbMaxScale*ln2);
            
            // remove features that are too close to the border
            int numFilteredFeatures = 0;
            for (auto& feature : features)
            {
                int scaleIndex = 0;
                if (_scaleInvariant)
                    scaleIndex = std::min(std::max(0,(int)(indexMultiplier*log(feature.scale()*sqrt2*0.85)+0.5)),_numScales-1);
                else
                    scaleIndex = _basicScaleIndex;
                
                const int border = _kernelSizes[scaleIndex];
                const int maxX = image.width()-border;
                const int maxY = image.height()-border;
                if (!(feature.x()<border) && !(feature.x()>maxX) && !(feature.y()<border) && !(feature.y()>maxY))
                {
                    *scalesIterator = scaleIndex;
                    features[numFilteredFeatures] = feature;
                    numFilteredFeatures++;
                    scalesIterator++;
                }
            }
            features.resize(numFilteredFeatures);
            
            // allocate memory for descriptors and initialize to zero:
            __block std::vector<BRISKDescriptor<DescriptorSize>> descriptors(numFilteredFeatures, BRISKDescriptor<DescriptorSize>());
            
            // Compute descriptors in 16 subsets of the features simultaneously
            const int rangeSize = (int)ceil((float)features.size()/16.0);
            dispatch_apply(16, dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_HIGH, 0), ^(const size_t n)
                           {
                               // temporary array that holds the smoothed intensity values of the BRISK kernel:
                               std::vector<int> smoothedIntensities(_numSamplePoints);
                               
                               const int startIdx = n*rangeSize;
                               const int endIdx = std::min((n+1)*rangeSize, features.size());
                               
                               // compute descriptors:
                               for (int f=startIdx; f<endIdx; ++f)
                               {
                                   Feature& feature = features[f];
                                   
                                   // angle between the feature gradient and (1,0)^T
                                   int theta = 0;
                                   
                                   // compute the feature gradient:
                                   if (_rotationInvariant)
                                   {
                                       // compute the smoothed intensities in the unrotated pattern:
                                       for (int i=0; i<_numSamplePoints; i++)
                                           smoothedIntensities[i] = _smoothedIntensity(feature.x(), feature.y(), featureScales[f], 0, i, image, integralImage);
                                       
                                       // the feature gradient:
                                       int gradientX = 0;
                                       int gradientY = 0;
                                       
                                       // iterate through long distance pairs to compute gradient:
                                       for (const auto& lDP : _longDistancePairs)
                                       {
                                           const int deltaIntensity = smoothedIntensities[lDP.i]-smoothedIntensities[lDP.j];
                                           gradientX += deltaIntensity*lDP.weighted_dx/1024;
                                           gradientY += deltaIntensity*lDP.weighted_dy/1024;
                                       }
                                       const float angle = atan2((float)gradientY, (float)gradientX)/M_PI*180.0;
                                       feature.angle() = angle;
                                       theta = (int)((angle/360.0)*(float)_numRotations+0.5);
                                       if (theta<0)
                                           theta += _numRotations;
                                       else if (theta >= _numRotations)
                                           theta -= _numRotations;
                                   }
                                   
                                   // compute the smoothed intensities in the rotated pattern:
                                   for (int i=0; i<_numSamplePoints; i++)
                                       smoothedIntensities[i] = _smoothedIntensity(feature.x(), feature.y(), featureScales[f], theta, i, image, integralImage);
                                   
                                   int shifter = 0;
                                   
                                   // iterate through short distance pairs and compute descriptor:
                                   auto& descriptor = descriptors[f];
                                   int byteIdx = 0;
                                   for (auto& sDP : _shortDistancePairs)
                                   {
                                       if (shifter==8)
                                       {
                                           shifter = 0;
                                           byteIdx++;
                                       }
                                       if (smoothedIntensities[sDP.i]>smoothedIntensities[sDP.j])
                                           descriptor[byteIdx] |= (1<<shifter);
                                       // else already initialized to zero
                                       shifter++;
                                   }
                               }
                           });
            
            return descriptors;
        }
        
    private:
        
        template < std::size_t NumKernelRings >
        void _generateKernelLUT(const std::array<float,NumKernelRings>& radiuses, const std::array<int,NumKernelRings>& numSamplesPerRing, const float& deltaMin, const float& deltaMax)
        {
            _kernelLUT.resize(_numScales*_numRotations*_numSamplePoints);
            auto sampleIterator = _kernelLUT.begin();
            
            const float lbMaxScale = log(_maxScale)/log(2.0);
            const float lbScaleStep = lbMaxScale/(float)_numScales;
            const float sigma = 1.3;
            
            std::vector<float> scales;
            
            for (int scale=0; scale<_numScales; scale++)
            {
                scales.push_back(pow((double)2.0, (double)(scale*lbScaleStep)));
                _kernelSizes.push_back(0);
                
                double alpha, theta;
                for (int rot=0; rot<_numRotations; rot++)
                {
                    // rotation of feature:
                    theta = (double)rot/(double)_numRotations*2.0*M_PI;
                    for (int ring=0; ring<NumKernelRings; ring++)
                    {
                        for (int sample=0; sample<numSamplesPerRing[ring]; sample++)
                        {
                            // compute sample coordinates:
                            alpha = (double)sample/(double)numSamplesPerRing[ring]*2.0*M_PI;
                            sampleIterator->x = scales[scale]*radiuses[ring]*cos(theta+alpha);
                            sampleIterator->y = scales[scale]*radiuses[ring]*sin(theta+alpha);
                            
                            // compute gaussian kernel sigma:
                            if (ring == 0)
                                sampleIterator->sigma = sigma*scales[scale]*0.5;
                            else
                                sampleIterator->sigma = sigma*scales[scale]*(double)radiuses[ring]*sin(M_PI/numSamplesPerRing[ring]);
                            
                            // adapt size of kernel if necessary:
                            const int size = ceil(scales[scale]*radiuses[ring]+sampleIterator->sigma)+1;
                            if (size > _kernelSizes[scale])
                                _kernelSizes[scale] = size;
                            
                            // increment iterator:
                            sampleIterator++;
                        }
                    }
                }
            }
            
            // generate short and long distance pairs:
            const float deltaMinSquared = deltaMin*deltaMin;
            const float deltaMaxSquared = deltaMax*deltaMax;
            
            for (int i=1; i<_numSamplePoints; i++)
            {
                for (int j=0; j<i; j++)
                {
                    // (deltaX,deltaY)^T = p_j - p_i:
                    const float deltaX = _kernelLUT[j].x-_kernelLUT[i].x;
                    const float deltaY = _kernelLUT[j].y-_kernelLUT[i].y;
                    const float normSquared = deltaX*deltaX+deltaY*deltaY;
                    if (normSquared > deltaMinSquared)
                    {
                        // long distance pair:
                        BRISKLongDistancePair lDP;
                        lDP.i = i;
                        lDP.j = j;
                        lDP.weighted_dx = (int)(deltaX/normSquared*2048.0+0.5);
                        lDP.weighted_dy = (int)(deltaY/normSquared*2048.0+0.5);
                        _longDistancePairs.push_back(lDP);
                    } else if(normSquared<deltaMaxSquared)
                    {
                        // short distance pair:
                        BRISKShortDistancePair sDP;
                        sDP.i = i;
                        sDP.j = j;
                        sDP.squaredDist = normSquared;
                        _shortDistancePairs.push_back(sDP);
                    }
                }
            }
            
            const int descriptorBits = DescriptorSize*128;
            
            // sort short distance pairs according to distance
            for (int i=0; i<descriptorBits; i++)
            {
                int shortest = i;
                for (int j=i+1; j<_shortDistancePairs.size(); j++)
                {
                    if (_shortDistancePairs[j].squaredDist < _shortDistancePairs[shortest].squaredDist)
                        shortest = j;
                }
                // swap i-th element with shortest-th element
                BRISKShortDistancePair tmp = _shortDistancePairs[shortest];
                _shortDistancePairs[shortest] = _shortDistancePairs[i];
                _shortDistancePairs[i] = tmp;
            }
            _shortDistancePairs.resize(descriptorBits);

            // shuffle short distance pairs:
            srand(time(NULL));
            std::vector<int> shuffle(descriptorBits);
            for (int i=0; i<_shortDistancePairs.size(); i++)
                shuffle[i] = i;
    
            for (int k=0; k<10; k++)
            {
                for (int i=0; i<_shortDistancePairs.size()-1; i++)
                {
                    const int j = i+(int)((descriptorBits-i)*((double)rand()/((double)(RAND_MAX)+1.0)));
                    const int tmp = shuffle[j];
                    shuffle[j] = shuffle[i];
                    shuffle[i] = tmp;
                }
            }

            std::vector<BRISKShortDistancePair> tmpPairs = _shortDistancePairs;
            for (int i=0; i<_shortDistancePairs.size(); i++)
                _shortDistancePairs[i] = tmpPairs[shuffle[i]];

        }
        
        // approximate circular gaussian kernel with radius sigma by mean box kernel with sidelength 2*sigma
        int _smoothedIntensity(const float& x, const float& y, const int& scaleIdx, const int& rotationIdx, const int& sampleIdx, const GrayscaleImage& image, const Image<int>& integralImage) const
        {
            // compute float position and sigmaHalf (half of the box side length) of sample point:
            const BRISKSamplePoint& samplePoint = _kernelLUT[scaleIdx*_numRotations*_numSamplePoints+rotationIdx*_numSamplePoints+sampleIdx];
            const float xf = x+samplePoint.x;
            const float yf = y+samplePoint.y;
            const float sigmaHalf = samplePoint.sigma;
       
            // box area smaller than 1 pixel:
            if(sigmaHalf < 0.5)
            {
                // just do a bilinear interpolation inside the layer
                const int xi = (int)xf;
                const int yi = (int)yf;
                //multiply offsets with 1024 so that we can use integer arithmetic
                const int xOffset=(xf-(float)x)*1024.0;
                const int yOffset=(yf-(float)y)*1024.0;
                // get values of the four neighboring pixels
                const uint8_t *imageData = &image[yi][xi];
                const int valueUpperLeft = (int)*(imageData++);
                const int valueUpperRight = (int)*imageData;
                imageData +=image.width();
                const int valueLowerRight = (int)*(imageData--);
                const int valueLowerLeft = (int)*imageData;
                // interpolate value linear between upper pixels
                const int valueUpper = 1024*valueUpperLeft + xOffset * (valueUpperRight-valueUpperLeft);
                // interpolate value linear between lower pixels
                const int valueLower = 1024*valueLowerLeft + xOffset *  (valueLowerRight-valueLowerLeft);
                // interpolate linear between upper and lower values and round
                return (1024*valueUpper + yOffset * (valueLower-valueUpper) + 524288)/1048576;
                
            }
            
            // box area greater than one pixel:
            
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
            const float area=sigmaHalf*sigmaHalf*4.0;
            // scaling (1024*1024) to use integer arithmetic and normalization over box area:
            const int scaling = (int)(1048576.0/area);
            
            // scale boundary weights to int:
            const int weightXLeft = (int)((weightXLeftF)*(float)scaling);
            const int weightXRight = (int)((weightXRightF)*(float)scaling);
            const int weightYTop = (int)((weightYTopF)*(float)scaling);
            const int weightYBottom = (int)((weightYBottomF)*(float)scaling);
            
            //compute weights of the boundary corners:
            const int weightTopLeft = (weightXLeftF*weightYTopF)*scaling;
            const int weightTopRight = (weightXRightF*weightYTopF)*scaling;
            const int weightBottomLeft = (weightXLeftF*weightYBottomF)*scaling;
            const int weightBottomRight = (weightXRightF*weightYBottomF)*scaling;
            
            // compute average of the values in box area:
            const uint8_t *imageData = &image[(int)yTop][(int)xLeft];
            const int dX = (int)xRight-(int)xLeft-1;
            const int dY = (int)yBottom-(int)yTop-1;
            
            if(dX+dY>2)
            {
                // now the calculation:
                
                // first the corners:
                int retVal=weightTopLeft*(int)(*imageData);
                imageData += dX+1;
                retVal+=weightTopRight*(int)(*imageData);
                imageData += (dY+1)*image.width();
                retVal+=weightBottomRight*(int)(*imageData);
                imageData -= dX+1;
                retVal+= weightBottomLeft*(int)(*imageData);
                
                // next the edges:
                const int *integralData = &integralImage[(int)yTop-1][(int)xLeft];
                // find a simple path through the different surface corners
                const int tmp1 = (*integralData);
                integralData += dX;
                const int tmp2 = (*integralData);
                integralData += image.width();
                const int tmp3 = (*integralData);
                integralData++;
                const int tmp4 = (*integralData);
                integralData += dY*image.width();
                const int tmp5 = (*integralData);
                integralData--;
                const int tmp6 = (*integralData);
                integralData += image.width();
                const int tmp7 = (*integralData);
                integralData -= dX;
                const int tmp8 = (*integralData);
                integralData -= image.width();
                const int tmp9 = (*integralData);
                integralData--;
                const int tmp10 = (*integralData);
                integralData -= dY*image.width();
                const int tmp11 = (*integralData);
                integralData++;
                const int tmp12 = (*integralData);
                
                // assign the weighted surface integrals:
                const int upper=(tmp3-tmp2+tmp1-tmp12)*weightYTop;
                const int middle=(tmp6-tmp3+tmp12-tmp9)*scaling;
                const int left=(tmp9-tmp12+tmp11-tmp10)*weightXLeft;
                const int right=(tmp5-tmp4+tmp3-tmp6)*weightXRight;
                const int bottom=(tmp7-tmp6+tmp9-tmp8)*weightYBottom;
                
                return (retVal+upper+middle+left+right+bottom+512)/1024;
            }
            
            int tempSum = 0;
            
            // top row:
            int average = (int)*(imageData++) * weightTopLeft;
            const uint8_t *endTop = imageData + dX;
            while (imageData < endTop)
                tempSum += (int)*(imageData++);
        
            average += tempSum * weightYTop;
            average += (int)*imageData * weightTopRight;
            imageData += image.width() - dX - 1;
            
            // middle rows:
            tempSum = 0;
            const uint8_t *endMidY = imageData + dY * image.width();
            while (imageData < endMidY)
            {
                average += (int)*(imageData++) * weightXLeft;
                const uint8_t *endMidX = imageData + dX;
                while (imageData < endMidX)
                    tempSum += (int)*(imageData++);
                average += (int)*imageData * weightXRight;
                imageData += image.width() - dX - 1;
            }
            average += tempSum * scaling;
            
            // bottom row:
            tempSum = 0;
            average += (int)*(imageData++) * weightBottomLeft;
            const uint8_t *endBottom = imageData + dX;
            while (imageData < endBottom)
                tempSum += (int)*(imageData++);
        
            average += tempSum * weightYBottom;
            average += (int)*imageData * weightBottomRight;
            
            // rescale, round and return
            return (average+512)/1024;
        }
        
        bool _scaleInvariant;
        bool _rotationInvariant;
        
        static int _numSamplePoints; // total number of sample points in the descriptor kernel
        static std::vector<int> _kernelSizes; // half diameter of the descriptor kernel per scale
        
        static int _basicScaleIndex; // scale index of scale 1.0 in the kernelLUT
        
        static std::vector<BRISKSamplePoint> _kernelLUT;
        static std::vector<BRISKShortDistancePair> _shortDistancePairs;
        static std::vector<BRISKLongDistancePair> _longDistancePairs;
        
        static constexpr float _maxScale = 17.0; // maximum scale for maximal four allowed octave in the descriptor kernel LUT: 12.0*√2 = 16.97...; if more octaves should be allowed this must be changed;
        static constexpr int _numScales = 64; // number of discretized scales in the descriptor kernel LUT
        static constexpr int _numRotations = 1024; // number of discretized rotations in the descriptor kernel LUT
        static constexpr int _numKernelRings = 5; // number of rings in the descriptor kernel
    };
    
    template < unsigned int DescriptorSize >
    int BRISKDescription<DescriptorSize>::_numSamplePoints;
    
    template < unsigned int DescriptorSize >
    std::vector<int> BRISKDescription<DescriptorSize>::_kernelSizes;
    
    template < unsigned int DescriptorSize >
    int BRISKDescription<DescriptorSize>::_basicScaleIndex;
    
    template < unsigned int DescriptorSize >
    std::vector<BRISKSamplePoint> BRISKDescription<DescriptorSize>::_kernelLUT;
    
    template < unsigned int DescriptorSize >
    std::vector<BRISKShortDistancePair> BRISKDescription<DescriptorSize>::_shortDistancePairs;
    
    template < unsigned int DescriptorSize >
    std::vector<BRISKLongDistancePair> BRISKDescription<DescriptorSize>::_longDistancePairs;
}

// Overload left shift operator for printing
template < size_t DescriptorSize >
inline std::ostream& operator<<(std::ostream& os, const ACGT::BRISKDescriptor<DescriptorSize>& descriptor)
{
    for (const uint8_t& dByte : descriptor.data())
    {
        for (uint8_t i=0; i<8; ++i)
        {
            if (dByte & (1<<i))
                os << 1;
            else
                os << 0;
        }
        os << " ";
    }
    return os;
}

#endif
