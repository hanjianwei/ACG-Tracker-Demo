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

#ifndef __ACG_Tracker_Demo__PatchDescription__
#define __ACG_Tracker_Demo__PatchDescription__

#include <cstdlib>
#include <array>

#include "Features/FeatureDescription.h"

namespace ACGT
{
    /**
     * Patch Descriptor
     **/
    class PatchDescriptor
    {
        
    public:
        
        const uint8_t& operator[](const int& idx) const;
        uint8_t& operator[](const int& idx);
        
        inline float distance(const PatchDescriptor& other) const
        {
            int distance;
            asm volatile("mov         r4, %0           \n"
                         "mov         r5, %1           \n"
                         "mov         r6, %2           \n"
                         "vmov.i16    q0, #0           \n" // initialize global popcount accumulator to zero
                         "0:                           \n" // beginning of loop
                         "vld1.8      {q1}, [r4]!      \n" // load 128 bit from first descriptor
                         "vld1.8      {q2}, [r5]!      \n" // load 128 bit from second descriptor
                         "vabal.u8    q0, d2, d4       \n" // q0 = q0+abs(d0-d1)
                         "vabal.u8    q0, d3, d5       \n" // q0 = q0+abs(d0-d1)
                         "subs        r6, r6, #1       \n" // decrease descriptorSize
                         "bne         0b               \n" // end of loop, repeat loop if desriptorSize != 0
                         "vpaddl.u16  q0, q0           \n" // add consecutive shorts of global accumulator and store in 4 consecutive ints
                         "vpaddl.u32  q0, q0           \n" // add consecutive ints of global accumulator and store in 2 consecutive longs
                         "vadd.i64    d0, d0, d1       \n" // finally add the remaining longs together and store the result in d0
                         "vst1.32     {d0[0]}, [%3]    \n" // store the result as an int in distance
                         :
                         : "r"(_data.data()), "r"(other._data.data()), "r"(1), "r"(&distance)
                         : "r4", "r5", "r6"
                         );
            return (float)distance;
        }
        
    private:
        
        std::array<uint8_t,16> _data;
    };
    
    class PatchDescription : public FeatureDescription<PatchDescriptor>
    {
        
    public:
        
        virtual std::vector<PatchDescriptor> extract(const GrayscaleImage& image, std::vector<Feature>& features) const;
        PatchDescriptor extract(const GrayscaleImage& image, const Feature& f) const;
        bool descriptable(const GrayscaleImage& image, const Feature& f) const;
    };
}

#endif /* defined(__ACG_Tracker_Demo__PatchDescription__) */
