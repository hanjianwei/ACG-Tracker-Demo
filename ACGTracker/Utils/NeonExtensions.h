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

#ifndef ACG_Tracker_Demo_NeonExtensions_h
#define ACG_Tracker_Demo_NeonExtensions_h

#include <arm_neon.h>

namespace ACGT
{
    namespace NEONExtensions
    {
        // bytewise average of two doublewords
        inline uint8x8_t vavg_u8(uint8x8_t d1,uint8x8_t d2)
        {
            // Compute average (x+y)/2 as (x&y)+((x^y)>>1)
            // This does not produce overflows, thus we can stick to 8 bit unsigned integers
            return vsra_n_u8(vand_u8(d1, d2),veor_u8(d1, d2),1);
        }
        
        // bytewise average of two quadwords
        inline uint8x16_t vavgq_u8(uint8x16_t q1,uint8x16_t q2)
        {
            // Compute average (x+y)/2 as (x&y)+((x^y)>>1)
            // This does not produce overflows, thus we can stick to 8 bit unsigned integers
            return vsraq_n_u8(vandq_u8(q1, q2),veorq_u8(q1, q2),1);
        }
    }
}

#endif
