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

#ifndef ACG_Tracker_Demo_TwoViewMatching_h
#define ACG_Tracker_Demo_TwoViewMatching_h

#include "Match.h"

namespace ACGT
{
    namespace TwoViewMatching
    {
        template < typename DescriptorType >
        std::vector<Match> match(const std::vector<DescriptorType>& descriptors1, const std::vector<DescriptorType>& descriptors2, const float& threshold)
        {
            std::vector<Match> matches;
            for (int idx1=0; idx1<descriptors1.size(); ++idx1)
            {
                const DescriptorType& d1 = descriptors1[idx1];
                for (int idx2=0; idx2<descriptors2.size(); ++idx2)
                {
                    const float distance = d1.distance(descriptors2[idx2]);
                    if (distance < threshold)
                        matches.push_back(Match(idx1,idx2,distance));
                }
            }
            return matches;
        }
        
        template < typename DescriptorType >
        std::vector<Match> matchNN(const std::vector<DescriptorType>& descriptors1, const std::vector<DescriptorType>& descriptors2, const float& threshold)
        {
            std::vector<Match> matches;
            for (int idx1=0; idx1<descriptors1.size(); ++idx1)
            {
                const DescriptorType& d1 = descriptors1[idx1];
                Match nn;
                for (int idx2=0; idx2<descriptors2.size(); ++idx2)
                {
                    const float distance = d1.distance(descriptors2[idx2]);
                    if (distance < nn.distance())
                        nn = Match(idx1,idx2,distance);
                }
                if (nn.distance() < threshold)
                    matches.push_back(nn);
            }
            return matches;
        }
        
        template < typename DescriptorType >
        std::vector<Match> matchNNDR(const std::vector<DescriptorType>& descriptors1, const std::vector<DescriptorType>& descriptors2, const float& threshold, const float& ratio)
        {
            std::vector<Match> matches;
            for (int idx1=0; idx1<descriptors1.size(); ++idx1)
            {
                const DescriptorType& d1 = descriptors1[idx1];
                Match nn;
                Match nn2nd;
                for (int idx2=0; idx2<descriptors2.size(); ++idx2)
                {
                    const float distance = d1.distance(descriptors2[idx2]);
                    if (distance < nn.distance())
                    {
                        nn2nd = nn;
                        nn = Match(idx1,idx2,distance);
                    } else if (distance < nn2nd.distance())
                        nn2nd = Match(idx1,idx2,distance);
                    
                }
                if (nn.distance() < threshold && nn.distance() < ratio*nn2nd.distance())
                    matches.push_back(nn);
            }
            return matches;
        }
        
        template < typename DescriptorType >
        std::vector<Match> matchNNMutual(const std::vector<DescriptorType>& descriptors1, const std::vector<DescriptorType>& descriptors2, const float& threshold)
        {
            std::vector<Match> nn1(descriptors1.size(),Match());
            std::vector<Match> nn2(descriptors2.size(),Match());
            
            for (int idx1=0; idx1<descriptors1.size(); ++idx1)
            {
                const DescriptorType& d1 = descriptors1[idx1];
                for (int idx2=0; idx2<descriptors2.size(); ++idx2)
                {
                    const float distance = d1.distance(descriptors2[idx2]);
                    if (distance < nn1[idx1].distance())
                        nn1[idx1] = Match(idx1,idx2,distance);
                    if (distance < nn2[idx2].distance())
                        nn2[idx2] = Match(idx1,idx2,distance);
                }
            }
            std::vector<Match> matches;
            for (int idx1=0; idx1<descriptors1.size(); ++idx1)
            {
                if (nn1[idx1].distance() == std::numeric_limits<float>::infinity())
                    continue;
                if (nn2[nn1[idx1].idx2()].distance() == std::numeric_limits<float>::infinity())
                    continue;
                if (nn2[nn1[idx1].idx2()].idx1() == idx1)
                    matches.push_back(nn1[idx1]);
            }
            return matches;
        }
        
        template < typename DescriptorType >
        std::vector<Match> matchNNMutualDR(const std::vector<DescriptorType>& descriptors1, const std::vector<DescriptorType>& descriptors2, const float& threshold, const float& ratio)
        {
            std::vector<Match> nn1 = matchNNDR(descriptors1, descriptors2, threshold, ratio);
            std::vector<Match> nn2 = matchNNDR(descriptors2, descriptors1, threshold, ratio);
            
            std::vector<Match> matches;
            for (const Match& m1 : nn1)
            {
                if (std::find_if(std::begin(nn2), std::end(nn2), [&](const Match& m2)
                                 {
                                     return m1.idx1() == m2.idx2() && m1.idx2() == m2.idx1();
                                 }) != std::end(nn2))
                    matches.push_back(m1);
            }
            return matches;
        }
    };
}


#endif
