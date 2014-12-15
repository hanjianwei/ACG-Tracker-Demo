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

#ifndef ACG_Tracker_Demo_FeatureTracking_h
#define ACG_Tracker_Demo_FeatureTracking_h

#include <vector>
#include <limits>
#include <ctime>

#include "Feature.h"
#include "FeatureTrack.h"
#include "Match.h"

namespace ACGT
{
    namespace FeatureTracking
    {
        template < typename DescriptorType, unsigned int NumDescriptors, unsigned int RetainInit>
        std::vector<Match> track(const std::vector<Feature>& features, const std::vector<DescriptorType>& descriptors, const std::vector<FeatureTrack<DescriptorType,NumDescriptors,RetainInit>>& tracks, const float& maxPixelDistance, const unsigned int& threshold)
        {
            std::vector<Match> matches;
            matches.resize(tracks.size());
            
            int numMatches = 0;
            for (int idx2=0; idx2<tracks.size(); ++idx2)
            {
                Match nn;
                const Eigen::Vector2f& predict = tracks[idx2].predict();
                int idx1 = 0;
                while(idx1 < descriptors.size() && predict.x()-features[idx1].x()>maxPixelDistance)
                    ++idx1;
                for (; idx1<descriptors.size(); ++idx1)
                {
                    if (features[idx1].x()-predict.x()>maxPixelDistance)
                        break;
                    
                    if (fabs(predict.y()-features[idx1].y())>maxPixelDistance)
                        continue;

                    for (const DescriptorType& d1 : tracks[idx2].descriptors())
                    {
                        const float distance =  d1.distance(descriptors[idx1]);
                        if (distance < threshold && distance < nn.distance())
                        {
                            nn.idx1() = idx1;
                            nn.idx2() = idx2;
                            nn.distance() = distance;
                        }
                    }
                }
                if (nn.distance() < std::numeric_limits<float>::infinity())
                    matches[numMatches++] = nn;
            }
            matches.resize(numMatches);
            return matches;
        }
    }
}

#endif
