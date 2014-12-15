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

#ifndef __ACG_Tracker_Demo__ACGTracker__
#define __ACG_Tracker_Demo__ACGTracker__

#include <memory>
#include <vector>

#include "Settings/Settings.h"
#include "Types/Intrinsics.h"
#include "Types/SE3.h"
#include "Features/FeatureTrack.h"
#include "Features/PatchDescription/PatchDescription.h"
#include "Map/Camera.h"
#include "Map/Map.h"

namespace ACGT
{
    class ACGTracker
    {
        
    public:
        
        void createMap(const BGRAImage& img1,
                       const Intrinsics& intrinsics1,
                       const Eigen::Vector3d& gravity1,
                       const BGRAImage& img2,
                       const Intrinsics& intrinsics2,
                       const Eigen::Vector3d& gravity2);
        
        bool localizeImage(const BGRAImage& img,
                           const Intrinsics& intrinsics,
                           SE3& pose);
        
        void reset();
        
        const Map& map() const;
        
    private:
        
        std::unique_ptr<Map> _map;
    };
}

#endif /* defined(__ACG_Tracker_Demo__ACGTracker__) */
