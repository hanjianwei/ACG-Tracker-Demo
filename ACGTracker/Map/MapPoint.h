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

#ifndef __ACG_Tracker_Demo__MapPoint__
#define __ACG_Tracker_Demo__MapPoint__

#include <vector>
#include <array>
#include <Eigen/Core>

#include "Settings/Settings.h"
#include "Features/BRISK/BRISKDescription.h"
#include "Map/Camera.h"
#include "Types/Point3d.h"

namespace ACGT
{
    class KeyframeReference
    {
        
    public:
        
        KeyframeReference(const long& keyframeID, const long& featureIdx) : _keyframeID(keyframeID), _featureIdx(featureIdx) {};
        KeyframeReference(const KeyframeReference& other) : _keyframeID(other._keyframeID), _featureIdx(other._featureIdx) {};
        
        const long& keyframeID() const;
        const long& featureIdx() const;
        
    private:
        
        long _keyframeID;
        long _featureIdx;
    };
    
    class MapPoint : public Point3d
    {
        
    public:
        
        typedef long ID;
        
        MapPoint();
        MapPoint(const MapPoint& other);
        MapPoint(MapPoint&& other);
        
        MapPoint& operator=(MapPoint&& other);
        MapPoint& operator=(const MapPoint& other);
        
        const long& id() const;
        
        Eigen::Vector3d& coordinatesOpt();
        const Eigen::Vector3d& coordinatesOpt() const;
        
        const std::vector<KeyframeReference>& keyframeReferences() const;
        
        const unsigned int& framestamp() const;
        unsigned int& framestamp();
        
        const unsigned int& matchCount() const;
        unsigned int& matchCount();
        
        const unsigned int& inlierCount() const;
        unsigned int& inlierCount();
        
        void addKeyframeReference(const Camera& keyframe, const long& featureIdx);
        void removeKeyframeReference(const long& keyframeID);
        
    private:
        
        ID _id;
        
        Eigen::Vector3d _coordinatesOpt;
        
        std::vector<KeyframeReference> _keyframeReferences;
        
        unsigned int _framestamp;
        
        unsigned int _matchCount;
        unsigned int _inlierCount;
        
        static long _idCreater;
        
    };
}

#endif /* defined(__ACG_Tracker_Demo__MapPoint__) */
