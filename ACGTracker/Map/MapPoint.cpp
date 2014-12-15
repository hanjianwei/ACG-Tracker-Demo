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

#include <algorithm>
#include <cassert>
#include <vector>

#include "Map/MapPoint.h"
#include "Map/Camera.h"
#include "Types/GrayscaleImage.h"

namespace ACGT
{
    long MapPoint::_idCreater = 1;
    
    MapPoint::MapPoint() : _id(_idCreater++), Point3d()
    {
        
    }
    
    MapPoint::MapPoint(const MapPoint& other) : _id(_idCreater++), _keyframeReferences(other._keyframeReferences), Point3d(other._coordinates,other._color)
    {
        
    }
    
    MapPoint::MapPoint(MapPoint&& other)
    {
        std::swap(_id, other._id);
        std::swap(_coordinates, other._coordinates);
        std::swap(_color, other._color);
        std::swap(_keyframeReferences, other._keyframeReferences);
    }
    
    MapPoint& MapPoint::operator=(MapPoint&& other)
    {
        std::swap(_id, other._id);
        std::swap(_coordinates, other._coordinates);
        std::swap(_color, other._color);
        std::swap(_keyframeReferences, other._keyframeReferences);
        return *this;
    }
    
    MapPoint& MapPoint::operator=(const MapPoint& other)
    {
        _id = other._id;
        _coordinates = other._coordinates;
        _color = other._color;
        _keyframeReferences = other._keyframeReferences;
        return *this;
    }
    
    const long& MapPoint::id() const
    {
        return _id;
    }
    
    Eigen::Vector3d& MapPoint::coordinatesOpt()
    {
        return _coordinatesOpt;
    }
    
    const Eigen::Vector3d& MapPoint::coordinatesOpt() const
    {
        return _coordinatesOpt;
    }
    
    const long& KeyframeReference::keyframeID() const
    {
        return _keyframeID;
    }
    
    const long& KeyframeReference::featureIdx() const
    {
        return _featureIdx;
    }
    
    const unsigned int& MapPoint::framestamp() const
    {
        return _framestamp;
    }
    
    unsigned int& MapPoint::framestamp()
    {
        return _framestamp;
    }
    
    const unsigned int& MapPoint::matchCount() const
    {
        return _matchCount;
    }
    
    unsigned int& MapPoint::matchCount()
    {
        return _matchCount;
    }
    
    const unsigned int& MapPoint::inlierCount() const
    {
        return _inlierCount;
    }
    
    unsigned int& MapPoint::inlierCount()
    {
        return _inlierCount;
    }
    
    const std::vector<KeyframeReference>& MapPoint::keyframeReferences() const
    {
        return _keyframeReferences;
    }
    
    void MapPoint::addKeyframeReference(const Camera& keyframe, const long& featureIdx)
    {
        assert(std::find_if(std::begin(_keyframeReferences), std::end(_keyframeReferences), [&](const KeyframeReference& reference)
                            {
                                return reference.keyframeID() == keyframe.id();
                            }) == _keyframeReferences.end());
        _keyframeReferences.push_back(KeyframeReference(keyframe.id(), featureIdx));
    }
    
    void MapPoint::removeKeyframeReference(const long& keyframeID)
    {
        auto keyframeReference = std::find_if(std::begin(_keyframeReferences), std::end(_keyframeReferences), [&](const KeyframeReference& reference)
                                               {
                                                   return reference.keyframeID() == keyframeID;
                                               });
        if (keyframeReference == std::end(_keyframeReferences))
            return;

        _keyframeReferences.erase(keyframeReference);
    }
}
