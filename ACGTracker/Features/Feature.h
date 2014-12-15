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

#ifndef __ACG_Tracker_Demo__Feature__
#define __ACG_Tracker_Demo__Feature__

#include <iostream>
#include <Eigen/Core>

namespace ACGT
{
    class Camera;
    
    class Feature
    {
    public:
    
        float& x();
        float& y();
        int& layer();
        float& response();
        int& score();
        float& scale();
        float& angle();
        
        const float& x() const;
        const float& y() const;
        const int& layer() const;
        const float& response() const;
        const int& score() const;
        const float& scale() const;
        const float& angle() const;
        
        Eigen::Vector2f coordinates() const;
        
        // Computes the feature in normalized device coordinates, with respect to the given camera
        Eigen::Vector3d NDCs(const Camera& camera) const;
        
    private:
      
        float _x;            // floating point x coordinate of keypoint
        float _y;            // floating point y coordinate of keypoint
        int _layer;          // the layer of the keypoint
        float _response;     // the response of the keypoint
        int _score;          // the unrefined score of the feature
        float _scale;        // the scale of the keypoint
        float _angle;        // the angle of the keypoint
    };
}

// Overload left shift operator for printing
inline std::ostream& operator<<(std::ostream& os, const ACGT::Feature& feature)
{
    os << feature.x() << " " << feature.y() << " ";
    return os;
}

// Function that


#endif /* defined(__ACG_Tracker_Demo__Feature__) */
