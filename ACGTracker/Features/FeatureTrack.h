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

#ifndef ACG_Tracker_Demo_FeatureTrack_h
#define ACG_Tracker_Demo_FeatureTrack_h

#include <Eigen/Core>
#include <list>

namespace ACGT
{
    template < typename DescriptorType, unsigned int NumDescriptors, unsigned int RetainInit>
    class FeatureTrack
    {
       
    public:
        
        long& trackedObjectID()
        {
            return _trackedObjectID;
        }
        
        const long& trackedObjectID() const
        {
            return _trackedObjectID;
        }
        
        Eigen::Vector2f& predict()
        {
            return _predict;
        }
        
        const Eigen::Vector2f& predict() const
        {
            return _predict;
        }
        
        const std::list<DescriptorType>& descriptors() const
        {
            return _descriptors;
        }
        
        const unsigned int& retainCounter() const
        {
            return _retainCounter;
        }
        
        void trackedSuccessfully(const DescriptorType& descriptor)
        {
            _descriptors.push_front(descriptor);
            if (_descriptors.size() > NumDescriptors)
                _descriptors.pop_back();
            _retainCounter = RetainInit;
        }
        
        void lost()
        {
            _retainCounter--;
        }
        
    private:
        
        long _trackedObjectID;
        Eigen::Vector2f _predict;
        std::list<DescriptorType> _descriptors;
        unsigned int _retainCounter;
    };
}

#endif
