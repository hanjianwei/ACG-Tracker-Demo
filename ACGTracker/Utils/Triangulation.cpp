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

#include "Triangulation.h"
#include <cassert>

namespace ACGT
{
    namespace Triangulation
    {
        void triangulateCorrespondences(std::vector<Eigen::Vector3d>& points, const std::vector<Eigen::Vector3d>& NDCs1, const std::vector<Eigen::Vector3d>& NDCs2, const Camera& camera1, const Camera& camera2)
        {
            assert(NDCs1.size() == NDCs2.size());
            std::vector<Eigen::Vector3d> NDCs(2);
            std::vector<SE3> poses(2);
            poses[0] = camera1.pose();
            poses[1] = camera2.pose();
            for (int i=0; i<NDCs1.size(); ++i)
            {
                NDCs[0] = NDCs1[i];
                NDCs[1] = NDCs2[i];
                points[i] = triangulate(NDCs, poses);
            }
        }
    }
}