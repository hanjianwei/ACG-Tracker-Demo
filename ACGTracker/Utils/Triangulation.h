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

#ifndef __ACG_Tracker_Demo__Triangulation__
#define __ACG_Tracker_Demo__Triangulation__

#include <vector>
#include <Accelerate/Accelerate.h>
#include <cassert>

#include "Map/Camera.h"

namespace ACGT
{
    namespace Triangulation
    {
        void triangulateCorrespondences(std::vector<Eigen::Vector3d>& points, const std::vector<Eigen::Vector3d>& NDCs1, const std::vector<Eigen::Vector3d>& NDCs2, const Camera& camera1, const Camera& camera2);
        
        // triangulates
        static inline Eigen::Vector3d triangulate(const std::vector<Eigen::Vector3d>& NDCs, const std::vector<SE3>& poses)
        {
            assert(NDCs.size() == poses.size());
            assert(NDCs.size() >= 2);
            
            // variables for the lapack svd
            char jobu = 'N';
            char jobvt = 'A';
            __CLPK_integer m = 2*NDCs.size();
            __CLPK_integer n = 4;
            __CLPK_integer lDA = 2*NDCs.size();
            __CLPK_doublereal sigma[4];
            __CLPK_integer lDU = 2*NDCs.size();
            __CLPK_doublereal vt[16];
            __CLPK_integer lDVT = 4;
            __CLPK_integer lWork = 2*std::max((__CLPK_integer)1,std::max(3*std::min(m,n)+std::max(m,n),5*std::min(m,n)));
            std::vector<__CLPK_doublereal> work(lWork);
            __CLPK_integer info;
            
            std::vector<double> A(8*NDCs.size());
            for (int i=0; i<NDCs.size(); ++i)
            {
                const Eigen::Matrix3d R = poses[i].R();
                const Eigen::Vector3d t = poses[i].t();
                const double x = NDCs[i].x()/NDCs[i].z();
                const double y = NDCs[i].y()/NDCs[i].z();
                A[2*i] = x*R(2,0)-R(0,0);
                A[2*i+1] = y*R(2,0)-R(1,0);
                A[2*(i+NDCs.size())] = x*R(2,1)-R(0,1);
                A[2*(i+NDCs.size())+1] = y*R(2,1)-R(1,1);
                A[2*(i+2*NDCs.size())] = x*R(2,2)-R(0,2);
                A[2*(i+2*NDCs.size())+1] = y*R(2,2)-R(1,2);
                A[2*(i+3*NDCs.size())] = x*t.z()-t.x();
                A[2*(i+3*NDCs.size())+1] = y*t.z()-t.y();
            }
            
            // Find least squares solution of A*p = 0 as the last column of V from the svd of A = U*Sigma*V^T
            dgesvd_(&jobu,
                    &jobvt,
                    &m,
                    &n,
                    A.data(),
                    &lDA,
                    sigma,
                    NULL,
                    &lDU,
                    vt,
                    &lDVT,
                    work.data(),
                    &lWork,
                    &info);
            
            return Eigen::Vector3d(vt[3]/vt[15],vt[7]/vt[15],vt[11]/vt[15]);
        }
    }
}

#endif /* defined(__ACG_Tracker_Demo__Triangulation__) */
