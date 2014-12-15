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

#include <vector>
#include <array>
#include <Accelerate/Accelerate.h>
#include <Eigen/Core>
#include <cassert>
#include <limits>
#include <iostream>
#include <ctime>

#include "PNP.h"
#include "Types/SE3.h"
#include "Types/Intrinsics.h"
#include "Types/EulerAngles.h"
#include "Utils/NormalizationTransform.h"
#include "Utils/Metrics.h"
#include "Utils/PolynomialSolver.h"
#include "Settings/Settings.h"

namespace ACGT
{
    namespace PNP
    {
        std::vector<SE3> P3P(std::vector<Eigen::Vector3d>& u, std::vector<Eigen::Vector3d>& X)
        {
            assert(u.size() == 3 && X.size() == 3);
            
            std::vector<SE3> poses;
            
            // Ensure that the calibrated image coordinates are de-homogenized
            u[0].x() /= u[0].z();
            u[0].y() /= u[0].z();
            u[0].z() = 1.0;
            u[1].x() /= u[1].z();
            u[1].y() /= u[1].z();
            u[1].z() = 1.0;
            u[2].x() /= u[2].z();
            u[2].y() /= u[2].z();
            u[2].z() = 1.0;
            
            // Check if features and map points are distint
            if (u[0].x() == u[1].x() && u[0].y() == u[1].y())
                return poses;
                
            if (u[0].x() == u[2].x() && u[0].y() == u[2].y())
                return poses;
            
            if (u[1].x() == u[2].x() && u[1].y() == u[2].y())
                return poses;
            
            if (X[0].x() == X[1].x() && X[0].y() == X[1].y() && X[0].z() == X[1].z())
                return poses;
        
            if (X[0].x() == X[2].x() && X[0].y() == X[2].y() && X[0].z() == X[2].z())
                return poses;
        
            if (X[1].x() == X[2].x() && X[1].y() == X[2].y() && X[1].z() == X[2].z())
                return poses;
        
            
            // check if the points are collinear, if so, return since we won't get a meaningfull
            // pose estimate (degenerate case)
            {
                Eigen::Vector2d line_dir(u[0].x()-u[1].x(),u[0].y()-u[1].y());
                Eigen::Vector2d line_normal(-line_dir.y(), line_dir.x());
                line_normal.normalize();
                Eigen::Vector2d u2_2d(u[2].x(),u[2].y());
                if( fabs( line_normal.dot(u2_2d) ) < 1e-6 )
                    return poses;
            }
            
            ////
            // 1. compute the rays through the pixels
            Eigen::Vector3d ray_a = u[0];
            Eigen::Vector3d ray_b = u[1];
            Eigen::Vector3d ray_c = u[2];
            
            // get the squared norms of the vectors, then normalize them
            double l_ray_a( ray_a.norm() );
            double l_ray_b( ray_b.norm() );
            double l_ray_c( ray_c.norm() );
            
            
            ray_a.normalize();
            ray_b.normalize();
            ray_c.normalize();
            
            
            ////
            // 2.
            // a) compute the cosine of the angles between the rays
            double cos_ab = ray_a.dot(ray_b);
            double cos_ac = ray_a.dot(ray_c);
            double cos_bc = ray_b.dot(ray_c);
            
            // b) pairwise compute the squared length between the 3D points
            // in difference to the paper, Rab, ... are squared distances!
            double Rab = (X[1]- X[0]).squaredNorm();
            double Rac = (X[2]- X[0]).squaredNorm();
            double Rbc = (X[2]- X[1]).squaredNorm();

            // c) compute K1 and K2
            double K1 = Rbc / Rac;
            double K2 = Rbc / Rab;
         
            ////
            // 3.
            // a) compute the coefficients of the 4-th order polynomial that we want to
            // solve later on.
            
            // G4
            double tmp_1 = K1*K2 - K1 - K2;
            double tmp_2 = K1*K2 + K1 - K2;
            double tmp_3 = K1*K2 - K1 + K2;
            double G4 = tmp_1 * tmp_1 - 4.0*K1*K2*(cos_bc*cos_bc);
            // G3
            double G3 = 4.0*tmp_1*K2*(1.0-K1)* cos_ab + 4.0*K1*cos_bc*( tmp_3 * cos_ac + 2.0*K2*cos_ab*cos_bc );
            // G2
            double tmp_d = 2.0*K2*(1.0-K1)*cos_ab;
            double G2 = tmp_d * tmp_d + 2.0*tmp_2*tmp_1 + 4.0*K1*( (K1-K2)*(cos_bc*cos_bc) + (1.0 - K2)*K1*(cos_ac*cos_ac) - 2.0*K2*(1.0 + K1)* cos_ab* cos_ac* cos_bc );
            // G1
            double G1 = 4.0*tmp_2*K2*(1.0 - K1)* cos_ab + 4.0*K1*( tmp_3* cos_ac* cos_bc + 2.0*K1*K2*cos_ab*(cos_ac*cos_ac));
            // G0
            double G0 = tmp_2 * tmp_2 - 4.0*(K1*K1)*K2* (cos_ac*cos_ac);

            // find the roots of the quartic
            double roots_quartic[4];
            int nb_pos_real_roots = PolynomialSolver::solveQuartic(roots_quartic, G4, G3, G2, G1, G0);
            
            ////
            // 4. for every positive, real solution of the polynomial we compute the
            // length a,b and c and solve for the rotation matrix and the translation
            for( size_t i=0; i<nb_pos_real_roots; ++i )
            {
                ////
                // compute a and b (note that Rab here corresponds to Rab^2 in the
                // paper!)
                double a = sqrt( Rab ) / sqrt( roots_quartic[i]*roots_quartic[i] - 2.0*roots_quartic[i]* cos_ab + 1.0 );
                
                double b = a*roots_quartic[i];
                
                
                ////
                // compute c, to do so, we first have to compute m, m_prime, p, p_prime,
                // q and q_prime
                double m = 1.0 - K1;
                double p = 2.0 * ( K1 * cos_ac - roots_quartic[i] * cos_bc );
                double q = roots_quartic[i]*roots_quartic[i] - K1;
                
                double m_prime = 1.0;
                double p_prime = - ( 2.0 * roots_quartic[i] * cos_bc );
                double q_prime = roots_quartic[i]*roots_quartic[i] * ( 1.0 - K2 ) + 2.0 * roots_quartic[i] * K2 * cos_ab - K2;
                
                double c[2];
                int c_count = 0;
                
                if( fabs((m_prime * q) - (m * q_prime)) > 1e-5 )
                {
                    // get y from equation (A26)
                    double y = ( p_prime*q - p*q_prime ) / ( m*q_prime - m_prime*q );
                    c[0] = y*a;
                    c_count = 1;
                } else
                {
                    // get y from equation (A27)
                    double root_ = sqrt( (cos_ac*cos_ac) + ( Rac - a*a ) / (a*a) );
                    double y[2];
                    y[0] = cos_ac + root_;
                    y[1] = cos_ac - root_;
                    
                    // check if the solutions satisfy equation (A3), if not, we do not
                    // use them
                    double c_tmp = y[0]*a;
                    if( fabs(Rbc - b*b - c_tmp*c_tmp + 2.0*b*c_tmp*cos_bc) < 1e-5 )
                    {
                        c[0] = c_tmp;
                        c_count = 1;
                    }
                    
                    c_tmp = y[1]*a;
                    if( fabs(Rbc - b*b - c_tmp*c_tmp + 2.0*b*c_tmp*cos_bc) < 1e-5 )
                    {
                        c[c_count] = c_tmp;
                        ++c_count;
                    }
                }
                
                ////
                // for all possible combinations of a,b and c we compute R and t
                for( int j=0; j<c_count; ++j )
                {
                    ////
                    // construct the plane P1
                    double cos_LAB = (a*a - b*b + Rab ) / (2.0*a*sqrt(Rab));
                    double QA = a * cos_LAB;
                    
                    Eigen::Vector3d AB = X[1]-X[0];
                    AB.normalize();
                    
                    Eigen::Vector3d Q = QA*AB+X[0];
                    
                    // implicit representation of P1 (with normal AB)
                    double P1_d = -AB.dot(Q);
                    
                    ////
                    // construct the plane P2
                    double cos_LAC = (a*a - c[j]*c[j] + Rac ) / (2.0*a*sqrt(Rac));
                    double QA2 = a * cos_LAC;
                    
                    Eigen::Vector3d AC = X[2]-X[0];
                    AC.normalize();
                    
                    Eigen::Vector3d Q2 = QA2*AC+X[0];
                    
                    // implicit representation of P2 (with normal AC)
                    double P2_d = -AC.dot(Q2);
                    
                    
                    ////
                    // construction of the plane P3
                    
                    // choose the normal such that it points into the direction of the camera
                    Eigen::Vector3d normal = (c[j]*ray_c-a*ray_a).cross((b*ray_b-a*ray_a));
                    //(ray_c*c[j] - ray_a*a) % (ray_b*b - ray_a*a);
                    
                    if( -normal.z() < 0.0 )
                        normal = (X[2]-X[0]).cross(X[1]-X[0]);
                    else
                        normal = (X[1]-X[0]).cross(X[2]-X[0]);
                    
                    normal.normalize();
                    
                    // implicit representation of P3 (with normal normal)
                    double P3_d = -normal.dot(X[0]);
                    //(-normal) | mv_corresp[0].m_point3D;
                    
                    
                    ////
                    // compute the intersection R of the planes
                    // see http://www.cgafaq.info/wiki/Intersection_of_three_planes
                    Eigen::Vector3d R = -P1_d*AC.cross(normal);
                    R = R-P2_d*normal.cross(AB);
                    R = R-P3_d*AB.cross(AC);
                    
                    tmp_1 = 1.0 / AB.dot(AC.cross(normal));
                    R = tmp_1*R;
                    
                    ////
                    // compute the center of projection L in global coordinates
                    double AR = (R-X[0]).norm();
                    //( R - mv_corresp[0].m_point3D ).norm(); // length of line between A and R
                    if( a*a < AR*AR )
                        continue;
                    
                    double LR = sqrt( a*a - AR*AR );
                    Eigen::Vector3d L = LR*normal+R;
                    //(normal * LR) + R;
                    
                    ////
                    // compute the rotation matrix R
                    // first compute the 3D position of the image points
                    Eigen::Vector3d ray_A = X[0]-L;
                    ray_A.normalize();
                    ray_A = l_ray_a*ray_A;
                    
                    Eigen::Vector3d ray_B = X[1]-L;
                    ray_B.normalize();
                    ray_B = l_ray_b*ray_B;
                    
                    Eigen::Vector3d ray_C = X[2]-L;
                    ray_C.normalize();
                    ray_C = l_ray_c*ray_C;
                    
                    // compute the normal of the image plane, pointing away from L
                    Eigen::Vector3d n_3D = (ray_C-ray_A).cross(ray_B-ray_A);
                    n_3D.normalize();
                    
                    // check for orientation of normal and correct it if necessary
                    if( ray_A.dot(n_3D ) < 0.0 )
                        n_3D *= -1.0;
                    
                    
                    // get the 3D position of the principal point
                    Eigen::Vector3d PP_3D = n_3D+L;
                    
                    // get the principal point in local camera coordinates
                    Eigen::Vector3d n_i(0.0, 0.0, -1.0);
                    
                    // for both coordinate systems, compute a vetcor orthogonal to the
                    // direction the camera is looking to
                    Eigen::Vector3d r_3D = ray_A+L-PP_3D;
                    r_3D.normalize();
                    
                    Eigen::Vector3d r_i(-u[0].x(), -u[0].y(), 0.0);
                    r_i.normalize();
                    
                    // for both coordinate systems, compute an orthonormal system
                    Eigen::Vector3d u_3D = r_3D.cross(n_3D);
                    u_3D.normalize();
                    
                    Eigen::Vector3d u_i = r_i.cross(n_i);
                    u_i.normalize();
                    
                    // now, assemble the rotation matrix and store it
                    Eigen::Matrix3d rot;
                    rot(0,0) = r_i.x() * r_3D.x() + u_i.x() * u_3D.x();
                    rot(1,0) = r_i.y() * r_3D.x() + u_i.y() * u_3D.x();
                    rot(2,0) = r_i.z() * r_3D.x() + u_i.z() * u_3D.x() - n_3D.x();
                    rot(0,1) = r_i.x() * r_3D.y() + u_i.x() * u_3D.y();
                    rot(1,1) = r_i.y() * r_3D.y() + u_i.y() * u_3D.y();
                    rot(2,1) = r_i.z() * r_3D.y() + u_i.z() * u_3D.y() - n_3D.y();
                    rot(0,2) = r_i.x() * r_3D.z() + u_i.x() * u_3D.z();
                    rot(1,2) = r_i.y() * r_3D.z() + u_i.y() * u_3D.z();
                    rot(2,2) = r_i.z() * r_3D.z() + u_i.z() * u_3D.z() - n_3D.z();
                    
                    Eigen::Vector3d t = -rot*L;
                    
                    // compute the reprojection error of the point pairs
                    double reproj_err = 0.0;
                    Eigen::Vector3d reprojection;
                    for( int k=0; k<3; ++k)
                    {
                        reprojection = rot*X[k]+t;
                        reprojection /= reprojection.z();
                        reproj_err += (u[k]-reprojection).squaredNorm();
                    }
                  
                    if( reproj_err < 1e-12)
                    {
                        SE3 pose;
                        pose.R() = rot;
                        pose.t() = t;
                        poses.push_back(pose);
                    }
                }
            }
           
            return poses;
        }

        void countInliers(const SE3& pose, const std::vector<Eigen::Vector3d>& NDCs, const std::vector<Eigen::Vector3d>& points, const Intrinsics& intrinsics, const double& threshold, double& costMSAC, std::vector<int>& inliers)
        {
            assert(NDCs.size() == points.size());
            
            // Reproject 3D points into camera
            std::vector<Eigen::Vector3d> reprojections(points.size());
            
            vDSP_mmulD((double*)points.data(), 1, (double*)pose.R().data(), 1, (double*)reprojections.data(), 1, (int)points.size(), 3, 3);
            vDSP_vsaddD((double*)reprojections.data(), 3, (double*)pose.t().data(), (double*)reprojections.data(), 3, points.size());
            vDSP_vsaddD((double*)reprojections.data()+1, 3, (double*)pose.t().data()+1, (double*)reprojections.data()+1, 3, points.size());
            vDSP_vsaddD((double*)reprojections.data()+2, 3, (double*)pose.t().data()+2, (double*)reprojections.data()+2, 3, points.size());
            
            const float squaredThreshold = threshold*threshold;
            
            costMSAC = 0.0;
            inliers.clear();
            for (int i=0; i<points.size(); i++)
            {
                // Compute reprojection error
                const float error = Metrics::squaredReprojectionError(NDCs[i],reprojections[i],intrinsics);
                
                if (error < squaredThreshold)
                {
                    inliers.push_back(i);
                    costMSAC += error;
                } else
                    costMSAC += squaredThreshold;
            }
        }
        
        std::vector<int> ransacP3P(SE3& pose, const std::vector<Eigen::Vector3d>& NDCs, const std::vector<Eigen::Vector3d>& points, const float& threshold, const Intrinsics& intrinsics)
        {
            assert(NDCs.size() == points.size());
            
            std::vector<int> bestInliers;
            
            // check if there are at least two correspondences
            if (NDCs.size() < 3)
                return bestInliers;
            
            std::vector<int> inliers;
            inliers.reserve(NDCs.size());
            bestInliers.reserve(NDCs.size());
            
            int sampleIndices[3];
            std::vector<Eigen::Vector3d> samples1(3);
            std::vector<Eigen::Vector3d> samples2(3);
            
            // initialze fraction of outlier matches to 0.5 (half of the matches are outliers)
            double eps = 0.9;
            // set probability of finding an outlier free sample to 0.99:
            const double log1_P = log(0.001);
            // initialize number of RANSAC iterations
            unsigned int numIterations = (int)ceil(log1_P/log(1.0-(1.0-eps)*(1.0-eps)*(1.0-eps)));
            int i=0; // processed RANSAC iterations
            double minCostMSAC = std::numeric_limits<double>::max(); // smallest average reprojection error of the consensus set
            
            srand(time(NULL));
            
            while (i<std::max(Settings::RansacP3PMinIterations,std::min(numIterations,Settings::RansacP3PMaxIterations)))
            {
                // choose random sample
                for (int j = 0; j<3; j++) {
                    bool uniqueSample = false;
                    while (!uniqueSample)
                    {
                        uniqueSample = true;
                        sampleIndices[j] = (int)(NDCs.size()*((double)rand()/((double)(RAND_MAX)+1.0)));
                        for (int k=0; k<j; k++)
                        {
                            if (sampleIndices[k]==sampleIndices[j])
                            {
                                uniqueSample = false;
                                break;
                            }
                        }
                    }
                    
                    // fill sampleMatches
                    samples1[j] = NDCs[sampleIndices[j]];
                    samples2[j] = points[sampleIndices[j]];
                }
                
                // compute pose candidates from samples
                std::vector<SE3> candidates = P3P(samples1,samples2);
                
                // check all candidates
                for (const SE3& candidate : candidates)
                {
                    // count inliers
                    double costMSAC;
                    countInliers(candidate, NDCs, points, intrinsics, threshold, costMSAC, inliers);
                  
                    // update consensus set and numIterations
                    if (costMSAC < minCostMSAC)
                    {
                        pose = candidate;
                        inliers.swap(bestInliers);
                        minCostMSAC = costMSAC;
                        const double outlierRatio = 1.0 - (double)bestInliers.size()/(double)NDCs.size();
                        if (outlierRatio == 0.0)
                            numIterations = 0;
                        else if (outlierRatio < eps)
                        {
                            eps = outlierRatio;
                            numIterations = (int)ceil(log1_P/log(1.0-(1.0-eps)*(1.0-eps)*(1.0-eps)));
                        }
                    }
                }
                
                i++;
            }
            return bestInliers;
        }
        
        void reproject(const SE3& pose, const std::vector<Eigen::Vector3d>& points, const Intrinsics& intrinsics, Eigen::Vector2d* reprojections)
        {
            
            // transform map points to coordinate frame of current camera
            std::vector<Eigen::Vector3d> pointsTransformed(points.size());
            vDSP_mmulD((double*)points.data(), 1, pose.R().data(), 1, (double*)pointsTransformed.data(), 1, points.size(), 3, 3);
            vDSP_vsaddD((double*)pointsTransformed.data(), 3, pose.t().data(), (double*)pointsTransformed.data(), 3, points.size());
            vDSP_vsaddD((double*)pointsTransformed.data()+1, 3, pose.t().data()+1, (double*)pointsTransformed.data()+1, 3, points.size());
            vDSP_vsaddD((double*)pointsTransformed.data()+2, 3, pose.t().data()+2, (double*)pointsTransformed.data()+2, 3, points.size());
        
            // de-calibrate
            vDSP_vdivD((double*)pointsTransformed.data()+2, 3, (double*)pointsTransformed.data(), 3, (double*)reprojections, 2, points.size());
            vDSP_vdivD((double*)pointsTransformed.data()+2, 3, (double*)pointsTransformed.data()+1, 3, (double*)reprojections+1, 2, points.size());
            vDSP_vsmulD((double*)reprojections, 2, &intrinsics.f(), (double*)reprojections, 2, points.size());
            vDSP_vsaddD((double*)reprojections, 2, &intrinsics.pX(), (double*)reprojections, 2, points.size());
            vDSP_vsmulD((double*)reprojections+1, 2, &intrinsics.f(), (double*)reprojections+1, 2, points.size());
            vDSP_vsaddD((double*)reprojections+1, 2, &intrinsics.pY(), (double*)reprojections+1, 2, points.size());
        }
        
        void computeResiduals(const std::vector<Eigen::Vector2d>& features, const std::vector<Eigen::Vector2d>& reprojections, std::vector<Eigen::Vector2d>& residuals)
        {
            vDSP_vsubD((double*)reprojections.data(), 2, (double*)features.data(), 2, (double*)residuals.data(), 2, features.size());
            vDSP_vsubD((double*)reprojections.data()+1, 2, (double*)features.data()+1, 2, (double*)residuals.data()+1, 2, features.size());
        }
        
        // parameters = [yaw,pitch,roll,tX,tY,tZ]
        void jacobian(const std::vector<Eigen::Vector3d>& points, const Intrinsics& intrinsics, std::array<double,6>& parameters, std::vector<double>& J, std::vector<double>& JT)
        {
            J.resize(points.size()*6*2);
            JT.resize(points.size()*6*2);

            for (int p = 0; p<6; p++)
            {
                // compute dp
                double dp;
                if (p<3) {
                    dp = 0.000001*(fabs(parameters[p]));
                } else {
                    dp = 0.000001*(fabs(parameters[p]));
                }
                
                if (dp != 0.0)
                {
                    // approximate gradient by central difference
                    
                    // parameter perturbation (P+dP)
                    parameters[p] += dp;
                    
                    SE3 pose;
                    pose.R() = EulerAngles(parameters[2], parameters[1], parameters[0]).matrix();
                    pose.t().x() = parameters[3];
                    pose.t().y() = parameters[4];
                    pose.t().z() = parameters[5];
                    reproject(pose, points, intrinsics, (Eigen::Vector2d*)JT.data()+points.size()*p);
                    
                    // parameter perturbation (P-dP)
                    parameters[p] -= 2.0*dp;
                    
                    pose.R() = EulerAngles(parameters[2], parameters[1], parameters[0]).matrix();
                    pose.t().x() = parameters[3];
                    pose.t().y() = parameters[4];
                    pose.t().z() = parameters[5];
                    reproject(pose, points, intrinsics, (Eigen::Vector2d*)J.data()+points.size()*p);
                    
                    vDSP_vsubD((double*)J.data()+points.size()*p*2, 1, (double*)JT.data()+points.size()*p*2, 1, (double*)JT.data()+points.size()*p*2, 1, points.size()*2);
                    double twoDP = 2.0*dp;
                    vDSP_vsdivD((double*)JT.data()+points.size()*p*2, 1, &twoDP, (double*)JT.data()+points.size()*p*2, 1, points.size()*2);
                    
                    // restore parameters
                    parameters[p] += dp;
                    
                }
            }
            
            vDSP_mtransD(JT.data(), 1, J.data(), 1, 2*points.size(), 6);
        }
        
        constexpr double ResidualEps = 0.00000001;
        constexpr double ChiSquareEps = 0.00000001;
        constexpr double ParameterEps = 0.00000001;
        constexpr double StepEps = 1e-3;
        constexpr double CostEps = 1e-2;
        constexpr double LambdaStart = 1e-1;
        constexpr double LambdaMin = 0.0001;
        constexpr double LambdaMax = 1e4;
        
        void PNPRefineNonLinear(SE3& pose, const std::vector<Eigen::Vector3d>& NDCs, const std::vector<Eigen::Vector3d>& points, const Intrinsics& intrinsics)
        {
            assert(NDCs.size() == points.size());
          
            bool stop = false;
            double lambdaIncreaseFactor = 2.0;
            
            if (NDCs.size() < 3)
            {
                // not enough correspondences, pose not uniquely defined => exit
                return;
            }
            
            std::vector<Eigen::Vector2d> reprojections(NDCs.size());
            reproject(pose, points, intrinsics, reprojections.data());
          
            // de-calibrate NDCs
            std::vector<Eigen::Vector2d> features(NDCs.size());
            vDSP_vdivD((double*)NDCs.data()+2, 3, (double*)NDCs.data(), 3, (double*)features.data(), 2, NDCs.size());
            vDSP_vdivD((double*)NDCs.data()+2, 3, (double*)NDCs.data()+1, 3, (double*)features.data()+1, 2, NDCs.size());
            vDSP_vsmulD((double*)features.data(), 2, &intrinsics.f(), (double*)features.data(), 2, NDCs.size());
            vDSP_vsaddD((double*)features.data(), 2, &intrinsics.pX(), (double*)features.data(), 2, NDCs.size());
            vDSP_vsmulD((double*)features.data()+1, 2, &intrinsics.f(), (double*)features.data()+1, 2, NDCs.size());
            vDSP_vsaddD((double*)features.data()+1, 2, &intrinsics.pY(), (double*)features.data()+1, 2, NDCs.size());
            
            // calculate residuals
            std::vector<Eigen::Vector2d> residuals(NDCs.size());
            computeResiduals(features, reprojections, residuals);
            
            std::vector<double> J;  // Jacobian
            std::vector<double> JT; // Transpose of Jacobian
            
            std::array<double, 6> parameters;
            const EulerAngles angles(static_cast<Eigen::Matrix3d>(pose.R()));
            parameters[0] = angles.yaw();
            parameters[1] = angles.pitch();
            parameters[2] = angles.roll();
            parameters[3] = pose.t().x();
            parameters[4] = pose.t().y();
            parameters[5] = pose.t().z();
            jacobian(points, intrinsics, parameters, J, JT);
            
            double X2; // Chi-Square error criteria
            vDSP_svesqD((double*)residuals.data(), 1, &X2, NDCs.size()*2);
            X2 *= 0.5;
            
            double maxResiduals = 0.0;
            for (int i=0; i<NDCs.size(); i++)
            {
                maxResiduals = std::max(maxResiduals, fabs(residuals[i].x()));
                maxResiduals = std::max(maxResiduals, fabs(residuals[i].y()));
            }
            
            if (maxResiduals < ResidualEps)
                stop = true;
            
            double lambda = LambdaStart;
            int iterations = 0;
            
            std::array<double, 36> JT_J; //JT*J
            vDSP_mmulD(JT.data(), 1, J.data(), 1, JT_J.data(), 1, 6, 6, NDCs.size()*2);
            
            std::array<double, 6> JT_R; // JT*residuals
            vDSP_mmulD(JT.data(), 1, (double*)residuals.data(), 1, JT_R.data(), 1, 6, 1, NDCs.size()*2);
            
            std::array<double, 36> LHS;
            std::array<double, 6> RHS;
            
            while (!stop)
            {
                iterations++;
                
                // find deltaP
                LHS = JT_J;
                for (int i=0; i<6; i++)
                    LHS[7*i] += lambda*LHS[7*i];
                
                RHS = JT_R;

                char uplo = 'U';
                __CLPK_integer n = 6;
                __CLPK_integer nrhs = 1;
                __CLPK_integer lDA = 6;
                __CLPK_integer ipiv[6];
                __CLPK_integer lDB = 6;
                __CLPK_doublereal worksize;
                __CLPK_integer lwork = -1;
                __CLPK_integer info;
                dsysv_(&uplo, &n, &nrhs, LHS.data(), &lDA, ipiv, RHS.data(), &lDB, &worksize, &lwork, &info);
                std::vector<__CLPK_doublereal> work((int)worksize);
                lwork = (__CLPK_integer)worksize;
                dsysv_(&uplo, &n, &nrhs, LHS.data(), &lDA, ipiv, RHS.data(), &lDB, work.data(), &lwork, &info);
                
                std::array<double, 6> parametersUpdated;
                for (int i=0; i<6; i++)
                    parametersUpdated[i] = parameters[i]+RHS[i];
                
                SE3 poseUpdated;
                poseUpdated.R() = EulerAngles(parametersUpdated[2], parametersUpdated[1], parametersUpdated[0]).matrix();
                poseUpdated.t().x() = parametersUpdated[3];
                poseUpdated.t().y() = parametersUpdated[4];
                poseUpdated.t().z() = parametersUpdated[5];
                
                reproject(poseUpdated, points, intrinsics, reprojections.data());
                computeResiduals(features, reprojections, residuals);
                
                double X2Updated; // Chi-Square error criteria
                vDSP_svesqD((double*)residuals.data(), 1, &X2Updated, NDCs.size()*2);
                X2Updated *= 0.5;
                
                const double relativeCostChange = fabs(X2Updated-X2)/X2;
                double rhoDenom = 0.0;
                for (int i=0; i<6; i++)
                    rhoDenom -= 0.5*RHS[i]*(lambda*RHS[i]-JT_R[i]);
            
                const double rho = (X2-X2Updated)/rhoDenom; // Nielsen
                
                if (rho > StepEps)
                {
                    // updated parameters are significiantly better
                    X2 = X2Updated;
                    parameters = parametersUpdated;
                    pose = poseUpdated;
                    
                    jacobian(points, intrinsics, parameters, J, JT);
                    
                    vDSP_mmulD(JT.data(), 1, J.data(), 1, JT_J.data(), 1, 6, 6, NDCs.size()*2);
                    vDSP_mmulD(JT.data(), 1, (double*)residuals.data(), 1, JT_R.data(), 1, 6, 1, NDCs.size()*2);
                    
                    lambda = std::max(lambda/lambdaIncreaseFactor, LambdaMin);
                } else
                {
                    // just increase lambda
                    lambda *= lambdaIncreaseFactor;
                    lambda = std::min(lambda, LambdaMax);
                    lambdaIncreaseFactor *= 2.0;
                }
                
                if (iterations == Settings::PNPRefineNonLinearMaxIterations)
                {
                    // Maximum number of iterations reached
                    stop = true;
                    continue;
                }
                
                if (X2/(NDCs.size()*2.0) < ChiSquareEps)
                {
                    // Average Chi-Square Error small enough
                    stop = true;
                    continue;
                }
                
                double maxParameterChange = 0.0;
                for (int i=0; i<6; i++)
                    maxParameterChange = std::max(maxParameterChange, fabs(RHS[i]/parameters[i]));
                
                if (maxParameterChange < ParameterEps)
                {
                    // Parameters converged
                    stop = true;
                    continue;
                }
                
                double maxRHS = 0.0;
                for (int i=0; i<6; i++)
                    maxRHS = std::max(maxRHS,fabs(JT_R[i]));
            
                if (maxRHS < ResidualEps)
                {
                    // Parameters converged
                    stop = true;
                    continue;
                }
                
                if (relativeCostChange < CostEps)
                {
                    stop = true;
                    continue;
                }
                
                if (rho > StepEps)
                {
                    
                    jacobian(points, intrinsics, parameters, J, JT);
                    
                    vDSP_mmulD(JT.data(), 1, J.data(), 1, JT_J.data(), 1, 6, 6, NDCs.size()*2);
                    vDSP_mmulD(JT.data(), 1, (double*)residuals.data(), 1, JT_R.data(), 1, 6, 1, NDCs.size()*2);
                    
                    double twoRhoMinusOne = 2.0*rho-1.0;
                    lambda *= std::max(1.0/3.0,1.0-twoRhoMinusOne*twoRhoMinusOne*twoRhoMinusOne);
                    lambdaIncreaseFactor = 2.0;
                }
            }
        }
        
        std::vector<int> PNP(SE3& pose, /*const*/ std::vector<Eigen::Vector3d>& NDCs,/* const*/ std::vector<Eigen::Vector3d>& points, const Intrinsics& intrinsics, const float& inlierThreshold)
        {
            assert(NDCs.size() == points.size());
            
            std::vector<int> inliers;
            
            if (NDCs.size() < 3)
                return inliers;
            
            ////
            // Normalize points
            
            NormalizationTransform normalizationTransform(points);
            std::vector<Eigen::Vector3d> pointsNormalized = points;
            for (Eigen::Vector3d& p : pointsNormalized)
                p = normalizationTransform.normalize(p);
            
            ////
            // Ransac P3P
            inliers = ransacP3P(pose, NDCs, pointsNormalized, inlierThreshold, intrinsics);
            
            ////
            // Collect inliers
            
            std::vector<Eigen::Vector3d> inlierNDCs(inliers.size());
            for (int i=0; i<inliers.size(); ++i)
            {
                inlierNDCs[i] = NDCs[inliers[i]];
                pointsNormalized[i] = pointsNormalized[inliers[i]];
            }
            pointsNormalized.resize(inliers.size());
            
            ////
            // Optimize
            PNPRefineNonLinear(pose, inlierNDCs, pointsNormalized, intrinsics);
            
            ////
            // Denormalize pose
            
            pose = normalizationTransform.denormalize(pose);
            
            return inliers;
        }
    }
}
