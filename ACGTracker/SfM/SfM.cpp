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

#include <unordered_map>
#include <vector>
#include <Accelerate/Accelerate.h>

#include "SfM.h"
#include "Map/Camera.h"
#include "Map/MapPoint.h"
#include "Features/Match.h"
#include "Features/BRISK/BRISKDescription.h"
#include "Features/TwoViewMatching.h"
#include "Types/EulerAngles.h"
#include "EssentialMatrix3PtSolver.h"
#include "Utils/Metrics.h"
#include "Settings/Settings.h"
#include "Utils/Triangulation.h"

namespace ACGT
{
    /**
     * Extracts the SE(3) camera motion from an essential matrix of the form E = [t]*R(yaw) and the pitch and roll angles and triangulates the correspondences to 3D points
     * The method allocates memory for the reconstruction->points array.
     * The caller is responsible to free the memory allocated by the function for the reconstruction->points array.
     **/
    std::vector<Eigen::Vector3d> reconstructFromEssentialMatrix(const Eigen::Matrix3d& E, std::vector<Eigen::Vector3d>& NDCs1, std::vector<Eigen::Vector3d>& NDCs2, Camera& camera1, Camera& camera2)
    {
        assert(NDCs1.size() == NDCs2.size());
        
        std::vector<Eigen::Vector3d> reconstruction;
        
        const double tX = -E(1,2);
        const double tZ = E(1,0);
        double tY;
        double yaw1;
        double yaw2;
        
        // epsilon for double comparisons
        const double epsilon = 0.00001;
        
        
        if (fabs(tX) < epsilon && fabs(tZ) < epsilon)
        {
            // Cannot reconstruct, since tX and tZ are equal to zero
            return reconstruction;
        }
        
        const double a = tX;
        const double b = -tZ;
        const double c = E(0,1);
        const double R = sqrt(a*a+b*b);
        const double phi = atan2(b, a);
        yaw1 = asin(c/R)-phi;
        yaw2 = M_PI-asin(c/R)-phi;
        const double sin1 = sin(yaw1);
        const double cos1 = cos(yaw1);
        const double sin2 = sin(yaw2);
        const double cos2 = cos(yaw2);
        
        // compute tY and discard yaw candidates inconsistent with E:
        double yaws[2];
        int numYaws = 0;
        if (fabs(E(0,0)) < epsilon && fabs(E(2,0)) < epsilon)
        {
            // ty equals or very close to zero, cannot decide on a yaw, check both yaws
            tY = 0.0;
            numYaws = 2;
            yaws[0] = yaw1;
            yaws[1] = yaw2;
        } else
        {
            double tYTmp;
            // check consistency of yaw1 with the essential matrix:
            if (fabs(sin1) >= epsilon)
            {
                tYTmp = -E(0,0)/sin1;
                if (fabs(-tYTmp*cos1-E(2,0)) < epsilon)
                {
                    // yaw1 consistent with E
                    tY = tYTmp;
                    yaws[numYaws] = yaw1;
                    numYaws++;
                } // else yaw1 inconsistent
            } else
            {
                tYTmp = -E(2,0)/cos1;
                if (fabs(E(0,0)) < epsilon)
                {
                    // yaw1 consistent with E
                    tY = tYTmp;
                    yaws[numYaws] = yaw1;
                    numYaws++;
                } // else yaw1 inconsistent
            }
            
            // check consistency of yaw2 with the essential matrix:
            if (fabs(sin2) >= epsilon)
            {
                tYTmp = -E(0,0)/sin2;
                if (fabs(-tYTmp*cos2-E(2,0)) < epsilon)
                {
                    // yaw2 consistent with E
                    tY = tYTmp;
                    yaws[numYaws] = yaw2;
                    numYaws++;
                } // else yaw2 inconsistent
            } else
            {
                tYTmp = -E(2,0)/cos2;
                if (fabs(E(0,0)) < epsilon)
                {
                    // yaw2 consistent with E
                    tY = tYTmp;
                    yaws[numYaws] = yaw2;
                    numYaws++;
                } // else yaw2 inconsistent
            }
            if (numYaws == 0)
            {
                // Something is wrong, no yaw consistent with essential matrix!
                return reconstruction;
            }
            
        }
        
        // Set pose of first camera
        camera1.pose().R() = EulerAngles(camera1.gravity()).matrix();
        camera1.pose().t() = Eigen::Vector3d::Zero();
        
        // Check cheirality constraint for each candidate
        
        Eigen::Matrix3d RrRp2 = EulerAngles(camera2.gravity()).matrix();
        
        std::vector<Eigen::Vector3d> reconstructionTmp(NDCs1.size());
        reconstruction.resize(NDCs1.size());
        int bestScore = -1;
        SE3 bestPose;
        
        std::vector<double> RPz1(NDCs1.size()); // (R1*P).z for all triangulated points P
        std::vector<double> RPz2(NDCs1.size()); // (R2*P).z for all triangulated points P
        
        // double bestRoll;
        for (int i=0; i<numYaws; i++) {
            
            camera2.pose().R() = RrRp2*EulerAngles(0.0, 0.0, yaws[i]).matrix();
            camera2.pose().t() << tX, tY, tZ;
            camera2.pose().t() = camera2.pose().R()*camera2.pose().t();
            camera2.pose().t().normalize();
            
            const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R1 = camera1.pose().R();
            const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R2 = camera2.pose().R();
            
            // triangulate for (tX,tY,tZ)^T and count 3D points that fulfil the cheirality constraint
            Triangulation::triangulateCorrespondences(reconstructionTmp, NDCs1, NDCs2, camera1, camera2);
            
            // compute (R*P).z for all triangulated points p
            vDSP_mmulD((double*)reconstructionTmp.data(), 1, (double*)R1.data()+6, 1, RPz1.data(), 1, NDCs1.size(), 1, 3);
            vDSP_mmulD((double*)reconstructionTmp.data(), 1, (double*)R2.data()+6, 1, RPz2.data(), 1, NDCs2.size(), 1, 3);
            
            // count points that fulfil the cheirality constraint
            int candidateScore = 0;
            for (int y=0; y<NDCs1.size(); ++y)
            {
                if (RPz1[y] < 0.0 && RPz2[y]+camera2.pose().t().z() < 0.0)
                    candidateScore++;
            }
            
            // update motion if necessary
            if (candidateScore > bestScore)
            {
                bestScore = candidateScore;
                bestPose = camera2.pose();
                reconstruction.swap(reconstructionTmp);
            }
            
            // triangulate for (-tX,-tY,-tZ)^T and count 3D points that fulfil the cheirality constraint
            camera2.pose().t() *= -1.0;
            Triangulation::triangulateCorrespondences(reconstructionTmp, NDCs1, NDCs2, camera1, camera2);
            
            // compute (R*P).z for all triangulated points p
            vDSP_mmulD((double*)reconstructionTmp.data(), 1, (double*)R1.data()+6, 1, RPz1.data(), 1, NDCs1.size(), 1, 3);
            vDSP_mmulD((double*)reconstructionTmp.data(), 1, (double*)R2.data()+6, 1, RPz2.data(), 1, NDCs2.size(), 1, 3);
            
            // count points that fulfil the cheirality constraint
            candidateScore = 0;
            for (int y=0; y<NDCs1.size(); ++y)
            {
                if (RPz1[y] < 0.0 && RPz2[y]+camera2.pose().t().z() < 0.0)
                    candidateScore++;
            }
            
            // update motion if necessary
            if (candidateScore > bestScore)
            {
                bestScore = candidateScore;
                bestPose = camera2.pose();
                reconstruction.swap(reconstructionTmp);
            }
        }
        
        // Set camera2's pose and reconstruction
        camera2.pose() = bestPose;
        return reconstruction;
    }
    
    /**
     * Counts the inlier matches of a given essential matrix corresponding to a threshold.
     * Stores the indices of the inlier matches in the inlier array.
     * The function allocates memory for the inlierIndices array.
     * The caller is responsible to free the memory allocated by the function for the inlierIndices array.
     **/
    void countInliers(const Eigen::Matrix3d& E, const std::vector<Eigen::Vector3d>& NDCs1, const std::vector<Eigen::Vector3d>& NDCs2, const double& threshold, std::vector<int>& inliers)
    {
        assert(NDCs1.size() == NDCs2.size());
        
        inliers.clear();
        
        for (int i=0; i<NDCs1.size(); ++i)
        {
            if (Metrics::sampsonusError(NDCs1[i], NDCs2[i], E) < threshold)
                inliers.push_back(i);
        }
    }
    
    /**
     * Computes the SE(3) CameraPose that describes the camera motion (rotation & translation) between the first and the second camera.
     * After the call, camera1's pose and camera2's pose are set with respect to a reference frame, in which the y-axis is aligned with gravity and camera1's center of projection is the origin.
     * Additionally, a 3D scene reconstruction is computed.
     * It is assumed that features and descriptors have already been computed for both cameras and that both camera's gravity vectors have been set.
     * Returns the computed 3D reconstruction in an unordered_map. The values of the map are the computed map points and the keys are the corresponding point ids.
     **/
    std::unordered_map<long, MapPoint> reconstructFromStereo(Camera& camera1, Camera& camera2)
    {
        std::unordered_map<long,MapPoint> reconstruction;
        
        // Find matches
        std::vector<Match> matches = TwoViewMatching::matchNNMutualDR<BRISKDescriptor<Settings::DescriptorSize>>(camera1.descriptors(), camera2.descriptors(), Settings::MatchingThreshold,0.8);
        
        std::vector<Eigen::Vector3d> NDCs1;
        std::vector<Eigen::Vector3d> NDCs2;
        NDCs1.reserve(matches.size());
        NDCs2.reserve(matches.size());
        
        for (const auto& m : matches)
        {
            NDCs1.push_back(camera1.features()[m.idx1()].NDCs(camera1));
            NDCs2.push_back(camera2.features()[m.idx2()].NDCs(camera2));
        }
        
        // check if there are at least three correspondences
        if (matches.size() < 3)
        {
            // cannot reconstruct
            return reconstruction;
        }
        
        // Compute rotated NDCs
        
        std::vector<Eigen::Vector3d> NDCs1Rotated(matches.size());
        std::vector<Eigen::Vector3d> NDCs2Rotated(matches.size());
        
        Eigen::Matrix<double,3,3,Eigen::RowMajor> R1 = EulerAngles(camera1.gravity()).matrix();
        Eigen::Matrix<double,3,3,Eigen::RowMajor> R2 = EulerAngles(camera2.gravity()).matrix();
        
        vDSP_mmulD((double*)NDCs1.data(), 1, R1.data(), 1, (double*)NDCs1Rotated.data(), 1, NDCs1.size(), 3, 3);
        vDSP_mmulD((double*)NDCs2.data(), 1, R2.data(), 1, (double*)NDCs2Rotated.data(), 1, NDCs2.size(), 3, 3);
       
        // de-homogenize
        vDSP_vdivD((double*)NDCs1Rotated.data()+2, 3, (double*)NDCs1Rotated.data(), 3, (double*)NDCs1Rotated.data(), 3, NDCs1Rotated.size());
        vDSP_vdivD((double*)NDCs1Rotated.data()+2, 3, (double*)NDCs1Rotated.data()+1, 3, (double*)NDCs1Rotated.data()+1, 3, NDCs1Rotated.size());
        double one = 1.0;
        vDSP_vfillD(&one, (double*)NDCs1Rotated.data()+2, 3, NDCs1Rotated.size());
        vDSP_vdivD((double*)NDCs2Rotated.data()+2, 3, (double*)NDCs2Rotated.data(), 3, (double*)NDCs2Rotated.data(), 3, NDCs2Rotated.size());
        vDSP_vdivD((double*)NDCs2Rotated.data()+2, 3, (double*)NDCs2Rotated.data()+1, 3, (double*)NDCs2Rotated.data()+1, 3, NDCs2Rotated.size());
        vDSP_vfillD(&one, (double*)NDCs2Rotated.data()+2, 3, NDCs1Rotated.size());
        
        int sampleIndices[3];
        std::vector<Eigen::Vector3d> samples1(3);
        std::vector<Eigen::Vector3d> samples2(3);
        
        Eigen::Matrix3d E;
        std::vector<int> inliers;
        inliers.reserve(matches.size());
        
        Eigen::Matrix3d bestE;
        std::vector<int> bestInliers;
        bestInliers.reserve(matches.size());
        
        // initialze fraction of outlier matches to 0.99
        double eps = 0.99;
        // set probability of finding an outlier free sample to 0.99:
        const double log1_P = log(0.001);
        // initialize number of RANSAC iterations
        int numIterations = (int)ceil(log1_P/log(1.0-(1.0-eps)*(1.0-eps)*(1.0-eps)));
        int i=0; // processed RANSAC iterations
        
        srand(time(NULL));
        
        while (i<numIterations)
        {
            // choose random sample
            for (int j = 0; j<3; j++)
            {
                bool uniqueSample = false;
                while (!uniqueSample)
                {
                    uniqueSample = true;
                    sampleIndices[j] = (int)(matches.size()*((double)rand()/((double)(RAND_MAX)+1.0)));
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
                samples1[j] = NDCs1Rotated[sampleIndices[j]];
                samples2[j] = NDCs2Rotated[sampleIndices[j]];
            }
            
            // compute essential matrix candidates from samples
            std::vector<Eigen::Matrix3d> candidates = solveE3Pt(samples1, samples2);
            
            // check all candidates
            for (const auto& E : candidates)
            {
                // compute yaw:
                if (E(0,0) != 0.0 || E(2,0) != 0.0)
                {
                    double tY = sqrt(E(0,0)*E(0,0)+E(2,0)*E(2,0));
                    double yaw = atan2(E(0,0)/tY, E(2,0)/tY);
                    if (fabs(E(1,0)*sin(yaw)-E(1,2)*cos(yaw)-E(2,1))>0.0001)
                    {
                        tY = -tY;
                        if (yaw > 0.0)
                            yaw -= M_PI;
                        else
                            yaw += M_PI;
                    }
                    
                    // count inliers
                    countInliers(E,NDCs1Rotated,NDCs2Rotated,Settings::SampsonusErrorInlierThreshold,inliers);
                    if (inliers.size() > bestInliers.size())
                    {
                        bestInliers = inliers;
                        const double outlierRatio = 1.0 - (double)bestInliers.size()/(double)NDCs1.size();
                        if (outlierRatio == 0.0)
                            numIterations = 0;
                        else if ( outlierRatio < eps)
                        {
                            eps = outlierRatio;
                            numIterations = (int)ceil(log1_P/log(1.0-(1.0-eps)*(1.0-eps)*(1.0-eps)));
                        }
                        bestE = E;
                    }
                }
                
            }
            
            i++;
        }
        
        // Compute E from all inliers
        samples1.resize(bestInliers.size());
        samples2.resize(bestInliers.size());
        for (int i=0; i<bestInliers.size(); ++i)
        {
            samples1[i] = NDCs1Rotated[bestInliers[i]];
            samples2[i] = NDCs2Rotated[bestInliers[i]];
        }

        std::vector<Eigen::Matrix3d> candidates = solveE3Pt(samples1, samples2);

        // check all candidates
        for (const auto& E : candidates)
        {
            // compute yaw:
            if (E(0,0) != 0.0 || E(2,0) != 0.0)
            {
                double tY = sqrt(E(0,0)*E(0,0)+E(2,0)*E(2,0));
                double yaw = atan2(E(0,0)/tY, E(2,0)/tY);
                if (fabs(E(1,0)*sin(yaw)-E(1,2)*cos(yaw)-E(2,1))>0.0001)
                {
                    tY = -tY;
                    if (yaw > 0.0)
                        yaw -= M_PI;
                    else
                        yaw += M_PI;
                }
                
                // count inliers
                countInliers(E,NDCs1Rotated,NDCs2Rotated,Settings::SampsonusErrorInlierThreshold,inliers);
                if (inliers.size() > bestInliers.size())
                {
                    bestInliers = inliers;
                    const double outlierRatio = 1.0 - (double)bestInliers.size()/(double)NDCs1.size();
                    if (outlierRatio == 0.0)
                        numIterations = 0;
                    else if ( outlierRatio < eps)
                    {
                        eps = outlierRatio;
                        numIterations = (int)ceil(log1_P/log(1.0-(1.0-eps)*(1.0-eps)*(1.0-eps)));
                    }
                    bestE = E;
                }
            }
            
        }
        
        if (bestInliers.size() == 0)
            return reconstruction;
        
        // Reconstruct from E and inlier correspondences
        for (int i=0; i<bestInliers.size(); ++i)
        {
            NDCs1[i] = NDCs1[bestInliers[i]];
            NDCs2[i] = NDCs2[bestInliers[i]];
        }
        NDCs1.resize(bestInliers.size());
        NDCs2.resize(bestInliers.size());
        
        std::vector<Eigen::Vector3d> points = reconstructFromEssentialMatrix(bestE,NDCs1,NDCs2,camera1,camera2);
        for (int i=0; i<points.size(); ++i)
        {
            MapPoint m;
            m.coordinates() = points[i];
            m.addKeyframeReference(camera1, matches[bestInliers[i]].idx1());
            m.addKeyframeReference(camera2, matches[bestInliers[i]].idx2());
            camera1.addPointReference(matches[bestInliers[i]].idx1(), m.id());
            camera2.addPointReference(matches[bestInliers[i]].idx2(), m.id());
            reconstruction[m.id()] = std::move(m);
        }
        
        return reconstruction;
    }
}


