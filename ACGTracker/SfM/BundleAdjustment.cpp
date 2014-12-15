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

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <unordered_map>
#include <iostream>
#include <vector>
#include <array>

#include "BundleAdjustment.h"
#include "Map/MapPoint.h"
#include "Map/Camera.h"
#include "Types/Intrinsics.h"
#include "Utils/NormalizationTransform.h"
#include "Utils/Metrics.h"

namespace ACGT
{
    struct ReprojectionError
    {
        ReprojectionError(double observedX, double observedY, Intrinsics intrinsics)
        : observedX(observedX), observedY(observedY), intrinsics(intrinsics) {}
        
        template <typename T>
        bool operator()(const T* const cameraRotation,
                        const T* const cameraTranslation,
                        const T* const point,
                        T* residuals) const
        {
            
            // Use a quaternion rotation that doesn't assume the quaternion is
            // normalized, since one of the ways to run the bundler is to let Ceres
            // optimize all 4 quaternion parameters unconstrained.
            T p[3];
            ceres::QuaternionRotatePoint(cameraRotation, point, p);
            
            p[0] += cameraTranslation[0];
            p[1] += cameraTranslation[1];
            p[2] += cameraTranslation[2];
            
            if(p[2] == T(0)) return false;
            
            // Compute the center of distortion. The sign change comes from
            // the camera model that Noah Snavely's Bundler assumes, whereby
            // the camera coordinate system has a negative z axis.
            T predictedX = T(-intrinsics.f()) * p[0] / p[2] + T(intrinsics.pX());
            T predictedY = T(-intrinsics.f()) * p[1] / p[2] + T(intrinsics.pY());
            
            // The error is the difference between the predicted and observed position.
            residuals[0] = predictedX - T(observedX);
            residuals[1] = predictedY - T(observedY);
            
            return true;
        }
        
        double observedX;
        double observedY;
        Intrinsics intrinsics;
    };
    
    struct ScaledReprojectionError
    {
        ScaledReprojectionError(double observedX, double observedY, Intrinsics intrinsics, double scaleFactor)
        : observedX(observedX), observedY(observedY), intrinsics(intrinsics), scaleFactor(scaleFactor) {}
        
        template <typename T>
        bool operator()(const T* const cameraRotation,
                        const T* const cameraTranslation,
                        const T* const point,
                        T* residuals) const
        {
            
            // Use a quaternion rotation that doesn't assume the quaternion is
            // normalized, since one of the ways to run the bundler is to let Ceres
            // optimize all 4 quaternion parameters unconstrained.
            T p[3];
            ceres::QuaternionRotatePoint(cameraRotation, point, p);
            
            p[0] += cameraTranslation[0];
            p[1] += cameraTranslation[1];
            p[2] += cameraTranslation[2];
            
            if(p[2] == T(0)) return false;
            
            // Compute the center of distortion. The sign change comes from
            // the camera model that Noah Snavely's Bundler assumes, whereby
            // the camera coordinate system has a negative z axis.
            T predictedX = T(-intrinsics.f()) * p[0] / p[2] + T(intrinsics.pX());
            T predictedY = T(-intrinsics.f()) * p[1] / p[2] + T(intrinsics.pY());
            
            // The error is the difference between the predicted and observed position.
            residuals[0] = scaleFactor*(predictedX - T(observedX));
            residuals[1] = scaleFactor*(predictedY - T(observedY));
            
            return true;
        }
        
        double observedX;
        double observedY;
        Intrinsics intrinsics;
        double scaleFactor;
    };
    
    void bundleAdjustment(std::unordered_map<Camera::ID, Camera>& cameras, std::unordered_map<MapPoint::ID, MapPoint>& points)
    {
        ////
        // Normalize data
        
        std::vector<Eigen::Vector3d> coordinates;
        coordinates.reserve(points.size());
        for (const auto& point : points)
            coordinates.push_back(point.second.coordinates());
        
        NormalizationTransform normalizationTransform(coordinates);
        
        for (auto& point : points)
            point.second.coordinatesOpt() = normalizationTransform.normalize(point.second.coordinates());
        
        for (auto& camera : cameras)
            camera.second.poseOpt() = normalizationTransform.normalize(camera.second.pose());
        
        ////
        // Get quaternions from the camera's rotation matrices
        std::unordered_map<Camera::ID, std::array<double,4>> cameraQuaternions;
        for (const auto& camera : cameras)
        {
            Eigen::Quaterniond quat(camera.second.poseOpt().R());
            cameraQuaternions[camera.first][0] = quat.w();
            cameraQuaternions[camera.first][1] = quat.x();
            cameraQuaternions[camera.first][2] = quat.y();
            cameraQuaternions[camera.first][3] = quat.z();
        }
        
        ////
        // Scale initial cost to 1.0
        double initialCost = 0.0;
        for (auto& camera : cameras)
        {
            const auto& pointReferences = camera.second.pointReferences();
            const auto& features = camera.second.features();
            for (int i=0; i<pointReferences.size(); ++i)
            {
                if (pointReferences[i] <= 0)
                    continue;
                
                Eigen::Vector3d pDash = camera.second.pose().R()*points[pointReferences[i]].coordinates()+camera.second.pose().t();
                initialCost += Metrics::squaredReprojectionError(pDash, features[i].NDCs(camera.second), camera.second.intrinsics());
            }
        }
        double scaleFactor = std::sqrt(2.0/initialCost);
        
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_SCHUR;
        options.sparse_linear_algebra_library = ceres::CX_SPARSE;
        options.trust_region_strategy_type = ceres::DOGLEG;
        options.num_linear_solver_threads =  1;
        options.num_threads =  1;
        options.max_solver_time_in_seconds = 10;
        options.minimizer_progress_to_stdout = false;
        options.min_relative_decrease = 0.5;
        options.function_tolerance = 1e-8;

        ////
        // Configure the problem
        
        ceres::Problem problem;
        ceres::LocalParameterization* quaternionParameterization = new ceres::QuaternionParameterization;

        for (auto& camera : cameras)
        {
            const auto& pointReferences = camera.second.pointReferences();
            const auto& features = camera.second.features();
            const Intrinsics& intrinsics = camera.second.intrinsics();
            double* quat = cameraQuaternions[camera.first].data();
            
            for (int i=0; i<pointReferences.size(); ++i)
            {
                if (pointReferences[i] <= 0)
                    continue;
                
                MapPoint& p = points[pointReferences[i]];
                const Feature& feature = features[i];

                ceres::LossFunction* lossFunction = nullptr;
                {

                    ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<ScaledReprojectionError, 2, 4, 3, 3>(new ScaledReprojectionError(feature.x(), (float)camera.second.image().height()-1.0f-feature.y(), intrinsics,scaleFactor));
                    problem.AddResidualBlock(costFunction,
                                             lossFunction,
                                             quat,
                                             (double*)&camera.second.poseOpt().t(),
                                             (double*)&p.coordinatesOpt());
                }

            }
            problem.SetParameterization(quat, quaternionParameterization);
        }

        ////
        // Solve the problem

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        //std::cout << summary.FullReport() << std::endl;

        ////
        // Denormalize points and data
        
        for (auto& point : points)
            point.second.coordinatesOpt() = normalizationTransform.denormalize(point.second.coordinatesOpt());
        
        for (auto& camera : cameras)
        {
            Eigen::Quaterniond quat;
            quat.w() = cameraQuaternions[camera.first][0];
            quat.x() = cameraQuaternions[camera.first][1];
            quat.y() = cameraQuaternions[camera.first][2];
            quat.z() = cameraQuaternions[camera.first][3];
            camera.second.poseOpt().R() = Eigen::Matrix3d(quat);
            camera.second.poseOpt() = normalizationTransform.denormalize(camera.second.poseOpt());
        }
    }
}