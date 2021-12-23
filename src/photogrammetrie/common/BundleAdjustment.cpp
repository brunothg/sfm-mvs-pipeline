/*
* Photogrammetrie - SfM/MVS 3D reconstruction from 2D photos
* Copyright (C) 2021  Marvin Bruns
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Affero General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Affero General Public License for more details.
*
* You should have received a copy of the GNU Affero General Public License
* along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "BundleAdjustment.h"

#include <algorithm>
#include "photogrammetrie/util/OpenMpUtils.h"
#include "photogrammetrie/util/CeresUtils.h"

namespace photogrammetrie {

    const AppLogger BundleAdjustment::logger = AppLogger("BundleAdjustment");

    void BundleAdjustment::doBundleAdjustment(const Scene &scene, vector <shared_ptr<CeresPCE>> &ceresPce,
                                              vector <shared_ptr<ICamera>> &ceresCameras,
                                              vector <shared_ptr<CeresCameraShot>> &ceresShots,
                                              ceres::Solver::Summary &summary) {
        BundleAdjustment::logger.info("Führe Bündelblockausgleichung durch");
        ceres::Problem problem;

        ceresPce.clear();
        ceresPce.reserve(scene.getPointcloud().size());

        ceresCameras.clear();
        ceresCameras.reserve(scene.getCameras().size());

        ceresShots.clear();
        ceresShots.reserve(scene.getShots().size());

        for (auto &camera : scene.getCameras()) {
            camera->ceresCameraParameters();
            ceresCameras.push_back(camera);
        }

        for (auto &pce : scene.getPointcloud()) {
            // Pointcloud 3D-Punkt
            shared_ptr <CeresPCE> ceres3dPoint = make_shared<CeresPCE>();
            ceres3dPoint->pce = pce;
            ceres3dPoint->read();
            ceresPce.push_back(ceres3dPoint);

            vector < pair < shared_ptr < CameraShot > , cv::Point2d >> originPoints;
            pce->getOriginPoints(originPoints);
            for (auto &originPoint : originPoints) {
                auto &shot = originPoint.first;
                auto &point = originPoint.second;

                shared_ptr <CeresCameraShot> ceresShot;
                {
                    // Achtung: OMP
                    auto existingCeresShot = OpenMpUtils::find_if(ceresShots.begin(), ceresShots.end(),
                                                               [&shot](shared_ptr <CeresCameraShot> &it) {
                                                                   return it->shot == shot;
                                                               });

                    if (existingCeresShot == ceresShots.end()) {
                        ceresShot = make_shared<CeresCameraShot>();
                        ceresShot->shot = shot;
                        ceresShot->read();
                        ceresShots.push_back(ceresShot);
                    } else {
                        ceresShot = *existingCeresShot;
                    }
                }

                auto &ceresCamera = shot->getCamera();

                ceres::CostFunction *costFunction = ceresCamera->ceresCostFunction(point);
                double *ceres3dCoordinates = ceres3dPoint->coordinates;
                double *ceresCameraPose = ceresShot->pose;
                double *ceresCameraParameter = ceresCamera->ceresCameraParameters(false);

                problem.AddResidualBlock(costFunction, nullptr /* squared loss */, ceres3dCoordinates, ceresCameraPose,
                                         ceresCameraParameter);
            }
        }

        CeresUtils::solve(problem, summary);
        BundleAdjustment::logger.info("Ergebnis der Bündelblockausgleichung: \n" + summary.FullReport());
    }

    bool BundleAdjustment::doBundleAdjustment(photogrammetrie::Scene &scene) {


        vector <shared_ptr<CeresPCE>> ceresPces;
        vector <shared_ptr<CeresCameraShot>> ceresShots;
        vector <shared_ptr<ICamera>> ceresCameras;

        ceres::Solver::Summary summary;
        BundleAdjustment::doBundleAdjustment(scene, ceresPces, ceresCameras, ceresShots, summary);

#pragma omp parallel for default(none) shared(ceresPces)
        for (auto &ceresPce : ceresPces) {
            string logMsg = "PCE(before): \n";
            logMsg += ceresPce->pce->toString();

            ceresPce->write();

            logMsg += " \n\nPCE(after): \n" + ceresPce->pce->toString();
            BundleAdjustment::logger.debug(logMsg);
        }

#pragma omp parallel for default(none) shared(ceresShots)
        for (auto &ceresShot : ceresShots) {
            string logMsg = "Shot(before): \n";
            logMsg += ceresShot->shot->toString();

            ceresShot->write();

            logMsg += " \n\nShot(after): \n" + ceresShot->shot->toString();
            BundleAdjustment::logger.debug(logMsg);
        }

#pragma omp parallel for default(none) shared(ceresCameras)
        for (auto &ceresCamera : ceresCameras) {
            string logMsg = "Camera(before): \n";
            logMsg += ceresCamera->toString();

            ceresCamera->ceresApplyParameters();

            logMsg += " \n\nCamera(after): \n" + ceresCamera->toString();
            BundleAdjustment::logger.debug(logMsg);
        }

        return summary.termination_type == ceres::CONVERGENCE;
    }

    void CeresPCE::write() {
        if (pce == nullptr) {
            return;
        }

        pce->setCoordinates(cv::Point3d(coordinates[0], coordinates[1], coordinates[2]));
    }

    void CeresPCE::read() {
        if (pce == nullptr) {
            coordinates[0] = 0;
            coordinates[1] = 0;
            coordinates[2] = 0;
            return;
        }

        auto &pceCoordinates = pce->getCoordinates();
        coordinates[0] = pceCoordinates.x;
        coordinates[1] = pceCoordinates.y;
        coordinates[2] = pceCoordinates.z;
    }

    void CeresCameraShot::read() {
        if (shot == nullptr) {
            for (double &i : pose) {
                i = 0;
            }
            return;
        }

        CeresUtils::toCeresPose<double>(shot->getPose(), pose);
    }

    void CeresCameraShot::write() {
        if (shot == nullptr) {
            return;
        }

        cv::Mat_<double> cvPose(3, 4);
        CeresUtils::toOpenCvPose<double>(cvPose, pose);
        shot->setPose(cvPose);
    }
}
