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

#include "SceneUtils.h"

#include <ostream>
#include <fstream>
#include <opencv2/calib3d.hpp>
#include "photogrammetrie/util/MathUtils.h"

namespace photogrammetrie {

    void SceneUtils::writeToReprojectionErrorStatisticsCSV(const Scene &scene, const filesystem::path &outPath) {

        ofstream ofs(outPath);
        ofs << fixed << "N_Points" << "," << "Max_Error" << "," << "Min_Error"
            << "," << "Avg_Error" << "," << "Median_Error" << "," << "Variance_Error" << ","
            << "Deviation_Error" << endl;

        size_t size = 0;
        double maxError = 0, minError = -1, avgError = 0, medianError = 0, varianceError = 0, deviationError = 0;

        vector<double> errors;

        auto &pointcloud = scene.getPointcloud();

#pragma omp parallel for default(none) shared(pointcloud, errors)
        for (auto &point3d : pointcloud) {
            vector<pair<shared_ptr<CameraShot>, cv::Point2d>> origins;
            point3d->getOriginPoints(origins);
            for (auto &origin : origins) {
                auto &shot = origin.first;
                auto &camera = shot->getCamera();
                auto &point2d = origin.second;

                vector<cv::Point3f> cvPoints3d;
                vector<cv::Point2f> cvPoints2d;
                cv::Mat_<double> K, distortion;
                camera->getK(K);
                camera->getDistortion(distortion);
                cvPoints3d.push_back(point3d->getCoordinates());
                cv::projectPoints(cvPoints3d, shot->getRotationVec(), shot->getTranslationVec(), K, distortion,
                                  cvPoints2d);

                if (!cvPoints2d.empty()) {
                    auto &point2dReprojected = cvPoints2d[0];
                    auto reprojectionError = cv::norm(point2d - cv::Point2d(point2dReprojected));
#pragma omp critical
                    {
                        errors.push_back(reprojectionError);
                    }
                }
            }
        }

        size = errors.size();
        MathUtils::calculateStatistics(errors, minError, maxError, avgError, varianceError,
                                       deviationError, medianError);

        ofs << size << "," << maxError << "," << minError
            << "," << avgError << "," << medianError << "," << varianceError << "," << deviationError
            << endl;

        ofs.close();
    }

    void SceneUtils::writeToReprojectionErrorHistogramCSV(const Scene &scene, const filesystem::path &outPath,
                                                          double resolution) {

        map<double, unsigned long long> errorMap;
        vector<double> errors;

        auto &pointcloud = scene.getPointcloud();

#pragma omp parallel for default(none) shared(pointcloud, errors)
        for (auto &point3d : pointcloud) {
            vector<pair<shared_ptr<CameraShot>, cv::Point2d>> origins;
            point3d->getOriginPoints(origins);
            for (auto &origin : origins) {
                auto &shot = origin.first;
                auto &camera = shot->getCamera();
                auto &point2d = origin.second;

                vector<cv::Point3f> cvPoints3d;
                vector<cv::Point2f> cvPoints2d;
                cv::Mat_<double> K, distortion;
                camera->getK(K);
                camera->getDistortion(distortion);
                cvPoints3d.push_back(point3d->getCoordinates());
                cv::projectPoints(cvPoints3d, shot->getRotationVec(), shot->getTranslationVec(), K, distortion,
                                  cvPoints2d);

                if (!cvPoints2d.empty()) {
                    auto &point2dReprojected = cvPoints2d[0];
                    auto reprojectionError = cv::norm(point2d - cv::Point2d(point2dReprojected));
#pragma omp critical
                    {
                        errors.push_back(reprojectionError);
                    }
                }
            }
        }

        // Auflösung automtaisch bestimmen (Varianz)
        if (resolution < 0) {
            double _, varianceError = 0;
            MathUtils::calculateStatistics(errors, _, _, _, varianceError, _, _);

            resolution = varianceError;
        }

        for (auto &error : errors) {
            double distanceId = (resolution <= 0) ? error : ceil(error / resolution) * resolution;
            if (errorMap.count(distanceId) == 0) {
                errorMap.insert(make_pair(distanceId, 1));
            } else {
                errorMap[distanceId] = errorMap[distanceId] + 1;
            }
        }

        ofstream ofs(outPath);
        ofs << fixed << "Error" << "," << "Count" << endl;

        for (auto &error : errorMap) {
            ofs << error.first << "," << error.second << endl;
        }

        ofs.close();
    }
}