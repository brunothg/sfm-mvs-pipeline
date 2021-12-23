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

#include "SimpleCamera.h"

#include <ceres/rotation.h>
#include "photogrammetrie/util/OpenCvUtils.h"

namespace photogrammetrie {
    SimpleCamera::SimpleCamera() {
        K = (cv::Mat_<double>(3, 3) << 2500, 0, 0, 0, 2500, 0, 0, 0, 1);
    }

    void SimpleCamera::getK(cv::Mat_<double> &kOut) const {
        K.copyTo(kOut);
    }

    void SimpleCamera::setK(const cv::Mat_<double> &newK) {
        double focalX = newK.at<double>(0, 0), focalY = newK.at<double>(1, 1);
        double focal = (focalX == focalY) ? focalX : (focalX + focalY) / 2.0;
        K.at<double>(0, 0) = focal;
        K.at<double>(1, 1) = focal;
        K.at<double>(0, 2) = newK.at<double>(0, 2);
        K.at<double>(1, 2) = newK.at<double>(1, 2);
    }

    cv::Size SimpleCamera::getResolution() const {
        double cx, cy;
        cx = K.at<double>(0, 2);
        cy = K.at<double>(1, 2);

        return cv::Size(round(cx * 2.0), round(cy * 2.0));
    }

    void SimpleCamera::setResolution(const cv::Size &resolution) {
        double cx, cy;
        cx = (double)resolution.width / 2.0;
        cy = (double)resolution.height / 2.0;

        K.at<double>(0, 2) = cx;
        K.at<double>(1, 2) = cy;
    }

    string SimpleCamera::toString() const {
        return "SimpleCamera(K=\n" + OpenCvUtils::toString(K) + ")";
    }

    struct SimpleCameraCostFunction {
        double observed_x;
        double observed_y;

        double centerX;
        double centerY;

        SimpleCameraCostFunction(double observed_x, double observed_y, double centerX, double centerY) :
                observed_x(observed_x), observed_y(observed_y), centerX(centerX), centerY(centerY) {}

        template<typename T>
        bool operator()(const T *const coordinates3d,
                        const T *const cameraPose,
                        const T *const cameraParameters,
                        T *residuals) const {

            // Rotieren
            T p[3];
            ceres::AngleAxisRotatePoint(cameraPose, coordinates3d, p);
            // Verschieben
            p[0] += cameraPose[3];
            p[1] += cameraPose[4];
            p[2] += cameraPose[5];

            // Perspektiv Korrektur (3D -> 2D)
            T xp = p[0] / p[2];
            T yp = p[1] / p[2];

            // Projektion
            const T &focal = cameraParameters[0];
            T xd = focal * xp;
            T yd = focal * yp;

            T xu = xd;
            T yu = yd;

            // cost function
            residuals[0] = xu - (T(observed_x) - T(centerX));
            residuals[1] = yu - (T(observed_y) - T(centerY));
            return true;
        }
    };

    ceres::CostFunction *SimpleCamera::ceresCostFunction(const cv::Point2f &observedPoint) {
        double centerX, centerY;
        getCenter(centerX, centerY);

        return (new ceres::AutoDiffCostFunction<SimpleCameraCostFunction, 2, 3, 6, SimpleCamera::CeresRepresentationSize>(
                new SimpleCameraCostFunction(observedPoint.x, observedPoint.y, centerX, centerY)));
    }

    double *SimpleCamera::ceresCameraParameters(bool update) {
        if (update) {
            getFocalLength(ceresRepresentation[0]);
        }

        return ceresRepresentation;
    }

    void SimpleCamera::ceresApplyParameters() {
        setFocalLength(ceresRepresentation[0]);
    }

    SimpleCamera::~SimpleCamera() = default;
}