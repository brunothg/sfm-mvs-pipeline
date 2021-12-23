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

#include "DistortionCamera.h"

#include <memory>
#include <ceres/rotation.h>
#include "photogrammetrie/util/OpenCvUtils.h"

namespace photogrammetrie {

    DistortionCamera::DistortionCamera() {
        K = (cv::Mat_<double>(3, 3) << 2500, 0, 0, 0, 2500, 0, 0, 0, 1);
        distortion = cv::Mat_<double>::zeros(1, 4);
    }

    void DistortionCamera::getK(cv::Mat_<double> &kOut) const {
        K.copyTo(kOut);
    }

    void DistortionCamera::setK(const cv::Mat_<double> &newK) {
        double focalX = newK.at<double>(0, 0), focalY = newK.at<double>(1, 1);
        double focal = (focalX == focalY) ? focalX : (focalX + focalY) / 2.0;
        K.at<double>(0, 0) = focal;
        K.at<double>(1, 1) = focal;
        K.at<double>(0, 2) = newK.at<double>(0, 2);
        K.at<double>(1, 2) = newK.at<double>(1, 2);
    }

    cv::Size DistortionCamera::getResolution() const {
        return resolution;
    }

    void DistortionCamera::setResolution(const cv::Size &newResolution) {
        resolution = newResolution;
    }

    void DistortionCamera::getDistortion(cv::Mat_<double> &distortionOut) const {
        distortion.copyTo(distortionOut);
    };

    string DistortionCamera::toString() const {
        return "DistortionCamera(K=\n" + OpenCvUtils::toString(K) + "\nDistortion=\n" +
               OpenCvUtils::toString(distortion) + ")";
    }

    struct DistortionCameraCostFunction {
        double observed_x;
        double observed_y;

        DistortionCameraCostFunction(double observed_x, double observed_y) :
                observed_x(observed_x), observed_y(observed_y) {}

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

            // Verzeichnung
            const T &k1 = cameraParameters[3];
            const T &k2 = cameraParameters[4];
            const T &p1 = cameraParameters[5];
            const T &p2 = cameraParameters[6];
            T r2 = (xp * xp) + (yp * yp);
            T r4 = r2 * r2;

            T xu = xd + xd * (k1 * r2 + k2 * r4) /* radial */ +
                   (p1 * (r2 + 2.0 * (xd * xd)) + 2.0 * p2 * xd * yd) /* tangential */;
            T yu = yd + yd * (k1 * r2 + k2 * r4) /* radial */ +
                   (2.0 * p1 * xd * yd + p2 * (r2 + 2.0 * (yd * yd))) /* tangential */;

            // cost function
            const T &centerX = cameraParameters[1];
            const T &centerY = cameraParameters[2];
            residuals[0] = xu - (T(observed_x) - centerX);
            residuals[1] = yu - (T(observed_y) - centerY);
            return true;
        }
    };

    ceres::CostFunction *DistortionCamera::ceresCostFunction(const cv::Point2f &observedPoint) {
        return (new ceres::AutoDiffCostFunction<DistortionCameraCostFunction, 2, 3, 6, DistortionCamera::CeresRepresentationSize>(
                new DistortionCameraCostFunction(observedPoint.x, observedPoint.y)));
    }

    double *DistortionCamera::ceresCameraParameters(bool update) {
        if (update) {
            getFocalLength(ceresRepresentation[0]);
            getCenter(ceresRepresentation[1], ceresRepresentation[2]);
            ceresRepresentation[3] = distortion.at<double>(0, 0);
            ceresRepresentation[4] = distortion.at<double>(0, 1);
            ceresRepresentation[5] = distortion.at<double>(0, 2);
            ceresRepresentation[6] = distortion.at<double>(0, 3);
        }

        return ceresRepresentation;
    }

    void DistortionCamera::ceresApplyParameters() {
        setFocalLength(ceresRepresentation[0]);
        setCenter(ceresRepresentation[1], ceresRepresentation[2]);
        distortion.at<double>(0, 0) = ceresRepresentation[3];
        distortion.at<double>(0, 1) = ceresRepresentation[4];
        distortion.at<double>(0, 2) = ceresRepresentation[5];
        distortion.at<double>(0, 3) = ceresRepresentation[6];
    }

    DistortionCamera::~DistortionCamera() = default;
}