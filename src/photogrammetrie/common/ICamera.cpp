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

#include "ICamera.h"

#include <opencv2/calib3d.hpp>


namespace photogrammetrie {


    void ICamera::getCenter(double &centerX, double &centerY) const {
        cv::Mat_<double> K;
        getK(K);

        centerX = K.at<double>(0, 2);
        centerY = K.at<double>(1, 2);
    }

    void ICamera::getFocalLength(double &focalLengthX, double &focalLengthY) const {
        cv::Mat_<double> K;
        getK(K);
        focalLengthX = K.at<double>(0, 0);
        focalLengthY = K.at<double>(1, 1);
    }

    void ICamera::setFocalLength(const double &focalLengthX, const double &focalLengthY) {
        cv::Mat_<double> K;
        getK(K);
        K.at<double>(0, 0) = focalLengthX;
        K.at<double>(1, 1) = focalLengthY;
        setK(K);
    }

    void ICamera::setCenter(const double &centerX, const double &centerY) {
        cv::Mat_<double> K;
        getK(K);
        K.at<double>(0, 2) = centerX;
        K.at<double>(1, 2) = centerY;
        setK(K);
    }

    void ICamera::setFocalLength(const double &focalLength) {
        setFocalLength(focalLength, focalLength);
    }

    void ICamera::getFocalLength(double &focalLength) const {
        double focalX, focalY;
        getFocalLength(focalX, focalY);
        focalLength = (focalX == focalY) ? focalX : (focalX + focalY) / 2.0;
    }

    void ICamera::getDistortion(cv::Mat_<double> &distortion) const {
        cv::Mat_<double>(cv::Mat_<double>::zeros(1, 4)).copyTo(distortion);
    }

    void ICamera::undistort(cv::InputArray in, cv::OutputArray out) const {
        cv::Mat_<double> K;
        getK(K);

        cv::Mat_<double> distortion;
        getDistortion(distortion);

        cv::undistort(in, out, K, distortion);
    }

    void ICamera::undistortPoints(cv::InputArray in, cv::OutputArray out) const {
        cv::Mat_<double> K;
        getK(K);

        cv::Mat_<double> distortion;
        getDistortion(distortion);

        cv::undistortPoints(in, out, K, distortion, cv::noArray(), K);
    }
}