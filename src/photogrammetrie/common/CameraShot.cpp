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

#include "CameraShot.h"

#include <utility>
#include <opencv2/calib3d.hpp>
#include "photogrammetrie/util/OpenCvUtils.h"

namespace photogrammetrie {

    CameraShot::CameraShot(shared_ptr<ICamera> camera, const filesystem::path &imagePath) :
            camera(std::move(camera)),
            imagePath(filesystem::weakly_canonical(imagePath)), pose(cv::Mat_<double>::eye(3, 4)) {
        if (this->camera == nullptr) {
            throw invalid_argument("Die Kamera darf nicht null sein.");
        }
    }

    CameraShot::~CameraShot() = default;

    cv::Mat CameraShot::loadMImage(int flags) const {
        auto resolution = getCamera()->getResolution();

        auto image = cv::imread(imagePath.string(), flags);
        if (image.size() == resolution) {
            return image;
        } else {
            cv::Mat scaled;
            cv::resize(image, scaled, resolution);
            return scaled;
        }
    }

    cv::UMat CameraShot::loadImage(int flags, cv::AccessFlag accessFlags) const {
        return loadMImage(flags).getUMat(accessFlags);
    }

    const filesystem::path &CameraShot::getImagePath() const {
        return imagePath;
    }

    void CameraShot::setFeatures(const Features &newFeatures) {
        features = newFeatures;
    }

    const Features &CameraShot::getFeatures() const {
        return features;
    }

    string CameraShot::toString() const {
        return "CameraShot(" + imagePath.string() + ", recovered: " + to_string(recovered) + ", keypoints: " +
               to_string(features.keypoints.size()) +
               ", pose: \n" + OpenCvUtils::toString(pose) + ")";
    }

    cv::Size CameraShot::getImageSize() const {
        return getCamera()->getResolution();
    }

    const cv::Mat_<double> &CameraShot::getPose() const {
        return pose;
    }

    void CameraShot::setPose(const cv::Mat_<double> &newPose) {
        if (newPose.rows != 3 || newPose.cols != 4) {
            throw invalid_argument("Die Feature Matching Strategie darf nicht null sein.");
        }

        newPose.copyTo(pose);
    }

    const shared_ptr<ICamera> &CameraShot::getCamera() const {
        return camera;
    }

    cv::Mat_<double> CameraShot::getRotationMat() const {
        return pose(cv::Rect(0, 0, 3, 3));
    }

    cv::Mat_<double> CameraShot::getRotationVec() const {
        cv::Mat_<double> rVec;
        cv::Rodrigues(getRotationMat(), rVec);
        return rVec;
    }

    cv::Mat_<double> CameraShot::getTranslationVec() const {
        return pose(cv::Rect(3, 0, 1, 3));
    }

    cv::Mat_<double> CameraShot::getCenter() const {
        return -getRotationMat().t() * getTranslationVec();
    }

    bool CameraShot::isRecovered() const {
        return recovered;
    }

    void CameraShot::setRecovered(bool newRecovered) {
        recovered = newRecovered;
    }

}