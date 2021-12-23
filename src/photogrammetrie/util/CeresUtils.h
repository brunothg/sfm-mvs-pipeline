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

#ifndef PHOTOGRAMMETRIE_CERESUTILS_H
#define PHOTOGRAMMETRIE_CERESUTILS_H

#include <mutex>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/core.hpp>

#include "photogrammetrie/Photogrammetrie.h"

namespace photogrammetrie {

    /**
     * Hilfsfunktionen für Ceres
     *
     * @author Marvin Bruns
     */
    class CeresUtils {
    private:
        static once_flag initOnceFlag;
        static void initOnce();

    public:
        /**
         * Übernimmt alle Initialisierungsaufgaben (z.B. Logging)
         */
        static void init();

        /**
         * Löst ein Ceres Problem
         *
         * @param problem Das Problem
         * @param options Die Optionen
         * @param summary Die Zusammenfassung (out)
         */
        static void solve(ceres::Problem &problem, ceres::Solver::Options &options, ceres::Solver::Summary &summary);

        /**
         * Erstellt Standardeinstellungen
         *
         * @param options Die Einstellungen
         */
        static void defaultOptions(ceres::Solver::Options &options);

        /**
         * Löst ein Ceres Problem mit Standardeinstellungen
         *
         * @param problem Das Problem
         * @param summary Die Zusammenfassung (out)
         */
        static void solve(ceres::Problem &problem, ceres::Solver::Summary &summary);

        /**
         * Wandelt eine OpenCV Pose in eine Ceres Pose
         * @tparam T Typ (meist double oder float)
         * @param cvPose Die CV Pose - Matrix 3x4 (3x3Rotation, 3x1Translation)
         * @param ceresPose Die Ceres Pose - Array mit 6 Elementen (3xRotation, 3xTranslation) (out)
         */
        template<typename T>
        static void toCeresPose(const cv::Mat_<T> &cvPose, T *ceresPose);

        /**
         * Wandelt eine eine Ceres Pose in ein OpenCV Pose
         * @tparam T Typ (meist double oder float)
         * @param cvPose Die CV Pose - Matrix 3x4 (3x3Rotation, 3x1Translation) (out)
         * @param ceresPose Die Ceres Pose - Array mit 6 Elementen (3xRotation, 3xTranslation)
         */
        template<typename T>
        static void toOpenCvPose(cv::Mat_<T> &cvPose, const T *ceresPose);
    };

    template<typename T>
    void CeresUtils::toOpenCvPose(cv::Mat_<T> &cvPose, const T *ceresPose) {
        T angleAxis[3];
        for (int i = 0; i < 3; i++) {
            angleAxis[i] = ceresPose[i];
        }

        T angleMatrix[9];
        ceres::AngleAxisToRotationMatrix(angleAxis, angleMatrix);

        cvPose.template at<T>(0, 0) = angleMatrix[0];
        cvPose.template at<T>(1, 0) = angleMatrix[1];
        cvPose.template at<T>(2, 0) = angleMatrix[2];
        cvPose.template at<T>(0, 1) = angleMatrix[3];
        cvPose.template at<T>(1, 1) = angleMatrix[4];
        cvPose.template at<T>(2, 1) = angleMatrix[5];
        cvPose.template at<T>(0, 2) = angleMatrix[6];
        cvPose.template at<T>(1, 2) = angleMatrix[7];
        cvPose.template at<T>(2, 2) = angleMatrix[8];

        T translationAxis[3];
        for (int i = 0; i < 3; i++) {
            translationAxis[i] = ceresPose[i + 3];
        }
        cvPose.template at<T>(0, 3) = translationAxis[0];
        cvPose.template at<T>(1, 3) = translationAxis[1];
        cvPose.template at<T>(2, 3) = translationAxis[2];
    }

    template<typename T>
    void CeresUtils::toCeresPose(const cv::Mat_<T> &cvPose, T *ceresPose) {
        auto translation = cvPose(cv::Rect(3,0,1,3));
        auto rotation = cvPose(cv::Rect(0,0,3,3));

        T angleMatrix[9];
        angleMatrix[0] = rotation.template at<T>(0, 0);
        angleMatrix[1] = rotation.template at<T>(1, 0);
        angleMatrix[2] = rotation.template at<T>(2, 0);
        angleMatrix[3] = rotation.template at<T>(0, 1);
        angleMatrix[4] = rotation.template at<T>(1, 1);
        angleMatrix[5] = rotation.template at<T>(2, 1);
        angleMatrix[6] = rotation.template at<T>(0, 2);
        angleMatrix[7] = rotation.template at<T>(1, 2);
        angleMatrix[8] = rotation.template at<T>(2, 2);

        T angleAxis[3];
        ceres::RotationMatrixToAngleAxis<T>(angleMatrix, angleAxis);
        for (int i = 0; i < 3; i++) {
            ceresPose[i] = angleAxis[i];
        }

        T translationAxis[3];
        translationAxis[0] = translation.template at<T>(0, 0);
        translationAxis[1] = translation.template at<T>(1, 0);
        translationAxis[2] = translation.template at<T>(2, 0);
        for (int i = 0; i < 3; i++) {
            ceresPose[i + 3] = translationAxis[i];
        }
    }

}


#endif //PHOTOGRAMMETRIE_CERESUTILS_H
