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

#ifndef PHOTOGRAMMETRIE_OPENCVUTILS_H
#define PHOTOGRAMMETRIE_OPENCVUTILS_H

#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "photogrammetrie/Photogrammetrie.h"

namespace photogrammetrie {


    /**
     * Hilfsfunktionen für OpenCV
     *
     * @author Marvin Bruns
     */
    class OpenCvUtils {
    public:
        /**
         * Wandelt eine Matrix in einen String
         *
         * @param mat Die Matrix
         * @return Der Matrix-String
         */
        static string toString(const cv::Mat &mat);

        /**
         * Wandelt eine Matrix in einen String
         *
         * @param mat Die Matrix
         * @return Der Matrix-String
         */
        static string toString(const cv::UMat &mat);

        /**
         * Wandelt einen Punkt in einen String
         *
         * @param point Der Punkt
         * @return Der Punkt-String
         */
        static string toString(const cv::Point3d &point);

        /**
         * Wandelt einen Punkt in einen String
         *
         * @param point Der Punkt
         * @return Der Punkt-String
         */
        static string toString(const cv::Point3f &point);

        /**
         * Wandelt einen Punkt in einen String
         *
         * @param point Der Punkt
         * @return Der Punkt-String
         */
        static string toString(const cv::Point2f &point);

        /**
         * Wandelt einen Punkt in einen String
         *
         * @param point Der Punkt
         * @return Der Punkt-String
         */
        static string toString(const cv::Point2d &point);

        /**
         * Wandelt einen Scalar in einen String
         *
         * @param scalar Der Scalar
         * @return Der Scalar-String
         */
        static string toString(const cv::Scalar &scalar);

        /**
         * Wandelt eine Größe in einen String
         *
         * @param size Der Größe
         * @return Der Scalar-String
         */
        static string toString(const cv::Size &size);

        /**
         * Wandelt Keypoints zu Point2f
         * @param keypoints Die Keypoints
         * @return Die passenden Point2f
         */
        static vector<cv::Point2f> toPoint2f(const vector<cv::KeyPoint> &keypoints);

        /**
         * Wandelt Keypoints zu Point2d
         * @param keypoints Die Keypoints
         * @return Die passenden Point2d
         */
        static vector<cv::Point2d> toPoint2d(const vector<cv::KeyPoint> &keypoints);

        /**
         * Prüft, ob die beiden Matches übereinstimmen
         *
         * @param match1 Der erste Match
         * @param match2 Der zweite Match
         *
         * @return true, wenn die Idx- und Distanz-Werte übereinstimmen
         */
        static bool equals(const cv::DMatch &match1, const cv::DMatch &match2);

        /**
         * Prüft, ob die beiden Matches übereinstimmen
         *
         * @param match1 Der erste Match
         * @param match2 Der zweite Match
         *
         * @return true, wenn die Idx-Werte übereinstimmen
         */
        static bool equalsIgnoreDistance(const cv::DMatch &match1, const cv::DMatch &match2);

        /**
         * Skaliert ein Bild.
         * Das Ergebnis entspricht der Zielgröße, aber das Ausgangsbild wird skaliert (nicht verzerrt).
         * Dadurch entstehen gegebenenfalls schwarze Stellen.
         *
         * @param src Die Quelle
         * @param dst Das Ziel
         * @param scale Die Skalierung
         * @param nullColor Die Farbe, die für einen ggf. ungenutzten Bereich genutzt wird
         */
        static void
        scale(cv::InputArray src, cv::OutputArray dst, const double &scale, const int &interpolation = cv::INTER_LINEAR,
              const cv::Scalar &nullColor = cv::Scalar(0, 0, 0));

        /**
         * Skaliert ein Bild.
         * Das Ergebnis entspricht der Zielgröße, aber das Ausgangsbild wird skaliert (nicht verzerrt).
         * Dadurch entstehen gegebenenfalls schwarze Stellen.
         *
         * @param src Die Quelle
         * @param dst Das Ziel
         * @param scale Die Skalierung
         * @param nullColor Die Farbe, die für einen ggf. ungenutzten Bereich genutzt wird
         */
        static void
        scale(cv::InputArray src, cv::OutputArray dst, const cv::Size &scale, const int &interpolation = cv::INTER_LINEAR,
              const cv::Scalar &nullColor = cv::Scalar(0, 0, 0));

        /**
         * Skaliert Bildpunkte
         *
         * @param src Die Quell-Punkte
         * @param dst Die Ziel-punkte
         * @param scale Die Skalierung
         */
        static void scalePoints(vector<cv::Point2d> &src, vector<cv::Point2d> &dst, const double &scale);

        /**
         * Skaliert eine Kameramatrix anhand eines Faktors
         * @tparam T Datetyp (double, float)
         * @param K Die originale Kameramatrix
         * @param scaleFactor Der Skalierungsfactor (<1 -> verkleinern, >1 -> vergrößern)
         * @return Die skalierte Kameramatrix
         */
        template<typename T>
        static cv::Mat_<T> scaledK(cv::Mat_<T> &K, double &scaleFactor);

        /**
         * Berechnet die ROI (Region of Interest)
         * @param srcSize Größe des Quellbildes
         * @param stereoMatcher Der Matcher
         * @return Die ROI
         */
        static cv::Rect computeStereoMatcherROI(cv::Size &srcSize, const cv::Ptr<cv::StereoMatcher>& stereoMatcher);
    };

    template<typename T>
    cv::Mat_<T> OpenCvUtils::scaledK(cv::Mat_<T> &K, double &scaleFactor) {
        if (K.rows != 3 || K.cols != 3) {
            throw invalid_argument("Es wird eine 3x3 Kamera-Matrix erwartet.");
        }

        cv::Mat_<T> scaledK = K;

        if (scaleFactor != 1) {
            int i;
            for (i = 0; i < scaledK.cols; i++) {
                scaledK.template at<T>(0, i) = scaledK.template at<T>(0, i) * scaleFactor;
            }
            for (i = 0; i < scaledK.cols; i++) {
                scaledK.template at<T>(1, i) = scaledK.template at<T>(1, i) * scaleFactor;
            }
        }

        return K;
    }

}


#endif //PHOTOGRAMMETRIE_OPENCVUTILS_H
