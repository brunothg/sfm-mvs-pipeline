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

#ifndef PHOTOGRAMMETRIE_SFM_H
#define PHOTOGRAMMETRIE_SFM_H

#include <memory>
#include <opencv2/features2d.hpp>
#include <vector>

#include "photogrammetrie/Photogrammetrie.h"
#include "photogrammetrie/sfm/IFeatureMatchingStrategy.h"
#include "photogrammetrie/common/Scene.h"
#include "photogrammetrie/util/AppLogger.h"

namespace photogrammetrie {

    /**
     * Diese Klasse bildet den Structure from Motion (SfM) Algorithmus ab.
     * Standardmäßig wird SIFT als Merkmalsalgorithmus genutzt.
     *
     * @author Marvin Bruns
     */
    class SfM {
    private:
        static const AppLogger logger;

        cv::Ptr<cv::FeatureDetector> featureDetector;
        cv::Ptr<cv::DescriptorExtractor> descriptorExtractor;
        cv::Ptr<cv::DescriptorMatcher> descriptorMatcher;
        shared_ptr<IFeatureMatchingStrategy> featureMatchingStrategy;
        int maxFeatureDetectionThreads = 1;
        int minMatchCount = 20;
        int minMatchCountForBaselineHomography = 100;
        double ransacReprojectionMatchingThreshold = -3.0;
        double ransacReprojectionBaselineThreshold = -1.0;
        double ransacReprojectionPoseThreshold = -8.0;
        double minHomographyInlierRatio = 0.5;
        double minPoseInlierRatio = 0.5;
        double maxReprojectionError = 10.0;
        double pointcloudPointMergeDistance = 0.01;
        double pointcloudFeatureMergeDistance = 20.0;
        bool useDistinctFeatureMatchTest = false;

    public:
        SfM();

        virtual ~SfM();

        /**
         * Ändert den Algorithmus zum Finden und Beschreiben von Bildmerkmalen
         *
         * @param algorithm Der neue Algorithmus
         */
        void setFeatureAlgorithm(const cv::Ptr<cv::Feature2D> &algorithm);

        /**
         * Ändert den Algorithmus zum Finden übereinstimmender Bildmerkmale.
         * Es muss darauf geachtet werden, dass dieser zum Feature Algorithmus passt.
         *
         * @param algorithm Der neue Algorithmus
         */
        void setMatchingAlgorithm(const cv::Ptr<cv::DescriptorMatcher> &algorithm);

        /**
         * Setzt die Strategie zum Finden von Übereinstimmungen
         * @param featureMatchingStrategy Die neue Strategie
         */
        void setFeatureMatchingStrategy(const shared_ptr<IFeatureMatchingStrategy> &featureMatchingStrategy);

        /**
         * Setzt die maximale Anzahl an Threads für die Merkmalsfindung.
         * Vor allem bei SIFT ist vorsicht geboten, da dieser Algorithmus sehr Speicherintensiv ist.
         * Ein Wert <= 1 steht für keine Parallelisierung.
         *
         * @param maxThreads Maximale Anzahl gleichzeitiger Threads für die Merkmalssuche
         */
        void setMaxFeatureDetectionThreads(int maxThreads);

        /**
         * Setzt die Mindestanzahl Übereinstimmungen, das Bildpaare als solche erkannt werden
         *
         * @param minMatchCount Mindestanzahl Übereinstimmungen
         */
        void setMinMatchCount(int minMatchCount);

        /**
         * Setzt die Mindestanzahl Übereinstimmungen, die ein Bildpaar haben muss,
         * um für die Auswahl als initiales Bildpaar für die Rekonstruktion in Frage zu kommen.
         *
         * @param minMatchCountForBaselineHomography  Mindestzahl Übereinstimmungen
         */
        void setMinMatchCountForBaselineHomography(int minMatchCountForBaselineHomography);

        /**
         * Das Minimale Verhältnis an Inliers, damit eine Rekonstruktion der Kamerapose vorgenommen wird
         *
         * @param minInlierRatio Minimales Inlier Verhältnis
         */
        void setMinHomographyInlierRatio(double minInlierRatio);

        /**
         * Das Minimale Verhältnis an Inliers, damit eine Rekonstruktion der Kamerapose vorgenommen wird
         *
         * @param minPoseInlierRatio Minimales Inlier Verhältnis
         */
        void setMinPoseInlierRatio(double minPoseInlierRatio);

        /**
         * Der maximal erlaubte Abbildungsfehler bei der Triangulation
         *
         * @param maxReprojectionError Maximal erlaubter Abbildungsfehler
         */
        void setMaxReprojectionError(double maxReprojectionError);

        /**
         * Setzt den Threshold für den RANSAC Algorithmus im Matching Schritt
         *
         * @param ransacReprojectionThreshold Wert zwischen 0 und 1
         */
        void setRansacReprojectionMatchingThreshold(double ransacReprojectionThreshold);

        /**
         * Setzt den Threshold für den RANSAC Algorithmus im Baseline Schritt
         *
         * @param ransacReprojectionBaselineThreshold Wert zwischen 0 und 1
         */
        void setRansacReprojectionBaselineThreshold(double ransacReprojectionBaselineThreshold);

        /**
         * Setzt den Threshold für den RANSAC Algorithmus in Post-Baseline Schritten
         *
         * @param ransacReprojectionPoseThreshold Wert zwischen 0 und 1
         */
        void setRansacReprojectionPoseThreshold(double ransacReprojectionPoseThreshold);

        /**
         * Maximale Distanz, damit Punkte in der Punktwolke zusammengefasst werden
         *
         * @param pointcloudPointMergeDistance Die maximale Distanz, damit Punkte in der Punktwolke zusammengefasst werden
         */
        void setPointcloudPointMergeDistance(double pointcloudPointMergeDistance);

        /**
         * Maximale Distanz von Merkmalen, damit Punkte in der Punktwolke zusammengefasst werden
         *
         * @param pointcloudFeatureMergeDistance Die maximale Distanz von Merkmalen, damit Punkte in der Punktwolke zusammengefasst werden
         */
        void setPointcloudFeatureMergeDistance(double pointcloudFeatureMergeDistance);

        /**
         * Gibt an, ob Merkmalsübereinstimmungen, die nicht eindeutig sind herausgefiltert werden sollen.
         *
         * @param useDistinctFeatureMatchTest Wenn true, wird der Test durchgeführt
         */
        void setUseDistinctFeatureMatchTest(bool useDistinctFeatureMatchTest);

        /**
         * Rekonstruiert eine Szene nach dem SfM Algorithmus
         *
         * @param scene Die Szene, die rekonstruiert werden soll (in, out)
         */
        void reconstructScene(Scene &scene);

    private:
        /**
         * Extrahiert die Merkmale von Bildaufnahmen
         *
         * @param scene Die Szene
         */
        void extractFeatures(Scene &scene) const;

        /**
         * Berechnet die Übereinstimmungen
         *
         * @param scene Die Szene
         */
        void calculateShotMatches(Scene &scene);

        /**
         * Berechnet die Homographie für alle Bildpaare
         *
         * @param scene Die Szene
         */
        void calculateHomography(Scene &scene) const;

        /**
         * Führt die Triangulation aus.
         * D.h. es werden die Kameraposen, sowie die daraus resultierenden 3D-Punkte (Punktwolke) berechnet.
         *
         * @param scene Die Szene
         */
        void triangulate(Scene &scene) const;

        /**
         * Findet zu einem Kamerapaar mit Übereinstimmungen die entsprechenden Punkte im dreidimensionalen Raum
         *
         * @param match Der Match, zu dem die 3D-Punkte gesucht werden
         * @param points Die berechneten Punktwolken-Elemente (out)
         */
        void triangulateMatch(const shared_ptr<ShotMatches> &match, vector<PointcloudElement> &points) const;

        /**
         * Versucht für ein Paar von Kameras mit Übereinstimmungen die Kameraposen zu berechnen.
         * Das Ergebnis wird direkt in dem Match gespeichert.
         *
         * @param match Der Match, zu dem die Kameraposen gesucht werden
         * @param pose Die relative Pose von der rechten Kamera zu der linken Kamera (out)
         * @param inlierMatches Die Inlier Übereinstimmungen (out)
         */
        void findCameraPoseFromMatch(const shared_ptr<ShotMatches> &match, cv::Mat_<double> &pose,
                                     vector<cv::DMatch> &inlierMatches) const;

        /**
         * Versucht für eine Kamera anhand von 3D-/2D-Punktpaaren die Kamerapose zu berechnen.
         *
         * @param matches3d2d
         * @param pose Die Pose von der Kamera (out)
         * @param inlierRatio Inlier ratio (out)
         */
        void findCameraPoseFrom3d2dMatch(const ShotMatches3d2d &matches3d2d, cv::Mat_<double> &pose,
                                         double &inlierRatio) const;
    };

}


#endif //PHOTOGRAMMETRIE_SFM_H
