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

#include "SfM.h"

#include <algorithm>
#include <memory>
#include <thread>
#include <vector>
#include <set>
#include <opencv2/calib3d.hpp>
#include "UnorderedFeatureMatchingStrategy.h"
#include "photogrammetrie/util/OpenCvUtils.h"
#include "photogrammetrie/common/BundleAdjustment.h"

namespace photogrammetrie {

    const AppLogger SfM::logger = AppLogger("SfM");

    SfM::SfM() {
        setFeatureAlgorithm(cv::SIFT::create());
        setMatchingAlgorithm(cv::BFMatcher::create());
        setFeatureMatchingStrategy(make_shared<UnorderedFeatureMatchingStrategy>());
    }

    SfM::~SfM() = default;

    void SfM::setFeatureAlgorithm(const cv::Ptr<cv::Feature2D> &algorithm) {
        if (!algorithm) {
            throw invalid_argument("Der Feature Algorithmus darf nicht null sein.");
        }

        featureDetector = algorithm;
        descriptorExtractor = algorithm;
    }

    void SfM::setMatchingAlgorithm(const cv::Ptr<cv::DescriptorMatcher> &algorithm) {
        if (!algorithm) {
            throw invalid_argument("Der Feature Matching Algorithmus darf nicht null sein.");
        }

        descriptorMatcher = algorithm;
    }

    void SfM::setFeatureMatchingStrategy(const shared_ptr<IFeatureMatchingStrategy> &newFeatureMatchingStrategy) {
        if (!newFeatureMatchingStrategy) {
            throw invalid_argument("Die Feature Matching Strategie darf nicht null sein.");
        }
        featureMatchingStrategy = newFeatureMatchingStrategy;
    }

    void SfM::setMaxFeatureDetectionThreads(int maxThreads) {
        if (maxThreads == 0) {
            maxThreads = max(1, (int) thread::hardware_concurrency());
        }
        maxFeatureDetectionThreads = max(1, maxThreads);
    }

    void SfM::setMinMatchCount(int newMinMatchCount) {
        if (newMinMatchCount < 4) {
            throw invalid_argument("Der minimale Match Count darf nicht < 4 sein.");
        }
        minMatchCount = newMinMatchCount;
    }

    void SfM::setMinMatchCountForBaselineHomography(int newMinMatchCountForBaselineHomography) {
        if (newMinMatchCountForBaselineHomography < 4) {
            throw invalid_argument("Der minimale Match Count darf nicht < 4 sein.");
        }
        minMatchCountForBaselineHomography = newMinMatchCountForBaselineHomography;
    }

    void SfM::setMinHomographyInlierRatio(double newMinInlierRatio) {
        if (newMinInlierRatio < 0 || newMinInlierRatio > 1) {
            throw invalid_argument(
                    "Das minimale Homographie-Inlier-Verhältnis darf nicht < 0 und nicht größer 1 sein.");
        }
        minHomographyInlierRatio = newMinInlierRatio;
    }

    void SfM::setMinPoseInlierRatio(double newMinPoseInlierRatio) {
        if (newMinPoseInlierRatio < 0 || newMinPoseInlierRatio > 1) {
            throw invalid_argument("Das minimale Pose-Inlier-Verhältnis darf nicht < 0 und nicht größer 1 sein.");
        }
        minPoseInlierRatio = newMinPoseInlierRatio;
    }

    void SfM::setMaxReprojectionError(double newMaxReprojectionError) {
        if (newMaxReprojectionError < 0) {
            throw invalid_argument("Der maximale Reprojektionsfehlerwert darf nicht < 0 sein.");
        }
        maxReprojectionError = newMaxReprojectionError;
    }

    void SfM::setRansacReprojectionMatchingThreshold(double newRansacReprojectionThreshold) {
        if (newRansacReprojectionThreshold < 0 || newRansacReprojectionThreshold > 1) {
            throw invalid_argument("Der RANSAC Grenzwert darf nicht < 0 und nicht größer 1 sein.");
        }
        ransacReprojectionMatchingThreshold = newRansacReprojectionThreshold;
    }

    void SfM::setRansacReprojectionBaselineThreshold(double newRansacReprojectionBaselineThreshold) {
        if (newRansacReprojectionBaselineThreshold == 0 || newRansacReprojectionBaselineThreshold > 1) {
            throw invalid_argument("Der RANSAC Grenzwert darf nicht 0 und nicht größer 1 sein.");
        }
        ransacReprojectionBaselineThreshold = newRansacReprojectionBaselineThreshold;
    }

    void SfM::setRansacReprojectionPoseThreshold(double newRansacReprojectionPoseThreshold) {
        if (newRansacReprojectionPoseThreshold == 0 || newRansacReprojectionPoseThreshold > 1) {
            throw invalid_argument("Der RANSAC Grenzwert darf nicht 0 und nicht größer 1 sein.");
        }
        ransacReprojectionPoseThreshold = newRansacReprojectionPoseThreshold;
    }

    void SfM::setPointcloudPointMergeDistance(double newPointcloudPointMergeDistance) {
        pointcloudPointMergeDistance = newPointcloudPointMergeDistance;
    }

    void SfM::setPointcloudFeatureMergeDistance(double newPointcloudFeatureMergeDistance) {
        pointcloudFeatureMergeDistance = newPointcloudFeatureMergeDistance;
    }

    void SfM::setUseDistinctFeatureMatchTest(bool newUseDistinctFeatureMatchTest) {
        useDistinctFeatureMatchTest = newUseDistinctFeatureMatchTest;
    }

    void SfM::reconstructScene(Scene &scene) {
        SfM::logger.info("Beginne Rekonstruktion der Szene");

        // Vorbedingungen prüfen
        if (scene.getShots().size() < 2) {
            throw invalid_argument("Die Szene muss mindestens zwei Aufnahmen enthalten.");
        }

        // Merkmale extrahieren
        extractFeatures(scene);

        // Merkmalsübereinstimmungen finden
        calculateShotMatches(scene);

        // Hompgraphie berechnen
        calculateHomography(scene);

        // Szene rekonstruieren (Triangulation)
        triangulate(scene);
    }

    void SfM::triangulate(Scene &scene) const {
        SfM::logger.info("Rekonstruiere Szene");
        auto shotMatchesToTriangulate = scene.getShotMatches();
        vector<shared_ptr<ShotMatches>> triangulatedShotMatches;
        vector<shared_ptr<ShotMatches>> failedShotMatches;

        // Guard - Grundannahme: mindestens ein Bildpaar mit Übereinstimmungen
        if (shotMatchesToTriangulate.empty()) {
            return;
        }

        // Initiales Bildpaar anhand der Homographien ermitteln
        {
            vector<shared_ptr<ShotMatches>> homographyOrderedShotMatches(shotMatchesToTriangulate);
            homographyOrderedShotMatches.erase(
                    remove_if(homographyOrderedShotMatches.begin(), homographyOrderedShotMatches.end(),
                              [this](const shared_ptr<ShotMatches> &match) {
                                  return match->getHomographyInlierRatio() < minHomographyInlierRatio;
                              }), homographyOrderedShotMatches.end()
            );
            sort(homographyOrderedShotMatches.begin(), homographyOrderedShotMatches.end(),
                 [this](shared_ptr<ShotMatches> &a, shared_ptr<ShotMatches> &b) -> bool {
                     return a->getMatches().size() >= minMatchCountForBaselineHomography &&
                            a->getHomographyInlierRatio() < b->getHomographyInlierRatio();
                 });
            SfM::logger.debug(
                    "Bildpaare nach Homographien sortiert: " + to_string(
                            homographyOrderedShotMatches[0]->getHomographyInlierRatio()) +
                    " ... " +
                    to_string(homographyOrderedShotMatches[homographyOrderedShotMatches.size() -
                                                           1]->getHomographyInlierRatio()));

            for (auto &match : homographyOrderedShotMatches) {
                SfM::logger.info("Versuche initiale Triangulation für: " + match->toString());

                cv::Mat_<double> relativePose;
                vector<cv::DMatch> inlierMatches;
                try {
                    findCameraPoseFromMatch(match, relativePose, inlierMatches);
                } catch (cv::Exception &e) {
                    SfM::logger.warn("Pose konnte nicht ermittelt werden: " + string(e.what()));
                    continue;
                }
                double poseInliersRatio = (double) inlierMatches.size() / (double) match->getMatches().size();

                if (poseInliersRatio < minPoseInlierRatio) {
                    SfM::logger.info("Pose Inlier Verhältnis zu gering: " + to_string(poseInliersRatio));
                    continue;
                }
                match->setMatches(inlierMatches);
                match->getLeft()->setPose(cv::Mat_<double>::eye(3, 4));
                match->getRight()->setPose(relativePose);

                // Baue Punktwolke
                {
                    vector<PointcloudElement> pointCloudElements;
                    triangulateMatch(match, pointCloudElements);
                    for (auto &pce : pointCloudElements) {
                        scene.addPointcloudElement(make_shared<PointcloudElement>(pce));
                    }
                }

                match->getLeft()->setRecovered();
                match->getRight()->setRecovered();
                shotMatchesToTriangulate.erase(
                        remove(shotMatchesToTriangulate.begin(), shotMatchesToTriangulate.end(), match),
                        shotMatchesToTriangulate.end());
                triangulatedShotMatches.push_back(match);

                break; // Initiales Paar gefunden
            }
            BundleAdjustment::doBundleAdjustment(scene);
        }

        // Initiale Triangulation nicht erfolgreich
        if (triangulatedShotMatches.empty()) {
            SfM::logger.warn("Es konnte kein initiales Paar trianguliert werden");
            return;
        }

        // Weitere Aufnahmen hinzufügen
        {
            auto shotMatchesToTriangulateTotalCount = shotMatchesToTriangulate.size();
            while (!shotMatchesToTriangulate.empty()) {
                /*
                 * Suche die Aufnahme mit den meisten Übereinstimmungen zu bereits gefundenen 3D-Punkten.
                 * Gesucht ist die Aufnahme mit den meisten 3D-2D Übereinstimmungen.
                 *
                 * Anhand der 3D-2D Übereinstimmungen wird die Pose ermittelt.
                 *
                 * Hierzu werden dann alle noch offenen Matches trianguliert und in die Punktwolke integriert.
                 *
                 * Im Anschluss wird eine Bündelblockausgleichung vorgenommen.
                 * */
                SfM::logger.debug("Suche noch nicht rekonstruierte Kameras");
                vector<shared_ptr<CameraShot>> remainingShots;
                {
                    set<shared_ptr<CameraShot>> _remainingShots;
                    for (auto &shotMatchToTriangulate : shotMatchesToTriangulate) {
                        if (!shotMatchToTriangulate->getLeft()->isRecovered()) {
                            _remainingShots.insert(shotMatchToTriangulate->getLeft());
                        }
                        if (!shotMatchToTriangulate->getRight()->isRecovered()) {
                            _remainingShots.insert(shotMatchToTriangulate->getRight());
                        }
                    }
                    remainingShots = vector<shared_ptr<CameraShot>>(_remainingShots.begin(), _remainingShots.end());
                }
                if (remainingShots.empty()) {
                    break;
                }

                vector<ShotMatches3d2d> matches2d3d;
                SfM::logger.info("Suche 3d2d Übereinstimmungen");
                for (auto &remainingShot : remainingShots) {
                    ShotMatches3d2d match3d2d(remainingShot);
                    scene.find3d2dMatches(match3d2d);
                    matches2d3d.push_back(match3d2d);
                }
                if (!matches2d3d.empty()) {
                    sort(matches2d3d.begin(), matches2d3d.end(),
                         [](ShotMatches3d2d &a, ShotMatches3d2d &b) -> bool {
                             return a.getMatches().size() > b.getMatches().size();
                         });
                    SfM::logger.debug(
                            "Bildaufnahmen nach 3D->2D Übereinstimmungen sortiert (" + to_string(matches2d3d.size()) +
                            "): " + to_string(
                                    matches2d3d[0].getMatches().size()) +
                            " ... " +
                            to_string(matches2d3d[matches2d3d.size() -
                                                  1].getMatches().size()));
                } else {
                    SfM::logger.debug("Bildaufnahmen nach 3D->2D Übereinstimmungen (0)");
                    break;
                }

                const auto &best3d2dMatch = matches2d3d[0];
                SfM::logger.info("Versuche Triangulation für: " + best3d2dMatch.getShot()->toString());
                cv::Mat_<double> pose;
                double poseInliersRatio;
                try {
                    findCameraPoseFrom3d2dMatch(best3d2dMatch, pose, poseInliersRatio);
                } catch (cv::Exception &e) {
                    SfM::logger.warn("Pose konnte nicht ermittelt werden: " + string(e.what()));
                    poseInliersRatio = -1;
                }

                if (poseInliersRatio < minPoseInlierRatio) {
                    vector<shared_ptr<ShotMatches>> failedMatches;
                    for (auto &match : shotMatchesToTriangulate) {
                        if (match->getLeft() == best3d2dMatch.getShot() ||
                            match->getRight() == best3d2dMatch.getShot()) {
                            failedMatches.push_back(match);
                        }
                    }

                    for (auto &failedMatch : failedMatches) {
                        shotMatchesToTriangulate.erase(
                                remove(shotMatchesToTriangulate.begin(), shotMatchesToTriangulate.end(), failedMatch),
                                shotMatchesToTriangulate.end());
                        failedShotMatches.push_back(failedMatch);
                    }
                    SfM::logger.info("Pose Inlier Verhältnis zu gering: " + to_string(poseInliersRatio));
                    continue;
                }
                best3d2dMatch.getShot()->setPose(pose);
                best3d2dMatch.getShot()->setRecovered();

                vector<shared_ptr<ShotMatches>> shotMatches;
                best3d2dMatch.getDistinctShotMatches(shotMatches);
                for (auto &shotMatch : shotMatches) {
                    if (!shotMatch->getLeft()->isRecovered() || !shotMatch->getRight()->isRecovered()) {
                        continue;
                    }

                    // Inlier Matches ermitteln
                    SfM::logger.debug("Inlier Matches ermitteln");
                    {
                        vector<cv::DMatch> inlierMatches;
                        cv::Mat_<double> matchPose;
                        try {
                            findCameraPoseFromMatch(shotMatch, matchPose, inlierMatches);
                        } catch (cv::Exception &e) {
                            SfM::logger.warn("Pose konnte nicht ermittelt werden: " + string(e.what()));
                            continue;
                        }
                        shotMatch->setMatches(inlierMatches);
                    }

                    // Punktwolke erweitern
                    {
                        vector<PointcloudElement> pointCloudElements;
                        triangulateMatch(shotMatch, pointCloudElements);
                        SfM::logger.debug("Punktwolke erweitern");
                        for (auto &pce : pointCloudElements) {
                            scene.mergePointcloudElement3d2d(make_shared<PointcloudElement>(pce),
                                                             pointcloudPointMergeDistance,
                                                             pointcloudFeatureMergeDistance);
                        }
                    }

                    shotMatchesToTriangulate.erase(
                            remove(shotMatchesToTriangulate.begin(), shotMatchesToTriangulate.end(), shotMatch),
                            shotMatchesToTriangulate.end());
                    triangulatedShotMatches.push_back(shotMatch);
                }

                BundleAdjustment::doBundleAdjustment(scene);
                SfM::logger.info("Verlbeibende Bilderpaare: " + to_string(shotMatchesToTriangulate.size()) + "(" +
                                 to_string((int)(100.0 * (1.0 - ((double) shotMatchesToTriangulate.size() /
                                                        (double) shotMatchesToTriangulateTotalCount)))) + "%)");
            }
        }

        SfM::logger.info("Rekonstruktion der Szene wurde abgeschlossen: \n\tRekonstruierte Bildpaare: " +
                         to_string(triangulatedShotMatches.size()) + " \n\tVerbleibende Bildpaare: " +
                         to_string(failedShotMatches.size()));
    }

    void SfM::triangulateMatch(const shared_ptr<ShotMatches> &match, vector<PointcloudElement> &points) const {
        auto &matches = match->getMatches();
        if (matches.empty()) {
            return;
        }

        Features alignedFeaturesLeft;
        Features alignedFeaturesRight;
        match->alignFeatures(alignedFeaturesLeft, alignedFeaturesRight);

        auto &leftShot = match->getLeft();
        auto &rightShot = match->getRight();

        auto &leftCamera = match->getLeft()->getCamera();
        auto &rightCamera = match->getRight()->getCamera();

        cv::Mat_<double> leftK, rightK;
        leftCamera->getK(leftK);
        rightCamera->getK(rightK);

        cv::Mat_<double> leftDistortion, rightDistortion;
        leftCamera->getDistortion(leftDistortion);
        rightCamera->getDistortion(rightDistortion);

        auto leftPoints = OpenCvUtils::toPoint2f(alignedFeaturesLeft.keypoints);
        auto rightPoints = OpenCvUtils::toPoint2f(alignedFeaturesRight.keypoints);

        cv::Mat leftNormalizedPoints, rightNormalizedPoints;
        cv::undistortPoints(leftPoints, leftNormalizedPoints, leftK, leftDistortion);
        cv::undistortPoints(rightPoints, rightNormalizedPoints, rightK, rightDistortion);

        cv::Mat points4d; // 3D-Koordinaten (homogen)
        cv::triangulatePoints(leftShot->getPose(), rightShot->getPose(), leftNormalizedPoints, rightNormalizedPoints,
                              points4d);

        cv::Mat points3d; // 3D-Koordinaten (euklidisch)
        cv::convertPointsFromHomogeneous(points4d.t(), points3d);
        auto pointCount = points3d.rows;

        vector<cv::Point2f> leftReprojectedPoints, rightReprojectedPoints;
        projectPoints(points3d, leftShot->getRotationVec(), leftShot->getTranslationVec(), leftK, leftDistortion,
                      leftReprojectedPoints);
        projectPoints(points3d, rightShot->getRotationVec(), rightShot->getTranslationVec(), rightK, rightDistortion,
                      rightReprojectedPoints);

        // Filter Punkte mit einem zu großen Projektionsfehler und bilde Punktwolke
        for (size_t i = 0; i < pointCount; i++) {
            auto leftReprojectionError = cv::norm(
                    leftReprojectedPoints[i] - leftPoints[i]);
            auto rightReprojectionError = cv::norm(
                    rightReprojectedPoints[i] - rightPoints[i]);
            if (leftReprojectionError > maxReprojectionError || rightReprojectionError > maxReprojectionError) {
                continue;
            }

            PointcloudElement pcElement(cv::Point3d(points3d.at<float>(i, 0), points3d.at<float>(i, 1),
                                                    points3d.at<float>(i, 2)));
            cv::Point2d matchesLeftPoint(match->getLeft()->getFeatures().keypoints[matches[i].queryIdx].pt);
            cv::Point2d matchesRightPoint(match->getRight()->getFeatures().keypoints[matches[i].trainIdx].pt);
            auto pceOrigin = make_shared<PointcloudElement::Origin>(match, pair(matchesLeftPoint, matchesRightPoint));
            pcElement.addOrigin(pceOrigin);

            points.push_back(pcElement);
        }

        SfM::logger.debug(
                "Triangulation von " + match->toString() + ": \nPunkte " + to_string(points.size()) + " von " +
                to_string(pointCount));
    }

    void SfM::findCameraPoseFrom3d2dMatch(const ShotMatches3d2d &matches3d2d, cv::Mat_<double> &pose,
                                          double &inlierRatio) const {
        inlierRatio = 0;

        vector<cv::Point3d> points3d;
        vector<cv::Point2d> points2d;
        matches3d2d.getDistinct3d2dPoints(points3d, points2d);

        auto &camera = matches3d2d.getShot()->getCamera();

        cv::Mat_<double> K;
        camera->getK(K);

        cv::Mat_<double> distortion;
        camera->getDistortion(distortion);

        auto xSize = matches3d2d.getShot()->getImageSize();

        double ransacThreshold = (ransacReprojectionPoseThreshold < 0)
                                 ? ransacReprojectionPoseThreshold * -1
                                 : max({xSize.width, xSize.height}) * ransacReprojectionPoseThreshold;

        cv::Mat rvec, tvec;
        cv::Mat inliers;
        cv::solvePnPRansac(points3d, points2d, K, distortion, rvec, tvec, false, 100, ransacThreshold, 0.99, inliers);

        inlierRatio = (double) cv::countNonZero(inliers) / (double) points2d.size();

        cv::Mat R, t;
        cv::Rodrigues(rvec, R);
        t = tvec;

        cv::hconcat(R, t, pose);
        SfM::logger.debug(
                "Pose: (" + to_string(cv::countNonZero(inliers)) + " von " + to_string(points2d.size()) + ")\n" +
                OpenCvUtils::toString(pose));
    }

    void SfM::findCameraPoseFromMatch(const shared_ptr<ShotMatches> &match, cv::Mat_<double> &pose,
                                      vector<cv::DMatch> &inlierMatches) const {
        auto &matches = match->getMatches();

        Features alignedFeaturesLeft;
        Features alignedFeaturesRight;
        match->alignFeatures(alignedFeaturesLeft, alignedFeaturesRight);

        auto &leftCamera = match->getLeft()->getCamera();
        auto &rightCamera = match->getRight()->getCamera();

        auto leftSize = match->getLeft()->getImageSize();
        auto rightSize = match->getRight()->getImageSize();

        cv::Mat_<double> leftK, rightK;
        leftCamera->getK(leftK);
        rightCamera->getK(rightK);

        cv::Mat_<double> leftDistortion, rightDistortion;
        leftCamera->getDistortion(leftDistortion);
        rightCamera->getDistortion(rightDistortion);

        auto leftPoints = OpenCvUtils::toPoint2f(alignedFeaturesLeft.keypoints);
        auto rightPoints = OpenCvUtils::toPoint2f(alignedFeaturesRight.keypoints);


        double ransacThreshold = (ransacReprojectionBaselineThreshold < 0)
                                 ? ransacReprojectionBaselineThreshold * -1
                                 : max({leftSize.width, leftSize.height, rightSize.height, rightSize.height}) *
                                   ransacReprojectionBaselineThreshold;

        cv::Mat mask;
        cv::Mat R, t;
        cv::Mat E = cv::findEssentialMat(leftPoints,
                                         rightPoints, leftK, leftDistortion,
                                         rightK, rightDistortion, cv::RANSAC, 0.999, ransacThreshold, mask);
        cv::recoverPose(E, leftPoints, rightPoints, leftK, R, t, mask);

        cv::hconcat(R, t, pose);
        SfM::logger.debug(
                "Pose: (" + to_string(cv::countNonZero(mask)) + " von " + to_string(match->getMatches().size()) +
                ")\n" +
                OpenCvUtils::toString(pose));

        for (size_t i = 0; i < mask.rows; i++) {
            if (mask.at<uchar>(i)) {
                inlierMatches.push_back(matches[i]);
            }
        }
    }

    void SfM::calculateShotMatches(Scene &scene) {
        SfM::logger.info("Berechne Merkmalsübereinstimmungen");
        vector<ShotMatches> shotMatches;
        featureMatchingStrategy->calculateShotMatches(scene, descriptorMatcher, shotMatches);

        if (useDistinctFeatureMatchTest) {
            for (auto &shotMatch : shotMatches) {
                auto matches = shotMatch.getMatches();

                auto mCountBefore = matches.size();
                matches.erase(remove_if(matches.begin(), matches.end(), [matches](const cv::DMatch &i) {
                    auto mCount = count_if(matches.begin(), matches.end(), [i](const cv::DMatch &j) {
                        return i.trainIdx == j.trainIdx && i.queryIdx != j.queryIdx;
                    });
                    return mCount > 0;
                }), matches.end());
                auto mCountAfter = matches.size();
                SfM::logger.debug("Merkmalsübereinstimmungen gefilter: " + to_string(mCountAfter) + " von " +
                                  to_string(mCountBefore) + " für: \n" + shotMatch.toString());

                shotMatch.setMatches(matches);
            }
        }

        shotMatches.erase(
                remove_if(shotMatches.begin(), shotMatches.end(), [this](const ShotMatches &match) {
                    return match.getMatches().size() < minMatchCount;
                }), shotMatches.end()
        );
        for (auto &shotMatch : shotMatches) {
            scene.addShotMatches(make_shared<ShotMatches>(shotMatch));
        }
        SfM::logger.info("Es wurden Übereinstimmungen in Bildern gefunden: " + to_string(shotMatches.size()));
    }

    void SfM::extractFeatures(Scene &scene) const {
        SfM::logger.info("Merkmale werden extrahiert");
        auto &shots = scene.getShots();

        size_t max = shots.size(), completed = 0;
#pragma omp parallel for default(none) shared(shots, max, completed) num_threads(maxFeatureDetectionThreads)
        for (auto &shot : shots) {
            auto image = shot->loadImage();

            Features features;
            featureDetector->detect(image, features.keypoints);
            descriptorExtractor->compute(image, features.keypoints, features.descriptors);

            shot->setFeatures(features);
            SfM::logger.debug("Berechnete Merkmale für: " + shot->toString());

            completed++;
            SfM::logger.info("Merkmale berechnet: " + to_string(completed) + " von " + to_string(max) + " (" +
                             to_string((int) ((100.0 / max) * completed)) + "%)");
        }
    }

    void SfM::calculateHomography(Scene &scene) const {
        SfM::logger.info("Berechne Homographien");
        auto &shotMatches = scene.getShotMatches();

#pragma omp parallel for default(none) shared(shotMatches, ransacReprojectionMatchingThreshold)
        for (auto &shotMatch : shotMatches) {
            auto &matches = shotMatch->getMatches();
            if (matches.size() < 4) {
                // Homographie kann nicht gefunden werden
                continue;
            }

            // Matches sortieren/ausrichten
            Features alignedFeaturesLeft, alignedFeaturesRight;
            shotMatch->alignFeatures(alignedFeaturesLeft, alignedFeaturesRight);

            auto leftSize = shotMatch->getLeft()->getImageSize();
            auto rightSize = shotMatch->getRight()->getImageSize();
            double ransacThreshold = (ransacReprojectionMatchingThreshold < 0)
                                     ? ransacReprojectionMatchingThreshold * -1
                                     : max({leftSize.width, leftSize.height, rightSize.height, rightSize.height}) *
                                       ransacReprojectionMatchingThreshold;

            cv::Mat inlierMask;
            cv::Mat homography = cv::findHomography(OpenCvUtils::toPoint2f(alignedFeaturesLeft.keypoints),
                                                    OpenCvUtils::toPoint2f(alignedFeaturesRight.keypoints),
                                                    cv::RANSAC, ransacThreshold, inlierMask);
            const auto inlierCount = (homography.empty()) ? 0 : cv::countNonZero(inlierMask);
            const double inlierRatio = (double) inlierCount / (double) matches.size();

            shotMatch->setHomographyInlierRatio(inlierRatio);

            SfM::logger.debug(
                    "Homographie Anzahl Inliners für " + shotMatch->toString() +
                    ": " + to_string(inlierCount) + " \nRatio: " + to_string(inlierRatio) + "\nThreshold: " +
                    to_string(ransacThreshold));

        }
    }

}