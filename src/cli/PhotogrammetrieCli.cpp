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

#include "PhotogrammetrieCli.h"

#include <filesystem>
#include <vector>
#include <memory>
#include <algorithm>
#include <opencv2/features2d.hpp>

#include "util/AppStatistics.h"
#include "photogrammetrie/common/Scene.h"
#include "photogrammetrie/common/CameraShot.h"
#include "photogrammetrie/sfm/SfM.h"
#include "photogrammetrie/sfm/UnorderedFeatureMatchingStrategy.h"
#include "photogrammetrie/sfm/VideoFeatureMatchingStrategy.h"
#include "photogrammetrie/sfm/GridFeatureMatchingStrategy.h"
#include "photogrammetrie/mvs/MVS.h"
#include "photogrammetrie/common/DistortionCamera.h"
#include "photogrammetrie/common/SimpleRadialCamera.h"
#include "photogrammetrie/common/SimpleCamera.h"
#include "photogrammetrie/util/PclUtils.h"
#include "photogrammetrie/util/SceneUtils.h"

namespace photogrammetrie {
    const AppLogger PhotogrammetrieCli::logger = AppLogger("PhotogrammetrieCli");
    AppArgs PhotogrammetrieCli::args = AppArgs();

    void PhotogrammetrieCli::main(const int argc, const char **argv) {
        // Initialisiere Anwendung
        {
            string execName = argv[0];

            init(argc, argv, execName);
            checkImageParam(execName);
        }

        // Arbeitsverzeichnis vorbereiten
        filesystem::path workdir = getWorkingDir();
        prepareWorkingDir(workdir);

        // Statistiken mitschreiben
        AppStatistics appStatistics(workdir / "app.stat.csv");
        if (PhotogrammetrieCli::args.isFlag("stats")) {
            appStatistics.start();
        }

        // SfM
        {
            Scene scene = prepareScene();
            runSfM(scene);

            OpenMvsUtils::toOpenMVS(scene, workdir / "omvs", "mvs.bin");
        }

        // MVS
        {
            if (PhotogrammetrieCli::args.isFlag("dense")) {
                runMVS(workdir / "omvs", "mvs.bin");
            }
        }

        appStatistics.stop();

    }

    void PhotogrammetrieCli::runSfM(Scene &scene) {
        auto workdir = getWorkingDir();

        cv::Ptr<cv::Feature2D> featureDetector = configureFeatureDetector();
        cv::Ptr<cv::DescriptorMatcher> featureMatcher = configureFeatureMatcher();
        shared_ptr<IFeatureMatchingStrategy> matchingStrategy = configureFeatureMatcherStrategy();

        SfM sfm;
        sfm.setFeatureAlgorithm(featureDetector);
        sfm.setMatchingAlgorithm(featureMatcher);
        sfm.setFeatureMatchingStrategy(matchingStrategy);
        sfm.setMaxFeatureDetectionThreads(stoi(PhotogrammetrieCli::args.getArg("omp-feature-threads", "0")));
        sfm.setMinMatchCount(stoi(PhotogrammetrieCli::args.getArg("match-threshold", "20")));
        sfm.setMinMatchCountForBaselineHomography(
                stoi(PhotogrammetrieCli::args.getArg("baseline-homography-threshold", "100")));
        sfm.setRansacReprojectionMatchingThreshold(
                stod(PhotogrammetrieCli::args.getArg("ransac-matching-threshold", "0.006")));
        sfm.setRansacReprojectionBaselineThreshold(
                stod(PhotogrammetrieCli::args.getArg("ransac-baseline-threshold", "-1")));
        sfm.setRansacReprojectionPoseThreshold(
                stod(PhotogrammetrieCli::args.getArg("ransac-pose-threshold", "-8.0")));
        sfm.setMinHomographyInlierRatio(
                stod(PhotogrammetrieCli::args.getArg("homography-inlier-ratio-threshold", "0.4")));
        sfm.setMinPoseInlierRatio(stod(PhotogrammetrieCli::args.getArg("pose-inlier-ratio-threshold", "0.4")));
        sfm.setMaxReprojectionError(stod(PhotogrammetrieCli::args.getArg("reprojection-error-threshold", "10")));
        sfm.setPointcloudFeatureMergeDistance(
                stod(PhotogrammetrieCli::args.getArg("pointcloud-feature-merge-distance", "20")));
        sfm.setPointcloudPointMergeDistance(
                stod(PhotogrammetrieCli::args.getArg("pointcloud-point-merge-distance", "0.01")));
        sfm.setUseDistinctFeatureMatchTest(PhotogrammetrieCli::args.isFlag("distinct-matches"));
        sfm.reconstructScene(scene);

        PhotogrammetrieCli::logger.info("Färbe Punktwolke ein");
        scene.colorizePointcloud();

        PhotogrammetrieCli::logger.info("Schreibe Kameras (recvoered 3D)");
        PclUtils::writeToPLY(workdir / "cameras_recovered.ply", scene.getShots());

        PhotogrammetrieCli::logger.info("Schreibe Punktwolke (sparse 3D)");
        PclUtils::writeToPLY(workdir / "pointcloud_sparse.ply", scene.getPointcloud(),
                             (PhotogrammetrieCli::args.isFlag("colored")) ? cv::Scalar(-1) : cv::Scalar(0, 0, 0, 255));


        if (PhotogrammetrieCli::args.isFlag("stats")) {
            PhotogrammetrieCli::logger.info("Erstelle Statistiken");

            // Schreibe Statistik für Abbildungsfehler
            SceneUtils::writeToReprojectionErrorStatisticsCSV(scene, workdir / "reprojectionerror.stat.csv");
            SceneUtils::writeToReprojectionErrorHistogramCSV(scene, workdir / "reprojectionerror.hist.csv");
        }

        if (PhotogrammetrieCli::args.isFlag("artifacts")) {
            writeArtifacts(scene);
        }
    }

    void PhotogrammetrieCli::writeArtifacts(const Scene &scene) {
        auto workdir = getWorkingDir();

        // Schreibe Artefakte
        PhotogrammetrieCli::logger.info("Erstelle Artefakte");
        auto shots = scene.getShots();
        auto shotMatches = scene.getShotMatches();
        unsigned int index;

        // Schreibe Szene
        ofstream sceneFile;
        sceneFile.open(workdir / "scene.txt");
        sceneFile << scene.toString();
        sceneFile.close();

        // Schreibe Merkmale
        auto featuresDir = workdir / "features";
        filesystem::create_directories(featuresDir);
        index = 0;
#pragma omp parallel for default(none) shared(shots, featuresDir, index)
        for (auto &shot : shots) {
            unsigned int i;
#pragma omp critical
            {
                i = index;
                index++;
            }

            auto outpath = featuresDir / (to_string(i) + shot->getImagePath().filename().string() + ".jpg");
            cv::UMat keypointImage;
            cv::drawKeypoints(shot->loadImage(cv::IMREAD_COLOR), shot->getFeatures().keypoints, keypointImage,
                              cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            cv::imwrite(outpath.string(), keypointImage, vector<int>{cv::ImwriteFlags::IMWRITE_JPEG_QUALITY, 80});
        }

        // Schreibe Übereinstimmungen
        auto matchesDir = workdir / "matches";
        filesystem::create_directories(matchesDir);
        index = 0;
#pragma omp parallel for default(none) shared(shotMatches, matchesDir, index)
        for (auto &matches : shotMatches) {
            unsigned int i;
#pragma omp critical
            {
                i = index;
                index++;
            }

            auto outpath = matchesDir /
                           (to_string(i) + matches->getLeft()->getImagePath().filename().string() + "-" +
                            matches->getRight()->getImagePath().filename().string() + ".jpg");

            cv::UMat matchImage;
            cv::drawMatches(matches->getLeft()->loadImage(cv::IMREAD_COLOR),
                            matches->getLeft()->getFeatures().keypoints,
                            matches->getRight()->loadImage(cv::IMREAD_COLOR),
                            matches->getRight()->getFeatures().keypoints, matches->getMatches(), matchImage,
                            cv::Scalar::all(-1),
                            cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
            cv::imwrite(outpath.string(), matchImage, vector<int>{cv::ImwriteFlags::IMWRITE_JPEG_QUALITY, 80});
        }

        // Schreibe unverzerrte Bilder
        auto undistortedDir = workdir / "undistorted";
        filesystem::create_directories(undistortedDir);
        index = 0;
#pragma omp parallel for default(none) shared(shots, undistortedDir, index)
        for (auto &shot : shots) {
            unsigned int i;
#pragma omp critical
            {
                i = index;
                index++;
            }

            auto outpath = undistortedDir / (to_string(i) + shot->getImagePath().filename().string() + ".jpg");
            cv::UMat undistortedImage;
            shot->getCamera()->undistort(shot->loadImage(cv::IMREAD_COLOR), undistortedImage);
            cv::imwrite(outpath.string(), undistortedImage, vector<int>{cv::ImwriteFlags::IMWRITE_JPEG_QUALITY, 80});
        }
    }

    void PhotogrammetrieCli::runMVS(const std::filesystem::path &mvsWorkdir, const string &mvsFilename) {
        auto workdir = getWorkingDir();
        bool reconstructMesh = PhotogrammetrieCli::args.isFlag("mesh");

        MVS mvs(mvsWorkdir, mvsFilename);
        mvs.setArtifacts(PhotogrammetrieCli::args.isFlag("artifacts") && AppLogger::loglevel == AppLogger::LOG_TRACE);
        mvs.setRefineMesh(PhotogrammetrieCli::args.isFlag("refine-mesh"));
        mvs.setTextureMesh(PhotogrammetrieCli::args.isFlag("colored"));
        mvs.setMatchingStrategy((PhotogrammetrieCli::args.isFlag("sgm")) ? MVS::MATCHING_SGM : MVS::MATCHING_DEFAULT);
        mvs.setDecimateMesh(!PhotogrammetrieCli::args.isFlag("no-decimate"));

        mvs.densifyScene();

        PhotogrammetrieCli::logger.info("Schreibe Punktwolke (dense 3D)");
        OpenMvsUtils::writeToPLY(workdir / "pointcloud_dense.ply", mvs.getScene().pointcloud);

        if (reconstructMesh) {
            mvs.meshScene();

            PhotogrammetrieCli::logger.info("Schreibe Mesh (dense)");
            OpenMvsUtils::writeToPLY(workdir / "mesh_dense.ply", mvs.getScene().mesh);
        }

        if (!PhotogrammetrieCli::args.isFlag("artifacts")) {
            std::filesystem::remove_all(mvsWorkdir);
        }
    }

    Scene PhotogrammetrieCli::prepareScene() {
        logger.info("Baue Szene");

        vector<filesystem::path> imagePaths;
        for (const string &image : args.getArgs("image")) {
            const filesystem::path imagePath(image);
            if (!filesystem::exists(imagePath)) {
                logger.warn(
                        "Das Bild oder der Bildordner '" + imagePath.string() + "' konnte nicht gefunden werden");
                continue;
            }

            if (filesystem::is_directory(imagePath)) {
                vector<filesystem::path> subImagePaths;
                for (const filesystem::path &subImagePath : filesystem::directory_iterator(imagePath)) {
                    if (!filesystem::is_directory(subImagePath)) {
                        subImagePaths.push_back(subImagePath);
                    }
                }
                sort(subImagePaths.begin(), subImagePaths.end(),
                     [](const filesystem::path &p1, const filesystem::path &p2) -> int {
                         string s1 = p1.filename().string();
                         string s2 = p2.filename().string();
                         return s1 < s2;
                     });
                for (const filesystem::path &subImagePath : subImagePaths) {
                    imagePaths.push_back(subImagePath);
                }
            } else {
                imagePaths.push_back(imagePath);
            }
        }
        PhotogrammetrieCli::logger.debug(to_string(imagePaths.size()) + " Bilder gefunden");
        if (imagePaths.size() < 2) {
            PhotogrammetrieCli::logger.error("Es müssen mindestens zwei Bilder angegeben werden");
            exit(-1);
        }

        Scene scene;
        shared_ptr<ICamera> camera; // CLI unterstützt momentan nur ein globales Standardkameramodell
        string cameraModelName = PhotogrammetrieCli::args.getArg("camera-model", "SimpleRadial");
        if (cameraModelName == "SimpleRadial") {
            camera = make_shared<SimpleRadialCamera>();
        } else if (cameraModelName == "Distortion") {
            camera = make_shared<DistortionCamera>();
        } else if (cameraModelName == "Simple") {
            camera = make_shared<SimpleCamera>();
        } else {
            PhotogrammetrieCli::logger.warn("Unbekanntes Kameramodell: " + cameraModelName + " -> nutze SimpleRadial");
            camera = make_shared<SimpleRadialCamera>();
        }

        bool firstCamera = true;
        cv::Size resolution(0, 0);
        for (const filesystem::path &imagePath : imagePaths) {
            auto cameraShot = make_shared<CameraShot>(camera, imagePath);
            scene.addCameraShot(cameraShot);

            if (firstCamera) {
                firstCamera = false;
                resolution = cv::imread(imagePath).size();
            }
        }
        camera->setFocalLength((double) max(resolution.height, resolution.width) * (double) 1.2);
        camera->setCenter((double) resolution.width / (double) 2.0, (double) resolution.height / (double) 2.0);
        camera->setResolution(resolution);
        PhotogrammetrieCli::logger.info("Anfangszustand für die Kamera: \n" + camera->toString());

        return scene;
    }

    shared_ptr<IFeatureMatchingStrategy> PhotogrammetrieCli::configureFeatureMatcherStrategy() {
        int featureSequence = stoi(PhotogrammetrieCli::args.getArg("feature-sequence", "0"));
        int featureGridLength = stoi(PhotogrammetrieCli::args.getArg("feature-gridlength", "0"));
        shared_ptr<IFeatureMatchingStrategy> matchingStrategy;
        if (featureSequence >= 2) {
            if (featureGridLength >= 1) {
                matchingStrategy = make_shared<GridFeatureMatchingStrategy>(featureSequence, featureGridLength);
            } else {
                matchingStrategy = make_shared<VideoFeatureMatchingStrategy>(featureSequence);
            }
        } else {
            if (featureSequence != 0) {
                PhotogrammetrieCli::logger.warn(
                        "Ungültige Sequenzlänge: " + to_string(featureSequence) + ". Benutze default (Ungeordnet).");
            }

            matchingStrategy = make_shared<UnorderedFeatureMatchingStrategy>();
        }

        return matchingStrategy;
    }

    cv::Ptr<cv::Feature2D> PhotogrammetrieCli::configureFeatureDetector() {
        cv::Ptr<cv::Feature2D> featureDetector;
        string featureDetectorName = PhotogrammetrieCli::args.getArg("feature-detector");
        int featureLimit = stoi(PhotogrammetrieCli::args.getArg("feature-limit", "10000"));

        if (featureDetectorName == "ORB") {
            featureDetector = cv::ORB::create(featureLimit);
        } else {
            if (featureDetectorName != "SIFT" && !featureDetectorName.empty()) {
                PhotogrammetrieCli::logger.warn(
                        "Unbekannter Merkmalsalgorithmus: " + featureDetectorName + ". Benutze SIFT.");
            }
            featureDetector = cv::SIFT::create(featureLimit, 3, 0.09);
        }
        return featureDetector;
    }

    cv::Ptr<cv::DescriptorMatcher> PhotogrammetrieCli::configureFeatureMatcher() {
        cv::Ptr<cv::DescriptorMatcher> matcher;
        string featureDetectorName = PhotogrammetrieCli::args.getArg("feature-detector");
        string featureMatcherName = PhotogrammetrieCli::args.getArg("feature-matcher");

        auto default_warning = [featureMatcherName]() -> void {
            if (featureMatcherName != "BF" && !featureMatcherName.empty()) {
                PhotogrammetrieCli::logger.warn(
                        "Unbekannter Algorithmus für den Merkmalsvergleich: " + featureMatcherName + ". Benutze BF.");
            }
        };

        if (featureDetectorName == "ORB") {
            if (featureMatcherName == "FLANN") {
                auto indexParams = cv::makePtr<cv::flann::LshIndexParams>(6, 12, 1);
                auto searchParams = cv::makePtr<cv::flann::SearchParams>(100);
                matcher = cv::makePtr<cv::FlannBasedMatcher>(indexParams, searchParams);
            } else {
                default_warning();
                matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
            }
        } else { // Also SIFT
            if (featureMatcherName == "FLANN") {
                auto indexParams = cv::makePtr<cv::flann::KDTreeIndexParams>(5);
                auto searchParams = cv::makePtr<cv::flann::SearchParams>(100);
                matcher = cv::makePtr<cv::FlannBasedMatcher>(indexParams, searchParams);
            } else {
                default_warning();
                matcher = cv::BFMatcher::create(cv::NORM_L2);
            }
        }

        return matcher;
    }

    std::filesystem::path PhotogrammetrieCli::getWorkingDir() {
        filesystem::path workdir(PhotogrammetrieCli::args.getArg("out", "./reconstruction"));
        return workdir;
    }

    void PhotogrammetrieCli::prepareWorkingDir(const std::filesystem::path &workdir) {
        filesystem::remove_all(workdir);
        filesystem::create_directories(workdir);
    }

    void PhotogrammetrieCli::checkImageParam(const string &execName) {
        // Prüfe, ob Bilder angegeben sind
        if (PhotogrammetrieCli::args.getArgCount("image") <= 0) {
            printUsage(execName, "Keine Bilder angegeben");
            exit(-1);
        }
    }

    void PhotogrammetrieCli::init(const int argc, const char **argv, const string &execName) {
        PhotogrammetrieCli::args.parseArgs(argc - 1, argv + 1);

        // Prüfe Hilfekommando
        if (PhotogrammetrieCli::args.isFlag("help")) {
            printUsage(execName, "Hilfe");
            exit(-1);
        }
    }

    void PhotogrammetrieCli::printUsage(const string &execName, const string &message) {
        PhotogrammetrieCli::logger.log(
                message
                + " \nBenutzung: " + execName + " -Prun=photogrammetrie"
                + " \n\t -Pimage*=[Pfad zu einem Bild oder Bilderordner]"
                + " \n\t -Pout=[Pfad zum Arbeits- und Ausgabeordner] = 'reconstruction'"
                + " \n\t -Pcamera-model=[Das genutzte Kameramodell (SimpleRadial, Distortion, Simple)] = 'SimpleRadial'"
                + " \n\t -Pfeature-detector=[Algorithmus für die Merkmalsfindung (ORB oder SIFT)] = 'ORB'"
                + " \n\t -Pfeature-limit=[Max Merkmale pro Bild] = 10000 (max " + to_string(1 << 18) + ")"
                + " \n\t -Pfeature-matcher=[Algorithmus für den Merkmalsvergleich (BF oder FLANN)] = 'BF'"
                +
                " \n\t -Pfeature-sequence=[Sequenzlänge für die Paarbildung zum Merkmalsvergleich (0 oder >=2)] = 0 (auto)"
                + " \n\t -Pfeature-gridlength=[Länge einer Gridreihe,Nur in Verbindung mit -Pfeature-sequence. (>=1)]"
                + " \n\t -Pmatch-threshold=[Mindestgrenze an Übereinstimmungen für Bildpaare] = 20"
                + " \n\t -Pbaseline-homography-threshold=[Mindestgrenze an Übereinstimmungen für Bildpaare] = 100"
                + " \n\t -Phomography-inlier-ratio-threshold=[Mindestverhältnis Inliers für Bildpaare] = 0.4"
                + " \n\t -Ppose-inlier-ratio-threshold=[Mindestverhältnis Inliers für Bildpaare] = 0.4"
                + " \n\t -Pransac-matching-threshold=[RANSAC Threshold] = -3.0 (>0 max(width, height)*x, <0 -1*x)"
                + " \n\t -Pransac-baseline-threshold=[RANSAC Threshold] = -1.0 (>0 max(width, height)*x, <0 -1*x)"
                + " \n\t -Pransac-pose-threshold=[RANSAC Threshold] = -8.0 (>0 max(width, height)*x, <0 -1*x)"
                + " \n\t -Preprojection-error-threshold=[Max reprojection error] = 10.0"
                + " \n\t -Ppointcloud-feature-merge-distance=[Max feature merge distance] = 20.0"
                + " \n\t -Ppointcloud-point-merge-distance=[Max point merge distance] = 0.01"
                + " \n\t -Pomp-feature-threads=[Max Anzahl OMP Threads für die Merkmalsfindung] = 0 (auto)"
                + " \n\t --distinct-matches (Nur eindeutige Übereinstimmungen)"
                + " \n\t --colored (Die Punktwolke wird eingefärbt)"
                + " \n\t --dense (Es wird versucht eine dichte Punktwolke zu berechnen)"
                + " \n\t --sgm (Zur Berechnung der dichten Punktwolke wird der SGM Algorithmus genutzt)"
                + " \n\t --mesh (Aus der Punktwolke wird ein Mesh berechnet)"
                + " \n\t --no-decimate (Deaktiviert die Auflösungsautomatik)"
                +
                " \n\t --refine-mesh (Es wird versucht das Mesh zu verfeinern - kann ebenso zu schlechterem / low poly Mesh führen)"
                + " \n\t --stats (Zeichnet Ausführungsstatistiken auf)"
                + " \n\t --artifacts (Es werden einige Artefakte des Algorithmus in den Ausgabeordner geschrieben)"
                + " \n\t --help (Zeigt diese Hilfe an)",
                AppLogger::LOG_INFO,
                true
        );
    }
}
