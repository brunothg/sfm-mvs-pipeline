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

#include "MVS.h"

#include "photogrammetrie/util/OpenMvsUtils.h"

namespace photogrammetrie {
    const AppLogger MVS::logger = AppLogger("MVS");

    MVS::MVS(const std::filesystem::path &workdir, const string &filename) {

        setWorkdir(workdir);
        omvsScene = openMVS::Scene(omp_get_max_threads());
        omvsScene.Load(std::filesystem::weakly_canonical(workdir / ("" + filename)).string());
    }

    MVS::~MVS() = default;

    void MVS::setWorkdir(const filesystem::path &newWorkdir) {
        if (!filesystem::exists(newWorkdir) || !filesystem::is_directory(newWorkdir)) {
            throw invalid_argument("Es wurde ein ungültiges Arbeitsverzeichnis angegeben.");
        }
        auto workdir = newWorkdir;

        SEACAVE::g_strWorkingFolder = workdir.string();
        SEACAVE::g_strWorkingFolder = SEACAVE::Util::ensureValidFolderPath(SEACAVE::g_strWorkingFolder);
        SEACAVE::g_strWorkingFolderFull = SEACAVE::Util::getFullPath(SEACAVE::g_strWorkingFolder);
    }

    void MVS::setArtifacts(bool newArtifacts) {
        artifacts = newArtifacts;
    }

    void MVS::setMatchingStrategy(int newMatchingStrategy) {
        matchingStrategy = newMatchingStrategy;
    }

    void MVS::setRefineMesh(bool newRefineMesh) {
        refineMesh = newRefineMesh;
    }

    void MVS::setTextureMesh(bool newTextureMesh) {
        textureMesh = newTextureMesh;
    }

    void MVS::setDecimateMesh(bool newDecimateMesh) {
        decimateMesh = newDecimateMesh;
    }

    void MVS::init() {
        switch (AppLogger::loglevel) {
            case AppLogger::LOG_INFO:
                SEACAVE::g_nVerbosityLevel = 1;
                break;
            case AppLogger::LOG_DEBUG:
                SEACAVE::g_nVerbosityLevel = 2;
                break;
            case AppLogger::LOG_TRACE:
                SEACAVE::g_nVerbosityLevel = 3;
                break;
            default:
                SEACAVE::g_nVerbosityLevel = 0;
        }

        if (artifacts) {
            SEACAVE::g_nVerbosityLevel = 1000;
        }

        openMVS::OPTDENSE::init();
        openMVS::OPTDENSE::update();
        openMVS::OPTDENSE::nResolutionLevel = resolutionReductionSteps;
        openMVS::OPTDENSE::nMaxResolution = maxResolution;
        openMVS::OPTDENSE::nMinResolution = minResolution;
        openMVS::OPTDENSE::nNumViews = maxNeighbourViewCount;
        openMVS::OPTDENSE::nMinViewsFuse = minViewInlierCount;
        openMVS::OPTDENSE::nEstimateColors = 2; // estimate the colors for the dense point-cloud (0 - disabled, 1 - final, 2 - estimate)
        openMVS::OPTDENSE::nEstimateNormals = 2; // estimate the normals for the dense point-cloud (0 - disabled, 1 - final, 2 - estimate)

        Util::Init();
    }

    void MVS::densifyScene() {
        init();

        if (omvsScene.pointcloud.IsEmpty()) {
            MVS::logger.warn("Leere Punktwolke -> eine Rekonstruktion mit MVS ist nicht möglich");
            return;
        }
        if (omvsScene.images.IsEmpty()) {
            MVS::logger.warn("Keine Aufnahmen vorhanden -> eine Rekonstruktion mit MVS ist nicht möglich");
            return;
        }

        MVS::logger.info("Beginne Verdichtung:");
        // Densify
        bool densifySuccess = false;
        if (matchingStrategy == MVS::MATCHING_SGM) {
            auto disparitySuccess = omvsScene.DenseReconstruction(-1);
            auto disparityFusionSuccess = omvsScene.DenseReconstruction(-2);
            densifySuccess = disparitySuccess && disparityFusionSuccess;
        } else {
            auto depthSuccess = omvsScene.DenseReconstruction(0);
            densifySuccess = depthSuccess;
        }

        if (!densifySuccess) {
            MVS::logger.warn(
                    "... bei der Verdichtung sind Umstände aufgetreten, die eine erfolgreiche Verdichtung verhindern.");
        } else {
            MVS::logger.info("... Verdichtung abgeschlossen.");
        }
    }

    void MVS::meshScene() {
        init();

        if (omvsScene.pointcloud.IsEmpty()) {
            MVS::logger.warn("Leere Punktwolke -> eine Rekonstruktion mit MVS ist nicht möglich");
            return;
        }
        if (omvsScene.images.IsEmpty()) {
            MVS::logger.warn("Keine Aufnahmen vorhanden -> eine Rekonstruktion mit MVS ist nicht möglich");
            return;
        }

        MVS::logger.info("Beginne Meshgenerierung:");

        OpenMvsUtils::resetImageResoloution(omvsScene);

        // Mesh aufbauen
        omvsScene.ReconstructMesh(pointMergeDistance, false, 4, 1, 1);

		MVS::logger.info("Mesh wird gesäubert");
        omvsScene.mesh.Clean(1, meshSpuriousRemoveDistance, true, meshCloseHoles, meshSmoothIterations, false);
		MVS::logger.info("Löcher werden geschlossen");
        omvsScene.mesh.Clean(1, 0, true, meshCloseHoles, 0, false); // extra cleaning trying to close more holes
		MVS::logger.info("Manifold mesh");
        omvsScene.mesh.Clean(1, 0, false, 0, 0, true); // extra cleaning to remove non-manifold problems created by closing holes

        // Mesh verfeinern
        if (refineMesh && !omvsScene.mesh.IsEmpty()) {
			MVS::logger.info("Mesh wird verfeinert");
            omvsScene.RefineMesh(0,
                                 640,
                                 8,
                                 (decimateMesh) ? 0 : 1 /* 0 auto, 1 off */,
                                 meshCloseHoles,
                                 1 /* 0 off, 1 auto*/,
                                 16,
                                 3, 0.5,
                                 1, 0,
                                 0.2,
                                 0.9,
                                 0,
                                 45.05);
        }

        // Mesh texturieren
        if (textureMesh && !omvsScene.mesh.IsEmpty()) {
			MVS::logger.info("Textur wird erstellt");
            omvsScene.TextureMesh(0, 640, 0.06, 0.1, true, true, 0, 3);
        }

        MVS::logger.info("... Meshgenerierung abgeschlossen.");
    }

    const openMVS::Scene &MVS::getScene() const {
        return omvsScene;
    }

}