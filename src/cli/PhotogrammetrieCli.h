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

#ifndef PHOTOGRAMMETRIE_PHOTOGRAMMETRIECLI_H
#define PHOTOGRAMMETRIE_PHOTOGRAMMETRIECLI_H

#include <string>
#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "util/AppArgs.h"
#include "photogrammetrie/common/Scene.h"
#include "photogrammetrie/util/AppLogger.h"
#include "photogrammetrie/sfm/IFeatureMatchingStrategy.h"
#include "photogrammetrie/util/OpenMvsUtils.h"

namespace photogrammetrie {

    /**
     * Die Startklasse der Anwendung für Photogrammetrie.
     * Liest die Kommandozeilenparameter aus und konfiguriert danach
     * die weitere Programmausführung.
     *
     * @author Marvin Bruns
     */
    class PhotogrammetrieCli {
    private:
        static const AppLogger logger;

    public:
        static AppArgs args;

        PhotogrammetrieCli() = default;

        ~PhotogrammetrieCli() = default;

        /**
         * Dieser Methode werden die Startargumente
         * aus der main-Methode übergeben.
         *
         * @param argc Anzahl der Argumente
         * @param argv Die Argumente
         */
        static void main(int argc, const char **argv);

    private:
        /**
         * Gibt einen Infostring zur benutzung des Programms aus.
         *
         * @param execName Pfad zum Programm (normalerweise das Argument mit dem Index 0)
         * @param message Eine Nachricht, die dazu angezeigt werden soll
         */
        static void printUsage(const string &execName, const string &message = "");

        static void init(int argc, const char **argv, const string &execName);

        static void checkImageParam(const string &execName);

        static void prepareWorkingDir(const std::filesystem::path &workdir);

        static std::filesystem::path getWorkingDir();

        static cv::Ptr<cv::Feature2D> configureFeatureDetector();

        static cv::Ptr<cv::DescriptorMatcher> configureFeatureMatcher();

        static shared_ptr<IFeatureMatchingStrategy> configureFeatureMatcherStrategy();

        static Scene prepareScene();

        static void runSfM(Scene &scene);

        static void runMVS(const std::filesystem::path &workdir, const string &filename);

        static void writeArtifacts(const Scene &scene);
    };

}

#endif //PHOTOGRAMMETRIE_PHOTOGRAMMETRIECLI_H
