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

#ifndef PHOTOGRAMMETRIE_OPENMVSUTILS_H
#define PHOTOGRAMMETRIE_OPENMVSUTILS_H

#include <filesystem>
#include <OpenMVS/MVS.h>

#ifndef _USE_OPENCV
#define _USE_OPENCV
#endif

#include <OpenMVS/MVS/Interface.h>

#include "photogrammetrie/Photogrammetrie.h"
#include "photogrammetrie/common/Scene.h"

namespace photogrammetrie {

    /**
     * @author Marvin Bruns
     */
    class OpenMvsUtils {
    public:

        /**
         * Wandelt eine Szene in ein Interface für openMVS.
         * Für interne Zwecke sollten absolute Pfade genutzt werden.
         * Für externe/portable Zwecke sind relative Pfade meist besser.
         *
         * @param scene Die Ursprungszene
         * @param path Pfad für die Ausgabedateien (fehlende Ordner werden ggf. angelegt)
         * @param filename Name der Interfacedatei (images nicht erlaubt)
         * @param relativePaths Die Pfadangabe (z.B. Bilder) werden relativ zum Pfad angegeben
         * @return Das openMVS Interface
         */
        static openMVS::Interface
        toOpenMVS(const Scene &scene, const filesystem::path &path, const string &filename = "mvs.bin",
                  bool relativePaths = false);

        /**
         * Wandelt ein openMVS Interface in eine openMVS Scene
         *
         * @param interface Das Interface
         * @param tempFile Die temporäre Datei zum Serialisieren (wird ggf. überschrieben und gelöscht, wenn nicht angegeben, wird versucht automatisch eines zu erstellen)
         * @return Die Scene
         */
        static openMVS::Scene
        toScene(const openMVS::Interface &interface, filesystem::path tempFile = filesystem::path());

        /**
         * Setzt die Auflösung der Bilder wieder auf die Originalgröße
         *
         * @param scene Die Szene, dessen Bilder zurückgesetzt werden sollen
         * @param selectNeighborViews Gibt an, ob die Nachbarsichten selektiert werden sollen
         */
        static void resetImageResoloution(openMVS::Scene &scene, bool selectNeighborViews = true);

        /**
         * Schreibt ein Mesh in eine PLY-Datei.
         * PLY-Dateien können in externen Viewern (z.B. Meshlab) angesehen werden.
         *
         * @param path Speicherpfad
         * @param mesh Das Mesh
         * @param scalar Die Farbe der Punkte (-1 = auto)
         * @param binary Wenn true, wird das Binärformat geschrieben
         */
        static void writeToPLY(const filesystem::path &path, const openMVS::Mesh &mesh,
                               const cv::Scalar &color = cv::Scalar(-1, 0, 0, 255), bool binary = true);

        /**
         * Schreibt eine Punktwolke in eine PLY-Datei.
         * PLY-Dateien können in externen Viewern (z.B. Meshlab) angesehen werden.
         *
         * @param path Speicherpfad
         * @param pointcloud Die Punktwolke
         * @param scalar Die Farbe der Punkte (-1 = auto)
         * @param binary Wenn true, wird das Binärformat geschrieben
         */
        static void writeToPLY(const filesystem::path &path, const openMVS::PointCloud &pointcloud,
                               const cv::Scalar &color = cv::Scalar(-1, 0, 0, 255), bool binary = true);
    };

}

#endif //PHOTOGRAMMETRIE_OPENMVSUTILS_H
