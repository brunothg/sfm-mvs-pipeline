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

#ifndef PHOTOGRAMMETRIE_SCENEUTILS_H
#define PHOTOGRAMMETRIE_SCENEUTILS_H

#include <filesystem>
#include "photogrammetrie/Photogrammetrie.h"
#include "photogrammetrie/common/Scene.h"

namespace photogrammetrie {

    /**
     * @author Marvin Bruns
     */
    class SceneUtils {
    public:

        /**
         * Schreibt eine Statistikdatei über die Abbildungsfehler einer Szene
         *
         * @param scene Die Scene
         * @param outPath Pfad zu der Ausgabedatei (CSV)
         */
        static void writeToReprojectionErrorStatisticsCSV(const Scene &scene, const filesystem::path &outPath);

        /**
         * Schreibt eine Statistikdatei über die Verteilung der Abbildungsfehler einer Szene
         *
         * @param scene Die Scene
         * @param outPath Pfad zu der Ausgabedatei (CSV)
         * @param resolution Die genutzte Skala (-1 automatisch, 0 Einzelwerte, > 0 Zusammengefasst in x Schritte)
         */
        static void
        writeToReprojectionErrorHistogramCSV(const Scene &scene, const filesystem::path &outPath, double resolution = -1);


    };

}


#endif //PHOTOGRAMMETRIE_SCENEUTILS_H
