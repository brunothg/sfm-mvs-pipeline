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

#ifndef PHOTOGRAMMETRIE_PCLUTILS_H
#define PHOTOGRAMMETRIE_PCLUTILS_H

#include <vector>
#include <filesystem>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "photogrammetrie/Photogrammetrie.h"
#include "photogrammetrie/common/Scene.h"

namespace photogrammetrie {

    /**
     * Hilfsfunktionen für die PCL Bibliothek
     *
     * @author Marvin Bruns
     */
    class PclUtils {
    public:

        /**
         * Erstellt einen RGBA Wert, wie er von PointXYZRGB::rgba genutzt wird
         *
         * @param r Rot 0-255
         * @param g Grün  0-255
         * @param b Blau  0-255
         * @param a Alpha  0-255
         * @return Der RGBA Wert
         */
        static uint32_t toRGBA(uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, uint8_t a = 255);

        /**
         * Wandelt eine PCE Punktwolke in eine PCL Punktwolke
         *
         * @param pce Die PCE Punktwolke
         * @param pcl Die PCL Punktwolke
         * @param scalar Die Farbe der Punkte (-1 = auto)
         */
        static void toPCL(const vector<shared_ptr<PointcloudElement>> &pce, pcl::PointCloud<pcl::PointXYZRGB> &pcl,
                          const cv::Scalar &color = cv::Scalar(0, 0, 0, 255));


        /**
         * Wandelt eine PCE Punktwolke in eine PCL Punktwolke
         *
         * @param pce Die PCE Punktwolke
         * @param pcl Die PCL Punktwolke
         */
        static void toPCL(const vector<shared_ptr<PointcloudElement>> &pce, pcl::PointCloud<pcl::PointXYZ> &pcl);

        /**
         * Schreibt eine Statistikdatei
         *
         * @param pcl Die Punktwolke
         * @param outPath Pfad zu der Ausgabedatei (CSV)
         * @param maxEqual Maximale Anzahl Dubletten, die ignoriert werden
         */
        static void
        writeToStatisticsCSV(const pcl::PointCloud<pcl::PointXYZ> &pcl, const filesystem::path &outPath, int maxEqual = 0);

        /**
         * Schreibt eine Punktwolke-Nachbar Statistik als CSV Datei
         *
         * @param pcl Die Punktwolke
         * @param outPath Pfad zu der Ausgabedatei (CSV)
         * @param resolution Die genutzte Skala (-1 automatisch, 0 Einzelwerte, > 0 Zusammengefasst in x Schritte)
         * @param maxEqual Maximale Anzahl Dubletten, die ignoriert werden
         */
        static void writeToNeighborCSV(const pcl::PointCloud<pcl::PointXYZ> &pcl, const filesystem::path &outPath, double resolution = -1, int maxEqual = 0);

        /**
         * Schreibt eine Punktwolke-Nachbar Statistik als PLY Datei. <br>
         * Sowohl Punkte, als auch Flächen haben die Quality Eigenschaft (siehe z.B. Meshlab).
         *
         * @param mesh Das Mesh (kann auch nur die Punktwolke enthalten)
         * @param outPath Pfad zu der Ausgabedatei (PLY)
         * @param maxEqual Maximale Anzahl Dubletten, die ignoriert werden
         * @param distanceCutoff Maximaler Abstand - alles darüber wird auf diesen Wert gesetzt (-1 auto)
         * @param binary Wenn true, wird im Binärformat geschrieben
         */
        static void writeToNeighborPLY(const pcl::PolygonMesh &mesh, const filesystem::path &outPath, int maxEqual = 0, double distanceCutoff = -1, bool binary = true);

        /**
         * Schreibt eine PCE Punktwolke in eine PLY-Datei.
         * PLY-Dateien können in externen Viewern (z.B. Meshlab) angesehen werden.
         *
         * @param path Speicherpfad
         * @param pce Die PCE Punktwolke
         * @param scalar Die Farbe der Punkte (-1 = auto)
         * @param binary Wenn true, wird das Binärformat geschrieben
         */
        static void writeToPLY(const filesystem::path &path, const vector<shared_ptr<PointcloudElement>> &pce,
                               const cv::Scalar &color = cv::Scalar(0, 0, 0, 255), bool binary = true);

        /**
         * Schreibt die Kamerapositionen in eine PLY-Datei.
         * PLY-Dateien können in externen Viewern (z.B. Meshlab) angesehen werden.
         *
         * @param path Speicherpfad
         * @param allShots Die Aufnahmen
         * @param color De Farbe der Kameras
         * @param onlyRecovered wenn true, werden nur wiederhergstellte Kameras berücksichtigt
         */
        static void writeToPLY(const filesystem::path &path, const vector<shared_ptr<CameraShot>> &allShots,
                               const cv::Scalar &color = cv::Scalar(255, 255, 0, 255), bool onlyRecovered = true);

        /**
         * Findet die Punkte in der Punktwolke, die dem übergebenen Punkt am nächsten sind.
         * Dies kann ggf. der Punkt selber sein.
         *
         * @param point Der gesuchte Punkt
         * @param kdtree Der Suchbaum zu der Punktwolke (wenn vorhanden) oder nullptr
         * @param N Die Anzahl an Punkten die gesucht werden (das Ergebnis kann ggf. weniger enthalten)
         * @return Die N nächsten Punkte und der Abstand zum Suchpunkt
         */
        static vector<pair<pcl::PointXYZ, double>>
        getNearestNeighbors(const pcl::PointXYZ &point, const pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree, int N = 1);
    };

}

#endif //PHOTOGRAMMETRIE_PCLUTILS_H
