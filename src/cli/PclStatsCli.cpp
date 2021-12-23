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

#include "PclStatsCli.h"

#include <pcl/io/ply_io.h>

#include "photogrammetrie/util/PclUtils.h"

namespace photogrammetrie {

    const AppLogger PclStatsCli::logger = AppLogger("PclStatsCli");
    AppArgs PclStatsCli::args = AppArgs();

    void PclStatsCli::main(const int argc, const char **argv) {
        string execName = argv[0];
        args.parseArgs(argc - 1, argv + 1);

        // Prüfe Hilfekommando
        if (PclStatsCli::args.isFlag("help")) {
            printUsage(execName, "Hilfe");
            exit(-1);
        }

        auto plyInput = PclStatsCli::args.getArg("input", "pointcloud.ply");
        PclStatsCli::logger.info("Lade Punktwolke");
        pcl::PointCloud<pcl::PointXYZ> pcl;
        pcl::PolygonMesh mesh;
        pcl::io::loadPLYFile(plyInput, mesh);
        pcl::fromPCLPointCloud2(mesh.cloud, pcl);

        auto statsOutput = PclStatsCli::args.getArg("stats", "");
        if (!statsOutput.empty()) {
            PclStatsCli::logger.info("Schreibe Statistik");
            PclUtils::writeToStatisticsCSV(pcl, statsOutput);
        }

        auto neighborsOutput = PclStatsCli::args.getArg("neighbors", "");
        if (!neighborsOutput.empty()) {
            PclStatsCli::logger.info("Schreibe Nachbarn");
            PclUtils::writeToNeighborCSV(pcl, neighborsOutput);
        }

        auto qualityOutput = PclStatsCli::args.getArg("quality", "");
        if (!qualityOutput.empty()) {
            PclStatsCli::logger.info("Schreibe Quality");
            PclUtils::writeToNeighborPLY(mesh, qualityOutput);
        }

    }

    void PclStatsCli::printUsage(const string &execName, const string &message) {
        PclStatsCli::logger.log(
                message
                + " \nBenutzung: " + execName + " -Prun=pcl-stats"
                + " \n\t -Pinput (Eingabedatei - muss eine PLY Datei sein) = 'pointcloud.ply'"
                + " \n\t -Pstats (Ausgabedatei für Statistiken - es wird eine CSV Datei geschrieben)  (optional)"
                + " \n\t -Pneighbors (Ausgabedatei für Nachbarn - es wird eine CSV Datei geschrieben)  (optional)"
                + " \n\t -Pquality (Ausgabedatei für Quality PLY)  (optional)"
                + " \n\t --help (Zeigt diese Hilfe an)",
                AppLogger::LOG_INFO,
                true
        );
    }
}