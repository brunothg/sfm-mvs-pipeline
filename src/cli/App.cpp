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

#include "App.h"

#include <thread>

#include "PhotogrammetrieCli.h"
#include "PclStatsCli.h"

namespace photogrammetrie {

    const AppLogger App::logger = AppLogger("App");
    AppArgs App::args = AppArgs();

    void App::main(const int argc, const char **argv) {
        string execName = argv[0];

        args.parseArgs(argc - 1, argv + 1);
        AppLogger::loglevel = stoi(args.getArg("loglevel", to_string(AppLogger::LOG_INFO)));
        AppLogger::alwaysColored = args.isFlag("forceColoredOutput");

        // Genutzte Argumente ausgeben
        App::logger.debug("Argumente: \n" + App::args.toString());

        // OMP Threads limitieren
        const int cores = max(1, (int) thread::hardware_concurrency());
        const int ompThreads = stoi(PhotogrammetrieCli::args.getArg("omp-cpu-threads", to_string(cores)));
        omp_set_num_threads(ompThreads);
        App::logger.debug("Anzahl OMP Threads: " + to_string(ompThreads));

        auto runCmd = App::args.getArg("run", "help");
        if (runCmd == "photogrammetrie") {
            PhotogrammetrieCli::main(argc, argv);
        } else if (runCmd == "pcl-stats") {
            PclStatsCli::main(argc, argv);
        } else {
            printUsage(execName, "Hilfe");
            exit(-1);
        }

        App::logger.info("EOL");
    }

    void App::printUsage(const string &execName, const string &message) {
        const int cores = max(1, (int) thread::hardware_concurrency());

        App::logger.log(
                message
                + " \nBenutzung: " + execName
                + " \n\t -Prun=[Welches Subprogramm soll ausgeführt werden? (photogrammetrie, pcl-stats)]"
                + " \n\t -Ploglevel=[" + to_string(AppLogger::LOG_TRACE) + "..." + to_string(AppLogger::LOG_ERROR) +
                "] = 2"
                + " \n\t -Pomp-cpu-threads=[Max Anzahl OMP Threads] = " + to_string(cores)
                + " \n\t --forceColoredOutput (Es wird immer - auch in eine Pipe - farbig geschrieben)"
                + " \n\t --help (Zeigt diese Hilfe an, oder die Hilfe des Subprogramms)",
                AppLogger::LOG_INFO,
                true
        );
    }

}
