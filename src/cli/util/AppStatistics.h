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

#ifndef PHOTOGRAMMETRIE_APPSTATISTICS_H
#define PHOTOGRAMMETRIE_APPSTATISTICS_H

#include <thread>
#include <string>
#include <filesystem>

#include "photogrammetrie/util/AppLogger.h"

namespace photogrammetrie {

    /**
     * Diese Klasse kann dazu genutzt werden, Ausführungsstatistiken aufzuzeichnen
     *
     * @author Marvin Bruns
     */
    class AppStatistics {
    private:
        static const AppLogger logger;

        /**
         * Der eigene Thread
         */
        thread selfThread;

        /**
         * Gibt an, ob der Thread läuft
         */
        volatile bool running = false;

        /**
         * Interval in dem aufgezeichnet wird.
         * Angabe in Millisekunden.
         */
        unsigned int interval = 1000;

        /**
         * Pfad zu der Logdatei
         */
        const filesystem::path logPath;

    public:
        explicit AppStatistics(filesystem::path logPath);

        virtual ~AppStatistics();

        /**
         * Startet die Aufzeichnung
         */
        void start();

        /**
         * Beendet die Aufzeichnung
         */
        void stop();

    private:
        /**
         * Thread-Methode.
         * Hier wird die Statistikaufzeichnung vorgenommen.
         */
        void run();

        /**
         * Gibt die aktuelle Systemzeit in Millisekunden
         * @return Systemzeit in Millisekunden
         */
        static long getCurrentSysTimeMillis();
    };

}


#endif //PHOTOGRAMMETRIE_APPSTATISTICS_H
