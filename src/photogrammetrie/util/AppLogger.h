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

#ifndef __APPLOGGER_H__
#define __APPLOGGER_H__

#include <string>
#include <thread>
#include <mutex>
#include <chrono>

#include "photogrammetrie/Photogrammetrie.h"

namespace photogrammetrie {

    /**
     * Einfache Klasse für simple Log-Nachrichten.
     * 
     * @author Marvin Bruns
     */
    class AppLogger {
    private:

        /**
         * Synchronisationsmutex
         */
        static mutex sync;

        /**
         * Startzeitpunkt der Anwendung
         */
        static chrono::system_clock::time_point startTime;

        /**
         * Name des Loggers (sollte dem Klassennamen Entsprechen).
         */
        string name;

    public:
        static const int LOG_TRACE = 0;
        static const int LOG_DEBUG = 1;
        static const int LOG_INFO = 2;
        static const int LOG_WARN = 3;
        static const int LOG_ERROR = 4;

        /**
         * Der globale Loglevel
         */
        static int loglevel;

        /**
         * Gibt an, dass auch in eine Pipe farbig geschrieben wird
         */
        static bool alwaysColored;

        explicit AppLogger(const string& name);

        ~AppLogger() = default;

        /**
         * Schreibt eine Lognachricht auf die Konsole.
         *
         * @param message Die Nachricht
         * @param logToLevel Der Loglevel
         * @param force Wenn true wird der aktuelle Loglevel ignoriert
         */
        void log(const string& message, int logToLevel = AppLogger::LOG_INFO, bool force = false) const;

        /**
         * Schreibt eine Lognachricht auf die Konsole
         *
         * @param message Die Nachricht
         */
        void trace(const string& message) const;

        /**
         * Schreibt eine Lognachricht auf die Konsole
         *
         * @param message Die Nachricht
         */
        void debug(const string& message) const;

        /**
         * Schreibt eine Lognachricht auf die Konsole
         *
         * @param message Die Nachricht
         */
        void info(const string& message) const;

        /**
         * Schreibt eine Lognachricht auf die Konsole
         *
         * @param message Die Nachricht
         */
        void warn(const string& message) const;

        /**
         * Schreibt eine Lognachricht auf die Konsole
         *
         * @param message Die Nachricht
         */
        void error(const string& message) const;
    };

}

#endif