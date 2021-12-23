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

#ifndef __APP_H__
#define __APP_H__

#include <string>

#include "cli/util/AppArgs.h"
#include "photogrammetrie/util/AppLogger.h"

namespace photogrammetrie {

    /**
     * Die Startklasse der Anwendung.
     * Liest die Kommandozeilenparameter aus und konfiguriert danach
     * die weitere Programmausführung.
     * 
     * @author Marvin Bruns
     */
    class App {
    private:
        static const AppLogger logger;

    public:
        static AppArgs args;

        App() = default;

        ~App() = default;

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
    };

}

#endif