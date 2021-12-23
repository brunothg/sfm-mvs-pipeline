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

#ifndef __APPARGS_H__
#define __APPARGS_H__

#include <string>
#include <map>
#include <list>

namespace photogrammetrie {

    /**
     * Diese Klasse enthält die Aufrufparameter der Anwendung.
     * Verarbeitet werden Argumente der Form:
     *  (Parameter) -P[name]=[wert]
     * 
     * @author Marvin Bruns
     */
    class AppArgs {
    private:
        static const char DIVIDER;
        static const char FLAG_SET;
        static const char FLAG_UNSET;

        std::multimap<std::string, std::string> args;

    public:
        AppArgs() = default;

        ~AppArgs() = default;

        /**
         * Setzt (und ersetzt) die Aufrufparameter.
         * Im Anschluss befinden sich nur die übergebenen Parameter in dem Objekt.
         *
         * @param argc Anzahl der Argumente
         * @param argv Die Argumente
         */
        void parseArgs(const int argc, const char **argv);

        /**
         * Gibt die Werte zu einem Schlüssel
         *
         * @param key Der Schlüssel, zu dem die Werte ermittelt werden soll
         * @return Eine Liste mit den Werten
         */
        std::list<std::string> getArgs(const std::string key) const;

        /**
         * Gibt den Wert zu dem Schlüssel.
         * Gibt es mehrere Werte zu dem Schlüssel, wird der erste Wert genommen.
         *
         * @param key Der Schlüssel, zu dem der Werte ermittelt werden soll
         * @param defaultValue Standardwert, wenn nicht vorhanden
         * @return Der Wert oder einen leeren String, wenn nicht vorhanden
         */
        std::string getArg(const std::string key, const std::string defaultValue = "") const;

        /**
         * Gibt die Anzahl an Werten für den Schlüssel
         *
         * @param key Der Schlüssel, zu dem die Wertezahl ermittelt werden soll
         */
        int getArgCount(const std::string key) const;

        /**
         * Prüft, ob ein Flag gesetzt ist
         *
         * @param key Name der Flag
         * @return true, wenn die Flag gesetzt wurde
         */
        bool isFlag(const std::string key) const;

        /**
         * Generiert eine textuelle Darstellung der Argumente
         *
         * @return Eine String Darstellung des Objektes
         */
        std::string toString() const;

    };

}

#endif