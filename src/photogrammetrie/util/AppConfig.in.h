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

#ifndef __APPCONFIG_H__
#define __APPCONFIG_H__

#include <string>
#include "photogrammetrie/Photogrammetrie.h"

namespace photogrammetrie {

    /**
     * Diese Klasse enthält einige Informationen zu der Anwendung.
     * 
     * @author Marvin Bruns
     */
    class AppConfig {
    private:

    public:
        /**
         * Der Name der Anwendung
         */
        static const string appName;

        /**
         * Die Version der Anwendung
         */
        static const string appVersion;

    };

    inline const string AppConfig::appName = "${app_NAME}";
    inline const string AppConfig::appVersion = "${app_VERSION}";

}

#endif