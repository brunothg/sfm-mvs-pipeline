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

#ifndef PHOTOGRAMMETRIE_IFEATUREMATCHINGSTRATEGY_H
#define PHOTOGRAMMETRIE_IFEATUREMATCHINGSTRATEGY_H

#include <vector>

#include "photogrammetrie/Photogrammetrie.h"
#include "photogrammetrie/common/Scene.h"

namespace photogrammetrie {

    /**
     * Interface für Strategien zum finden von Übereinstimmungen
     *
     * @author Marvin Bruns
     */
    class IFeatureMatchingStrategy {
    public:
        virtual ~IFeatureMatchingStrategy() = default;

        /**
         * Berechnet Übereinstimmungen für eine Szene
         *
         * @param scene Die Szene, für die Übereinstimmungen gesucht werden sollen
         * @param matcher Der Algorithmus zum Berechnen von Übereinstimmungen
         * @param matches Die gefundenen Matches (out)
         */
        virtual void calculateShotMatches(const Scene &scene, cv::Ptr<cv::DescriptorMatcher> &matcher,
                                          vector<ShotMatches> &matches) = 0;

    };



}


#endif //PHOTOGRAMMETRIE_IFEATUREMATCHINGSTRATEGY_H
