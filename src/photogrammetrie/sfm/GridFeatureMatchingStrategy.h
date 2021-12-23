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

#ifndef PHOTOGRAMMETRIE_GRIDFEATUREMATCHINGSTRATEGY_H
#define PHOTOGRAMMETRIE_GRIDFEATUREMATCHINGSTRATEGY_H

#include "IFeatureMatchingStrategy.h"
#include "photogrammetrie/util/AppLogger.h"

namespace photogrammetrie {

    /**
    * Stellt eine Strategie zum finden von Übereinstimmungen bei geordneten Bildern.
    * Hierbei wird eine aufeinander folgende Bildreihe vorausgesetzt, dessen Einzelbilder entlang eines Rasters (x, y)
    * ausgerichtet sind.
    * Bei einer Sequenzlänge von 3 (selbst plus zwei Nachbarn) und einer Zeilenlänge von 5 werden für das erste Bild folgende Paare gebildet:<br>
    * Raster:<br>
    * 16 17 18 19 20<br>
    * 11 12 13 14 15<br>
    * 06 07 08 09 10<br>
    * 01 02 03 04 05<br>
    * Bildpaare: 01:02, 01:03, 01:06, 01:11, 01:07 <br>
    * Es werden Bildpaare mit n (sequenceLength -1) folgenden Bildern gebildet. Dies findet sowohl in x, als auch in y
    * Richtung statt.
    *
    * @author Marvin Bruns
    */
    class GridFeatureMatchingStrategy : public IFeatureMatchingStrategy {
    private:
        static const AppLogger logger;

        int sequenceLength = 2;
        int rowLength = 2;

    public:
        explicit GridFeatureMatchingStrategy(int sequenceLength, int rowLength);

        void setSequenceLength(int sequenceLength);

        void setRowLength(int rowLength);

        void calculateShotMatches(const Scene &scene, cv::Ptr<cv::DescriptorMatcher> &matcher, vector<ShotMatches> &matches) override;
    };

}

#endif //PHOTOGRAMMETRIE_GRIDFEATUREMATCHINGSTRATEGY_H
