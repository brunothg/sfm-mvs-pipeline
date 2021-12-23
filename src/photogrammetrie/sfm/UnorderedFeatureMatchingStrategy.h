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

#ifndef PHOTOGRAMMETRIE_UNORDEREDFEATUREMATCHINGSTRATEGY_H
#define PHOTOGRAMMETRIE_UNORDEREDFEATUREMATCHINGSTRATEGY_H


#include "IFeatureMatchingStrategy.h"
#include "photogrammetrie/util/AppLogger.h"


namespace photogrammetrie {

    /**
     * Stellt eine Strategie zum finden von Übereinstimmungen bei ungeordneten Bildern.
     * Diese Strategie stellt keine besonderen Anforderungen an die Ausgangslage.
     *
     * @author Marvin Bruns
     */
    class UnorderedFeatureMatchingStrategy : public IFeatureMatchingStrategy {
    private:
        static const AppLogger logger;

    public:
        UnorderedFeatureMatchingStrategy();

        void calculateShotMatches(const Scene &scene, cv::Ptr<cv::DescriptorMatcher> &matcher, vector<ShotMatches> &matches) override;
    };

}


#endif //PHOTOGRAMMETRIE_UNORDEREDFEATUREMATCHINGSTRATEGY_H
