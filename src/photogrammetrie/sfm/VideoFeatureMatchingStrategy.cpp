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

#include "VideoFeatureMatchingStrategy.h"


namespace photogrammetrie {

    const AppLogger VideoFeatureMatchingStrategy::logger = AppLogger("VideoFeatureMatchingStrategy");


    VideoFeatureMatchingStrategy::VideoFeatureMatchingStrategy(int sequenceLength) {
        setSequenceLength(sequenceLength);
    }

    void VideoFeatureMatchingStrategy::setSequenceLength(int newSequenceLength) {
        if (newSequenceLength < 2) {
            throw invalid_argument("Die Sequenzlänge darf nicht kleiner als 2 sein (Selbst plus einen Nachfolger).");
        }
        sequenceLength = newSequenceLength;
    }

    void
    VideoFeatureMatchingStrategy::calculateShotMatches(const Scene &scene, cv::Ptr<cv::DescriptorMatcher> &matcher,
                                                       vector<ShotMatches> &returnMatches) {
        auto &shots = scene.getShots();

        vector<pair<size_t, size_t>> matchPairs;
        for (size_t iLeft = 0; iLeft < shots.size(); iLeft++) {
            for (size_t iRight = iLeft + 1; iRight < shots.size() && (iRight - (iLeft + 1)) < (sequenceLength - 1); iRight++) {
                matchPairs.emplace_back(iLeft, iRight);
            }
        }

        size_t max = matchPairs.size(), completed = 0;
#pragma omp parallel for default(none) shared(matchPairs, shots, scene, returnMatches, matcher, max, completed)
        for (auto &matchPair : matchPairs) {
            const auto &shotLeft = shots[matchPair.first];
            const auto &shotRight = shots[matchPair.second];
            VideoFeatureMatchingStrategy::logger.debug(
                    "Finde Übereinstimmungen in " + shotLeft->getImagePath().string() + " und " +
                    shotRight->getImagePath().string());

            vector<cv::DMatch> goodMatches;
            try {
                vector<vector<cv::DMatch>> matches;
                matcher->knnMatch(shotLeft->getFeatures().descriptors, shotRight->getFeatures().descriptors,
                                  matches, 2);

                // Filter Übereinstimmungen nach Lowe's Ratio Test
                double matchRatio = 0.7;

                for (auto &match : matches) {
                    if (match.size() >= 2) {
                        if (match[0].distance < match[1].distance * matchRatio) {
                            goodMatches.push_back(match[0]);
                        }
                    } else{
                        goodMatches.push_back(match[0]);
                    }
                }
            } catch (cv::Exception &) {
                vector<cv::DMatch> matches;
                matcher->match(shotLeft->getFeatures().descriptors, shotRight->getFeatures().descriptors, matches);
                for (auto &match : matches) {
                    goodMatches.push_back(match);
                }
            }

			ShotMatches shotMatch(shotLeft, shotRight);
			shotMatch.setMatches(goodMatches);
#pragma omp critical
            {
                returnMatches.push_back(shotMatch);
            }
            VideoFeatureMatchingStrategy::logger.debug(
                    "Gefundene Übereinstimmungen in " + shotLeft->getImagePath().string() + " und " +
                    shotRight->getImagePath().string() + ": " + to_string(goodMatches.size()));

            completed++;
            VideoFeatureMatchingStrategy::logger.info(
                    "Übereinstimmungen berechnet: " + to_string(completed) + " von " + to_string(max) + " (" +
                    to_string((int) ((100.0 / max) * completed)) + "%)");
        }
    }



}