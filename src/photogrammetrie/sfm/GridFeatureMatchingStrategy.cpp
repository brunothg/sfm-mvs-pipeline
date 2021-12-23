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

#include "GridFeatureMatchingStrategy.h"

namespace photogrammetrie {

    const AppLogger GridFeatureMatchingStrategy::logger = AppLogger("GridFeatureMatchingStrategy");

    GridFeatureMatchingStrategy::GridFeatureMatchingStrategy(int sequenceLength, int rowLength) {
        setSequenceLength(sequenceLength);
        setRowLength(rowLength);
    }

    void GridFeatureMatchingStrategy::setSequenceLength(int newSequenceLength) {
        if (newSequenceLength < 2) {
            throw invalid_argument("Die Sequenzlänge darf nicht kleiner als 2 sein (Selbst plus einen Nachfolger).");
        }
        sequenceLength = newSequenceLength;
    }

    void GridFeatureMatchingStrategy::setRowLength(int newRowLength) {
        if (newRowLength < 1) {
            throw invalid_argument("Die Zeilenlänge darf nicht kleiner als 1 sein.");
        }
        rowLength = newRowLength;
    }

    void GridFeatureMatchingStrategy::calculateShotMatches(const Scene &scene, cv::Ptr<cv::DescriptorMatcher> &matcher,
                                                           vector<ShotMatches> &returnMatches) {
        auto &shots = scene.getShots();

        size_t rowCount = ceil(shots.size() / rowLength);
        size_t grid[rowCount][rowLength];
        for (size_t i = 0; i < rowCount; i++) {
            for (size_t j = 0; j < rowLength; j++) {
                grid[i][j] = -1;
            }
        }
        for (size_t i = 0; i < shots.size(); i++) {
            size_t row = i / rowLength;
            size_t column = i % rowLength;

            grid[row][column] = i;
        }

        vector<pair<size_t, size_t>> matchPairs;
        for (size_t iRow = 0; iRow < rowCount; iRow++) {
            for (size_t iCol = 0; iCol < rowLength; iCol++) {
                if (grid[iRow][iCol] < 0) {
                    continue;
                }

                for (size_t iMRow = 0; iMRow < sequenceLength; iMRow++) {
                    for (size_t iMCol = 0; iMCol < sequenceLength; iMCol++) {
                        size_t iMRowGlobal = iRow + iMRow;
                        size_t iMColGlobal = iCol + iMCol;

                        auto isSame = iRow == iMRowGlobal && iCol == iMColGlobal;
                        auto isTriangular = (iMRow + iMCol) < sequenceLength;
                        auto isInGrid = iMRowGlobal < rowCount && iMColGlobal < rowLength;
                        if (isSame || !isTriangular || !isInGrid || grid[iMRowGlobal][iMColGlobal] < 0) {
                            continue;
                        }

                        matchPairs.emplace_back(grid[iRow][iCol], grid[iMRowGlobal][iMColGlobal]);
                    }
                }
            }
        }

        string gridMatchPairs = "Grid Paare:";
        for (auto &matchPair : matchPairs) {
            gridMatchPairs += "\n" + to_string(matchPair.first) + " : " + to_string(matchPair.second);
        }
        GridFeatureMatchingStrategy::logger.debug(gridMatchPairs);

        size_t max = matchPairs.size(), completed = 0;
#pragma omp parallel for default(none) shared(matchPairs, shots, scene, returnMatches, matcher, max, completed)
        for (auto &matchPair : matchPairs) {
            const auto &shotLeft = shots[matchPair.first];
            const auto &shotRight = shots[matchPair.second];
            GridFeatureMatchingStrategy::logger.debug(
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
            GridFeatureMatchingStrategy::logger.debug(
                    "Gefundene Übereinstimmungen in " + shotLeft->getImagePath().string() + " und " +
                    shotRight->getImagePath().string() + ": " + to_string(goodMatches.size()));

            completed++;
            GridFeatureMatchingStrategy::logger.info(
                    "Übereinstimmungen berechnet: " + to_string(completed) + " von " + to_string(max) + " (" +
                    to_string((int) ((100.0 / max) * completed)) + "%)");
        }
    }


}