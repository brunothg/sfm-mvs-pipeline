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

#ifndef PHOTOGRAMMETRIE_MATHUTILS_H
#define PHOTOGRAMMETRIE_MATHUTILS_H

#include <vector>
#include <type_traits>
#include <algorithm>
#include <cmath>

#include "photogrammetrie/Photogrammetrie.h"

namespace photogrammetrie {
    /**
    * @author Marvin Bruns
    */
    class MathUtils {
    public:

        /**
         * Berechnet einige statistische Werte
         *
         * @param numbers Eingabemenge (in)
         * @param min Kleinster Wert (out)
         * @param max Größter Wert  (out)
         * @param mean Arithmetisches Mittel  (out)
         * @param variance Varianz  (out)
         * @param deviation Standardabweichung  (out)
         * @param median Der Median (out)
         */
        template<typename T, enable_if_t<is_arithmetic<T>::value, bool> = true>
        static void
        calculateStatistics(const vector<T> &numbers, T &min, T &max, double &mean, double &variance,
                            double &deviation, double &median);
    };

    template<typename T, enable_if_t<is_arithmetic<T>::value, bool>>
    void MathUtils::calculateStatistics(const vector<T> &numbersIn, T &minOut, T &maxOut, double &meanOut,
                                        double &varianceOut, double &deviationOut, double &medianOut) {
        vector<T> numbers = numbersIn;
        size_t size = numbers.size();
        sort(numbers.begin(), numbers.end());

        T minimum = 0, maximum = 0;
        double mean = 0, variance = 0, median = 0;

        if (!numbers.empty()) {

            for (auto &number : numbers) {
                minimum = min(minimum, number);
                maximum = max(maximum, number);
                mean += number;
            }
            mean /= (double) size;

            for (auto &number : numbers) {
                variance += pow((double)number - mean, 2);
            }
            variance /= ((double) size - 1.0);

            if (size % 2 == 0) {
                median = (double)(numbers[size / 2] + numbers[(size / 2) - 1]) / 2.0;
            } else {
                median = numbers[size / 2];
            }
        }

        minOut = minimum;
        maxOut = maximum;
        meanOut = mean;
        varianceOut = variance;
        deviationOut = sqrt(variance);
        medianOut = median;
    }

}

#endif //PHOTOGRAMMETRIE_MATHUTILS_H
