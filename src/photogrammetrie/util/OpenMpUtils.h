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

#ifndef PHOTOGRAMMETRIE_OPENMPUTILS_H
#define PHOTOGRAMMETRIE_OPENMPUTILS_H

namespace photogrammetrie {
    /**
    * @author Marvin Bruns
    */
    class OpenMpUtils {

    public:

        /**
         * Mit OpenMP parallelisierte Version von find_if.
         * Gibt irgendein passendes Ergebnis.
         */
        template<typename InputIterator, typename UnaryPredicate>
        static InputIterator find_if(InputIterator first, InputIterator last, UnaryPredicate pred, bool cancel = true);

        template<typename InputIterator, typename UnaryPredicate>
        static InputIterator find_if_break(InputIterator first, InputIterator last, UnaryPredicate pred);

        template<typename InputIterator, typename UnaryPredicate>
        static InputIterator find_if_nobreak(InputIterator first, InputIterator last, UnaryPredicate pred);

    };

    template<typename InputIterator, typename UnaryPredicate>
    InputIterator OpenMpUtils::find_if(InputIterator first, InputIterator last, UnaryPredicate pred, bool cancel) {
        return (cancel) ? OpenMpUtils::find_if_break(first, last, pred) : OpenMpUtils::find_if_nobreak(first, last, pred);
    }

    template<typename InputIterator, typename UnaryPredicate>
    InputIterator OpenMpUtils::find_if_break(InputIterator first, InputIterator last, UnaryPredicate pred) {
        auto found = last;

#pragma omp parallel default(none) shared(found, first, last, pred)
#pragma omp for
        for (auto it = first; it != last; ++it) {
            if (pred(*it)) {
#pragma omp critical
                {
                    found = it;
                }
#pragma omp cancel for
            }
        }

        return found;
    }

    template<typename InputIterator, typename UnaryPredicate>
    InputIterator OpenMpUtils::find_if_nobreak(InputIterator first, InputIterator last, UnaryPredicate pred) {
        auto found = last;

#pragma omp parallel default(none) shared(found, first, last, pred)
#pragma omp for
        for (auto it = first; it != last; ++it) {
            if (pred(*it)) {
#pragma omp critical
                {
                    found = it;
                }
            }
        }

        return found;
    }

}


#endif //PHOTOGRAMMETRIE_OPENMPUTILS_H
