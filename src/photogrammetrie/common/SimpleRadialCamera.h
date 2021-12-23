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

#ifndef PHOTOGRAMMETRIE_SIMPLERADIALCAMERA_H
#define PHOTOGRAMMETRIE_SIMPLERADIALCAMERA_H

#include "photogrammetrie/Photogrammetrie.h"
#include "photogrammetrie/common/ICamera.h"

namespace photogrammetrie {

    /**
     * Kameramodell mit einfacher radialer Verzeichnung.
     * Die Fokuslänge ist für x und y identisch (fx = fy).
     * Das Kamerazentrum ist fix und wird nicht optimiert.
     *
     * @author Marvin Bruns
     */
    class SimpleRadialCamera: public ICamera {
    private:
        /**
         * Länge des ceresRepresentation Arrays
         */
        static const int CeresRepresentationSize = 3;

        /**
         * Die Kameramatrix <br>
         *
         * f 0 cx <br>
         * 0 f cy <br>
         * 0 0  1
         *
         */
        cv::Mat_<double> K;

        /**
         * Die Verzerrungsmatrix.
         * p1 und p2 sind statisch (Wert 0) und nur vorhanden, da OpenCV diese vier Werte
         * als Minimum für Verzerrung erwartet.
         *
         * (k1, k2, p1, p2)
         *
         */
        cv::Mat_<double> distortion;

        /**
         * Ceres Darstellung der Kameraparameter.
         * <br>
         * [f, k1, k2]
         */
        double ceresRepresentation[SimpleRadialCamera::CeresRepresentationSize] = {0};

    public:
        SimpleRadialCamera();

        ~SimpleRadialCamera() override;

        void getK(cv::Mat_<double> &K) const override;

        void setK(const cv::Mat_<double> &K) override;

        [[nodiscard]] cv::Size getResolution() const override;

        void setResolution(const cv::Size &resolution) override;

        void getDistortion(cv::Mat_<double> &distortion) const override;

        ceres::CostFunction *ceresCostFunction(const cv::Point2f &observedPoint) override;

        /**
         * Baut die Kameraparameter.
         * Ist eindeutig (singleton) für eine Kamera. Überschreibt die vorherigen Werte mit den aktuellen Kamerawerten.
         * @return Kameraparameter [f, k1, k2]
         */
        double *ceresCameraParameters(bool update = true) override;

        void ceresApplyParameters() override;

        [[nodiscard]] string toString() const override;
    };

}


#endif //PHOTOGRAMMETRIE_SIMPLERADIALCAMERA_H
