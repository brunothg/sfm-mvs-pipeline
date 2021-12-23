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

#ifndef PHOTOGRAMMETRIE_DISTORTIONCAMERA_H
#define PHOTOGRAMMETRIE_DISTORTIONCAMERA_H

#include "photogrammetrie/Photogrammetrie.h"
#include "photogrammetrie/common/ICamera.h"

namespace photogrammetrie {

    /**
     * Kameramodell mit einfacher radialer und tangentialer Verzeichnung.
     * Das Kamerazentrum (Fokuspunkt) ist unabhängig von der Auflösung.
     * Die Fokuslänge ist für x und y identisch (fx = fy).
     *
     * @author Marvin Bruns
     */
    class DistortionCamera : public ICamera {
    private:
        /**
         * Länge des ceresRepresentation Arrays
         */
        static const int CeresRepresentationSize = 7;

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
         * Die Auflösung
         */
        cv::Size resolution;

        /**
         * Die Verzerrungsmatrix
         *
         * (k1, k2, p1, p2)
         *
         */
        cv::Mat_<double> distortion;

        /**
         * Ceres Darstellung der Kameraparameter.
         * <br>
         * [f, cx, cy, k1, k2, p1, p2]
         */
        double ceresRepresentation[DistortionCamera::CeresRepresentationSize] = {0};

    public:
        DistortionCamera();

        ~DistortionCamera() override;

        void getK(cv::Mat_<double> &K) const override;

        void setK(const cv::Mat_<double> &K) override;

        cv::Size getResolution() const override;

        void setResolution(const cv::Size &resolution) override;

        void getDistortion(cv::Mat_<double> &distortion) const override;

        ceres::CostFunction *ceresCostFunction(const cv::Point2f &observedPoint) override;

        /**
         * Baut die Kameraparameter.
         * Ist eindeutig (singleton) für eine Kamera. Überschreibt die vorherigen Werte mit den aktuellen Kamerawerten.
         * @return Kameraparameter [f, cx, cy, k1, k2, p1, p2]
         */
        double *ceresCameraParameters(bool update = true) override;

        void ceresApplyParameters() override;

        [[nodiscard]] string toString() const override;
    };

}

#endif //PHOTOGRAMMETRIE_DISTORTIONCAMERA_H
