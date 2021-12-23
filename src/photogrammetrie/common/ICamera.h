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

#ifndef PHOTOGRAMMETRIE_ICAMERA_H
#define PHOTOGRAMMETRIE_ICAMERA_H

#include <string>
#include <opencv2/core.hpp>
#include <ceres/ceres.h>
#include "photogrammetrie/Photogrammetrie.h"

namespace photogrammetrie {

    /**
     * Stellt eine physische Kamera dar.
     *
     * @author Marvin Bruns
     */
    class ICamera {
    public:
        virtual ~ICamera() = default;

        /**
         * Gibt die Auflösung der Kamera
         *
         * @return Die Kameraauflösung
         */
        [[nodiscard]] virtual cv::Size getResolution() const = 0;

        /**
         * Setzt die Auflösung der Kamera
         *
         * @param resolution Die Kameraauflösung
         */
        virtual void setResolution(const cv::Size &resolution) = 0;

        /**
         * Gibt die Kameramatrix (Intrinsics) <br>
         * <br>
         * fx s cx <br>
         * 0 fy cy <br>
         * 0 0  1
         *
         * @param K Die Kameramatrix
         */
        virtual void getK(cv::Mat_<double> &K) const = 0;

        /**
         * Setzt die Kameramatrix (Intrinsics) <br>
         * <br>
         * fx s cx <br>
         * 0 fy cy <br>
         * 0 0  1 <br>
         * <br>
         * Es werden ggf. nicht alle Werte genutzt oder haben einen festen Wert.
         *
         * @param K Die Kameramatrix
         */
        virtual void setK(const cv::Mat_<double> &K) = 0;

        /**
         * Setzt das Zentrum
         *
         * @param centerX X-Koordinate
         * @param centerY Y-Koordinate
         */
        virtual void setCenter(const double &centerX, const double &centerY);

        /**
         * Gibt das Zentrum
         *
         * @param centerX X-Koordinate (Out)
         * @param centerY Y-Koordinate (Out)
         */
        virtual void getCenter(double &centerX, double &centerY) const;

        /**
         * Setzt die Fokuslänge der Kamera
         *
         * @param focalLengthX Die X-Fokuslänge
         * @param focalLengthY Die Y-Fokuslänge
         */
        virtual void setFocalLength(const double &focalLengthX, const double &focalLengthY);

        /**
         * Setzt die Fokuslänge der Kamera
         *
         * @param focalLength Die Fokuslänge
         */
        virtual void setFocalLength(const double &focalLength);

        /**
         * Gibt die Fokuslänge der Kamera.
         * Es wird der Mittelwert der x und y Werte berechnet.
         *
         * @param focalLength Die Fokuslänge (out)
         */
        virtual void getFocalLength(double &focalLength) const;

        /**
         * Gibt die Fokuslänge der Kamera
         *
         * @param focalLengthX Die Fokuslänge (out)
         * @param focalLengthY Die Fokuslänge (out)
         */
        virtual void getFocalLength(double &focalLengthX, double &focalLengthY) const;

        /**
         * Gibt die Distortion Matrix <br>
         * <br>
         * [k1,k2,p1,p2[,k3[,k4,k5,k6]]] <br>
         *  mit kn = Radiale Verzerrung und pn = Tangentiale Verzerrung
         *
         * @param distortion Die Distortion Matrix
         */
        virtual void getDistortion(cv::Mat_<double> &distortion) const;

        /**
         * Rechnet die Verzeichnung aus einem Bild anhand der Kameraparameter heraus.
         *
         * @param in Das Eingabebild
         * @param out Das Ausgabebild
         */
        void undistort(cv::InputArray in, cv::OutputArray out) const;

        /**
         * Rechnet die Verzeichnung aus Bildpunkten anhand der Kameraparameter heraus.
         *
         * @param in Die Eingabebildpunkte
         * @param out Die Ausgabebildpunkte
         */
        void undistortPoints(cv::InputArray in, cv::OutputArray out) const;

        /**
        * Baut eine Kostenfunktion für eine Bündelblockausgleichung.
        * Die Kostenfunktion muss folgende Parameter akzeptieren:
        * <ul>
        *  <li>3D Koordinaten *double[3]{x, y, z}</li>
        *  <li>Kamerapose *double[6]{3xRotation(x, y, z), 3xTranslation (x, y, z)}</li>
        *  <li>Kameraparameter *double[?] (siehe Methode double *cameraParameter())</li>
        * </ul>
        *
        * @param observedPoint Der beobachtete Punkt zu den 3D Koordinaten
        * @return Die Kostenfunktion
        */
        virtual ceres::CostFunction *ceresCostFunction(const cv::Point2f &observedPoint) = 0;

        /**
         * Baut die Kameraparameter (z.B. Fokuslänge und Verzerrung).
         * Ist eindeutig (singleton) für eine Kamera. Überschreibt die vorherigen Werte mit den aktuellen Kamerawerten.
         *
         * @param update Wenn true, werden die vorherigen Werte überschrieben, sonst nicht
         *
         * @return (Ceres) Kameraparameter
         */
        virtual double *ceresCameraParameters(bool update = true) = 0;

        /**
         * Übernimmt die aktuellen Werte der (Ceres) Kameraparameter
         */
        virtual void ceresApplyParameters() = 0;

        [[nodiscard]] virtual string toString() const = 0;
    };

}

#endif //PHOTOGRAMMETRIE_ICAMERA_H
