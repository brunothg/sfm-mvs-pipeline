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

#ifndef PHOTOGRAMMETRIE_CAMERASHOT_H
#define PHOTOGRAMMETRIE_CAMERASHOT_H

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
#include <memory>
#include <string>
#include <filesystem>
#include <vector>
#include <map>

#include "photogrammetrie/Photogrammetrie.h"
#include "photogrammetrie/common/ICamera.h"

namespace photogrammetrie {

    /**
     * Diese Struktur bündelt die Merkmalsinformationen
     */
    struct Features {
        vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
    };

    /**
     * Repräsentiert eine Aufnahme, die mit einer Kamera durchgeführt wurde.
     * Dazu gehören z.B. das aufgenommene Bild, die genutzte Kamera, sowie die extrinsischen (Position und Rotation) Parameter.
     *
     * @author Marvin Bruns
     */
    class CameraShot {
    private:
        shared_ptr<ICamera> camera;
        filesystem::path imagePath;
        Features features;

        /**
         * Pose der Kamera zu der Aufnahme <br>
         *
         * [R|T] (3x3|3x1)
         */
        cv::Mat_<double> pose;

        /**
         * Gibt an, ob diese Aufnahme erfolgreich wiederhergestellt wurde.
         * Nur wenn dies true ist, kann die Pose als gültig und richtig betrachtet werden.
         */
        bool recovered = false;

    public:
        /**
         * Eine Bildaufnahme, die von einer Kamera aufgenommen wurde.
         * Es sollte daraug geachtet werden, dass Bildaufnahmen, die mit der gleichen Kamera aufgenommen wurden
         * das selbe Kamera Objekt zugeordnet bekommen (keine Kopie).
         *
         * @param camera Die aufnehmende Kamera
         * @param imagePath Pfad zu der Bilddatei
         */
        CameraShot(shared_ptr<ICamera> camera, const filesystem::path &imagePath);

        virtual ~CameraShot();

        /**
         * Gibt den Bildpfad
         *
         * @return Den Bildpfad
         */
        [[nodiscard]] const filesystem::path &getImagePath() const;

        /**
         * Gibt die Bildmerkmale.
         * Kann leere Bildmerkmale und Beschreibungen enthalten.
         *
         * @return Die Bildmerkmale
         */
        [[nodiscard]] const Features &getFeatures() const;

        /**
         * Gibt die Pose der Bildaufnahme
         *
         * @return Die Pose
         */
        [[nodiscard]] const cv::Mat_<double> &getPose() const;

        /**
         * Gibt die Rotationsmatrix
         *
         * @return Die Rotationsmatrix
         */
        [[nodiscard]] cv::Mat_<double> getRotationMat() const;

        /**
         * Gibt den Rotationsvektor (Rodrigues)
         *
         * @return Der Rotationsvektor
         */
        [[nodiscard]] cv::Mat_<double> getRotationVec() const;

        /**
         * Gibt den Translationsvektor
         *
         * @return Der Translationsvektor
         */
        [[nodiscard]] cv::Mat_<double> getTranslationVec() const;

        /**
         * Gibt das Kameracenter <br>
         * C = -R.t() * t
         *
         * @return Das Kameracenter
         */
        [[nodiscard]] cv::Mat_<double> getCenter() const;

        /**
         * Setzt die Pose der Bildaufnahme
         *
         * @param pose Die Pose
         */
        void setPose(const cv::Mat_<double> &pose);

        /**
         * Lädt das Bild in den Speicher.
         * Standardmäßig wird ein Graustufenbild geladen.
         *
         * @param flags Gibt an, ob das Bild in Farbe, Graustufen, ... geladen werden soll (cv::IMREAD_*)
         * @param accessFlags Gibt an, wie auf das Bild zugegriffen wird (cv::ACCESS_*)
         * @return Das Bild
         */
        [[nodiscard]] cv::UMat
        loadImage(int flags = cv::IMREAD_GRAYSCALE, cv::AccessFlag accessFlags = cv::ACCESS_READ) const;

        /**
         * Lädt das Bild in den Speicher.
         * Standardmäßig wird ein Graustufenbild geladen.
         *
         * @param flags Gibt an, ob das Bild in Farbe, Graustufen, ... geladen werden soll (cv::IMREAD_*)
         * @return Das Bild
         */
        [[nodiscard]] cv::Mat
        loadMImage(int flags = cv::IMREAD_GRAYSCALE) const;

        /**
         * Setzt die Bildmerkmale
         *
         * @param newFeatures Die neuen Bildmerkmale
         */
        void setFeatures(const Features &newFeatures);

        /**
         * Gibt die Abmessung des Bildes.
         * Entspricht der Kameraauflösung.
         *
         * @return Bildabmessungen
         */
        [[nodiscard]] cv::Size getImageSize() const;

        /**
         * Gibt die Kamera zu der Aufnahme
         *
         * @return Die Kamera
         */
        [[nodiscard]] const shared_ptr<ICamera> &getCamera() const;

        /**
         * Gibt an, ob die Pose erfolgreich ermittelt wurde.
         * Nur wenn dies true ist, kann die Pose als korrekt angesehen werden
         *
         * @return true, wenn die Pose erfolgreich wiederhergestellt wurde
         */
        [[nodiscard]] bool isRecovered() const;

        /**
         * Setzt den Wiederherstellungsstatus
         *
         * @param recovered true, wenn die Pose erfolgreich ermittelt wurde
         */
        void setRecovered(bool recovered = true);

        /**
         * Gibt eine Stringdarstellung des Objektes
         * @return Stringdarstellung
         */
        [[nodiscard]] string toString() const;

    };

}


#endif //PHOTOGRAMMETRIE_CAMERASHOT_H
