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

#ifndef PHOTOGRAMMETRIE_SCENE_H
#define PHOTOGRAMMETRIE_SCENE_H

#include <vector>
#include <memory>

#include "photogrammetrie/Photogrammetrie.h"
#include "photogrammetrie/common/CameraShot.h"

namespace photogrammetrie {

    /**
     * Repräsentiert zwei Aufnahmen mit Übereinstimmungen
     *
     * @author Marvin Bruns
     */
    class ShotMatches {
    private:
        /**
         * Linke Sicht
         */
        shared_ptr<CameraShot> left;

        /**
         * Rechte Sicht
         */
        shared_ptr<CameraShot> right;

        /**
         * Gefundene Übereinstimmungen.
         * Left -> queryIdx und Right -> trainIdx.
         */
        vector<cv::DMatch> matches;

        /**
         * Das Verhältnis von Inliers zur Gesamtzahl von Matches der Homographie
         */
        double homographyInlierRatio = -1;

    public:
        ShotMatches(const shared_ptr<CameraShot> &left, const shared_ptr<CameraShot> &right);

        /**
         * Gibt die linke Sicht
         *
         * @return Die linke Sicht
         */
        [[nodiscard]] const shared_ptr<CameraShot> &getLeft() const;

        /**
         * Gibt die rechte Sicht
         *
         * @return Die rechte Sicht
         */
        [[nodiscard]] const shared_ptr<CameraShot> &getRight() const;

        /**
         * Gibt die Übereinstimmungen <br>
         * Left -> queryIdx und Right -> trainIdx.
         *
         * @return Die Übereinstimmungen
         */
        [[nodiscard]] const vector<cv::DMatch> &getMatches() const;

        /**
         * Setzt die Übereinstimmungen
         *
         * @param matches Die neuen Übereinstimmungen
         */
        void setMatches(const vector<cv::DMatch> &matches);

        /**
         * Fügt eine Übereinstimmung hinzu
         *
         * @param match Die Übereinstimmung
         */
        void addMatch(const cv::DMatch &match);

        /**
         * Gibt das Verhältnis von Inliers zur Gesamtzahl von Matches der Homographie
         *
         * @return Das Das Verhältnis von Inliers zur Gesamtzahl von Matches der Homographie
         */
        [[nodiscard]] double getHomographyInlierRatio() const;

        /**
         * Setzt das Verhältnis von Inliers zur Gesamtzahl von Matches der Homographie
         * @param homographyInlierRatio Das Verhältnis von Inliers zur Gesamtzahl von Matches der Homographie
         */
        void setHomographyInlierRatio(double homographyInlierRatio);

        /**
         * Matches werden sortiert und ausrgerichtet.
         * Bei gleichem Index Entspricht z.B. ein Merkmal aus dem linken Bild exakt dem Merkmal im rechten Bild.
         * Ein solcher Index passt ebenso zu den ShotMatches::matches.
         *
         * @param alignedFeaturesLeft Die Sortierten Merkmale für das linke Bild (out)
         * @param alignedFeaturesRight Die Sortierten Merkmale für das reche Bild (out)
         */
        void alignFeatures(Features &alignedFeaturesLeft, Features &alignedFeaturesRight) const;

        /**
         * Gibt die Match-Punkte der Bilder.
         *
         * @param featurePoints Merkmale (linesBild, rechtesBild) (out)
         */
        void getFeaturePoints(vector<pair<cv::Point2d, cv::Point2d>> &featurePoints) const;

        /**
         * Gibt an, ob beide Sichten wiederhergestellt wurden
         *
         * @return true, wenn left und right recovered sind
         */
        [[nodiscard]] bool isRecovered() const;

        [[nodiscard]] string toString() const;
    };

    /**
     * Repräsentiert ein Element einer Punktwolke
     */
    class PointcloudElement {
    public:
        class Origin;

    private:
        /**
         * Koordinaten des Punktes
         */
        cv::Point3d coordinates;

        /**
         * Die Farbe des Punktes oder (Scalar(-1)), wenn aus den Ursprungsdaten bestimmt werden kann und soll.
         */
        cv::Scalar color;

        /**
         * Die Ursprungsdaten
         */
        vector<shared_ptr<PointcloudElement::Origin>> origins;

    public:
        explicit PointcloudElement(cv::Point3d coordinates);

        /**
         * Gibt die Koordinaten
         *
         * @return Die Koordinaten
         */
        [[nodiscard]] const cv::Point3d &getCoordinates() const;

        /**
         * Setzt die Koordinaten
         *
         * @param coordinates Die Koordinaten
         */
        void setCoordinates(const cv::Point3d &coordinates);

        /**
         * Gibt die Ursprünge
         *
         * @return Die Ursprünge
         */
        [[nodiscard]] const vector<shared_ptr<PointcloudElement::Origin>> &getOrigins() const;

        /**
         * Setzt die Ursprünge
         *
         * @param coordinates Die Ursprünge
         */
        void setOrigins(const vector<shared_ptr<PointcloudElement::Origin>> &origins);

        /**
         * Fügt einen Ursprung hinzu
         *
         * @param origin Der Ursprung
         * @return Der hinzugefügte Ursprung oder der bereits vorhandene Ursprung
         */
        shared_ptr<PointcloudElement::Origin> addOrigin(const shared_ptr<PointcloudElement::Origin> &origin);

        /**
         * Gibt eine Liste mit Ursprungsaufnahmen und -punkten
         *
         * @param originPoints Ursprungsaufnahmen und -punkte (out)
         */
        void getOriginPoints(vector<pair<shared_ptr<CameraShot>, cv::Point2d>> &originPoints) const;

        /**
         * Gibt eine Liste mit den Urpsrungsaufnahmen
         *
         * @param originShots Ursprungsaufnahmen (out)
         */
        void getOriginShots(vector<shared_ptr<CameraShot>> &originShots) const;

        /**
         * Setzt eine feste Farbe oder Scalar(-1) für den Automatikmodus (Nur verfügbar, wenn Ursprungsdaten angegeben sind)
         * @param color Die Farbe des Punktes
         */
        void setRgba(const cv::Scalar &color);

        /**
         * Gibt an, ob die Farbe bereits gesetzt oder berechnet wurde.
         *
         * @return True, wenn die Farbe nicht mehr berechnet werden muss.
         */
        bool isRgbaCalculated() const;

        /**
         * Berechnet und setzt die Farbe des Punktes aus den Ursprungspixeln oder der fest gesetzen Farbe.
         * Dazu wird das Bild in den Speicher geladen.
         * Deswegen sollte diese Methode mit Bedacht genutzt werden.
         *
         * @see Scene::colorizePointcloud
         * @return Die Punktfarbe
         */
        [[nodiscard]] cv::Scalar getRgba();

        [[nodiscard]] string toString() const;
    };

    /**
     * Ursprunggsdaten eines Pointcloud Elements
     */
    class PointcloudElement::Origin {
    private:
        /**
         * Die Ursprungsaufnahmen
         */
        shared_ptr<ShotMatches> match;

        /**
         * Features der Aufnhamen
         */
        pair<cv::Point2d, cv::Point2d> features;

    public:
        Origin(const shared_ptr<ShotMatches> &match, const pair<cv::Point2d, cv::Point2d> &features);

        /**
         * Gibt das Bildpaar
         *
         * @return Das Bildpaar
         */
        [[nodiscard]] const shared_ptr<ShotMatches> &getMatch() const;

        /**
         * Gibt die Übereinstimmung des Bildpaares
         *
         * @return Die Übereinstimmung
         */
        [[nodiscard]] const pair<cv::Point2d, cv::Point2d> &getFeatures() const;
    };

    /**
     * Repräsentiert die 3D->2D Übereinstimmungen einer Aufnahme.
     */
    class ShotMatches3d2d {
    public:
        class Match;

    private:
        /**
         * Die Sicht
         */
        shared_ptr<CameraShot> shot;

        /**
         * Die 3D<->2D Übereinstimmungen
         */
        vector<ShotMatches3d2d::Match> matches;

    public:
        explicit ShotMatches3d2d(const shared_ptr<CameraShot> &shot);

        /**
         * Gibt die Übereinstimmungen
         *
         * @return Die Übereinstimmungen
         */
        [[nodiscard]] const vector<ShotMatches3d2d::Match> &getMatches() const;

        /**
         * Setzt die Übereinstimmungen
         *
         * @param matches Die neuen Übereinstimmungen
         */
        void setMatches(const vector<ShotMatches3d2d::Match> &matches);

        /**
         * Fügt eine Übereinstimmung hinzu
         *
         * @param match Die neue Übereinstimmung
         */
        void addMatch(ShotMatches3d2d::Match &match);

        /**
         * Gibt die Sicht
         *
         * @return Die Sicht
         */
        [[nodiscard]] const shared_ptr<CameraShot> &getShot() const;

        /**
         * Gibt die distinkte Menge der 3D- und 2D-Punkte. Die Vektoren sind zueinander ausgerichtet.
         * D.h. bei gegebenem Index gehören 3D- und 2D-Punkt an diesem Index zusammen.
         *
         * @param points3d Die 3D Punkte
         * @param points2d Die 2D Punkte
         */
        void getDistinct3d2dPoints(vector<cv::Point3d> &points3d, vector<cv::Point2d> &points2d) const;

        /**
         * Gibt die distinkte Menge der Bildpaare
         *
         * @param shotMatches Die Bildpaare
         */
        void getDistinctShotMatches(vector<shared_ptr<ShotMatches>> &shotMatches) const;
    };

    class ShotMatches3d2d::Match {
    private:
        shared_ptr<PointcloudElement> point3d;
        cv::Point2d point2d;
        shared_ptr<ShotMatches> shotMatch;

    public:
        Match(const shared_ptr<PointcloudElement> &point3D, cv::Point2d point2D,
              const shared_ptr<ShotMatches> &shotMatch);

        [[nodiscard]] const shared_ptr<PointcloudElement> &getPoint3D() const;

        [[nodiscard]] const cv::Point2d &getPoint2D() const;

        [[nodiscard]] const shared_ptr<ShotMatches> &getShotMatch() const;
    };

    /**
     * Stellt die rekonstruierte Szende dar.
     * Enthalten sind zum Beispiel die genutzten Kameras,
     * die aufgenommenen Bilder, sowie die berechneten 3D-Punkte
     *
     * @author Marvin Bruns
     */
    class Scene {
    private:
        /**
         * Bildaufnahmen der Szene
         */
        vector<shared_ptr<CameraShot>> shots;

        /**
         * Die Übereinstimmungen der Bildaufnahmen
         */
        vector<shared_ptr<ShotMatches>> shotMatches;

        /**
         * Die Punktwolke
         */
        vector<shared_ptr<PointcloudElement>> pointcloud;

    public:
        Scene();

        virtual ~Scene();

        /**
         * Gibt die Kameras der Szene
         *
         * @return Kameras
         */
        [[nodiscard]] vector<shared_ptr<ICamera>> getCameras() const;

        /**
         * Gibt die Bildaufnahmen der Szene
         *
         * @return Bildaufnahmen
         */
        [[nodiscard]] const vector<shared_ptr<CameraShot>> &getShots() const;

        /**
         * Gibt die Übereinstimmungen in den Bildaufnahmen
         *
         * @return  Übereinstimmungen
         */
        [[nodiscard]] const vector<shared_ptr<ShotMatches>> &getShotMatches() const;

        /**
         * Gibt die Punktwolke
         *
         * @return  Punktwolke
         */
        [[nodiscard]] const vector<shared_ptr<PointcloudElement>> &getPointcloud() const;

        /**
         * Leert die Punktwolke
         */
        void clearPointcloud();

        /**
         * Fügt der Szene Übereinstimmungen hinzu (wenn nicht bereits vorhanden)
         *
         * @param matches Die Übereinstimmungen
         * @return Die hinzugefügte Übereinstimmung oder die bereits vorhandene Übereinstimmung
         */
        shared_ptr<ShotMatches> addShotMatches(const shared_ptr<ShotMatches> &matches);

        /**
         * Fügt der Szene eine neue Bildaufnahme hinzu (wenn nicht bereits vorhanden)
         *
         * @param shot Die neue Bildaufnahme der Szene
         */
        void addCameraShot(const shared_ptr<CameraShot> &shot);

        /**
         * Fügt ein neues Element zur Punktwolke hinzu
         *
         * @param element Das neue Punktwolkenelement
         */
        void addPointcloudElement(const shared_ptr<PointcloudElement> &element);

        /**
         * Fügt ein neues Element zur Punktwolke hinzu.
         * Existiert bereits ein Punkt, der sich innerhalb der Merge-Distanz befindet, werden diese zu einem verbunden.
         *
         * @param element Das neue Element für die Punktwolke
         * @param pointMergeDistance Die Distanz (der 3D-Punkte), bis zu der Punkte zusammengefügt werden
         * @param featureMergeDistance Die Distanz (der entsprechenden 2D-Merkmale), bis zu der Punkte zusammengefügt werden
         * @omp parallel for
         *
         * @return Das gemergte Element
         */
        shared_ptr<PointcloudElement>
        mergePointcloudElement3d2d(const shared_ptr<PointcloudElement> &element,
                                   const double &pointMergeDistance = 0.01,
                                   const double &featureMergeDistance = 20);

        /**
         * Fügt ein neues Element zur Punktwolke hinzu.
         * Existiert bereits ein Punkt, der sich innerhalb der Merge-Distanz befindet, werden diese zu einem verbunden.
         *
         * @param element Das neue Element für die Punktwolke
         * @param pointMergeDistance Der maximale Abstand, damit Punkte zusammengefügt werden
         * @omp parallel for
         *
         * @return Das gemergte Element
         */
        shared_ptr<PointcloudElement>
        mergePointcloudElement(const shared_ptr<PointcloudElement> &element, const double &pointMergeDistance = 0.0);

        /**
         * Fügt Elemente der Punktwolke zusammen, die nah genug beieinander liegen.
         *
         * @param pointMergeDistance Der maximale Abstand, damit Punkte zusammengefügt werden
         */
        void mergePointcloud(const double &pointMergeDistance = 0.0);

        /**
         * Färbt die Punktwolke ein.
         * Zuerst werden die Farbinformationen der Ursprungsbilder genutzt.
         * Wenn diese nicht verfügbar sind, wird die Standardfarbe genutzt.
         *
         * @param defaultColor Die Standardfarbe
         * @param overwrite Wenn true, werden bereits eingefärbte Punkte überschrieben, sonst nicht
         */
        void
        colorizePointcloud(const cv::Scalar &defaultColor = cv::Scalar(0, 0, 0, 255), const bool overwrite = false);

        /**
         * Findet die 3D->2D Übereinstimmungen zu einer Aufnahme.
         *
         * @param matches Die 3D->2D Übereinstimmungen (int, out)
         * @omp parallel for
         */
        void find3d2dMatches(ShotMatches3d2d &matches) const;

        /**
         * Wandelt die Szene in eine String-Darstellung
         *
         * @return String-Darstellung
         */
        [[nodiscard]] string toString() const;

    private:
        /**
         * Führt zwei Punktwolkenelemente zusammen (Child -> Parent).
         * Der Parent enthält nacher z.B. auch die Ursprünge des Kindes
         *
         * @param parent Das Element, in das gemergt wird.
         * @param child Das Element, das gemergt wird
         */
        static void merge(const shared_ptr<PointcloudElement> &parent, const shared_ptr<PointcloudElement> &child);
    };

}

#endif //PHOTOGRAMMETRIE_SCENE_H
