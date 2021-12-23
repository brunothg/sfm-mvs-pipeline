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

#ifndef PHOTOGRAMMETRIE_MVS_H
#define PHOTOGRAMMETRIE_MVS_H

#include <memory>
#include <filesystem>

#include <OpenMVS/MVS.h>
#ifndef _USE_OPENCV
#define _USE_OPENCV
#endif
#include <OpenMVS/MVS/Interface.h>

#include "photogrammetrie/Photogrammetrie.h"
#include "photogrammetrie/common/Scene.h"
#include "photogrammetrie/util/AppLogger.h"

namespace photogrammetrie {

    /**
     * Diese Klasse bildet den Multi-View-Stereo (MVS) Algorithmus ab. <br>
     *
     * @author Marvin Bruns
     */
    class MVS {
    public:
        static const int MATCHING_DEFAULT = 0;
        static const int MATCHING_SGM = 1;

    private:
        static const AppLogger logger;

        openMVS::Scene omvsScene;

        bool artifacts = false;

        int matchingStrategy = MATCHING_DEFAULT;
        bool refineMesh = true;
        bool textureMesh = true;

        int resolutionReductionSteps = 0;
        int maxResolution = 3200;
        int minResolution = 640;
        int maxNeighbourViewCount = 5; // number of views used for depth-map estimation (0 - all neighbor views available)
        int minViewInlierCount = 3;  // minimum number of images that agrees with an estimate during fusion in order to consider it inlier

        float pointMergeDistance = 2.5; // Pixelabstand damit Pixel als unterschiedlich betrachtet werden

        float meshSpuriousRemoveDistance = 20;
        int meshCloseHoles = 30; // Versucht kleinere Löcher zu schließen
        bool decimateMesh = true; // Verkleinert das Mesh
        int meshSmoothIterations = 2; // Versucht das Mesh etwas zu glätten

    public:
        MVS(const std::filesystem::path &workdir, const string &filename);

        virtual ~MVS();

        /**
         * Gibt an, ob alle Artefakte geschrieben werden sollen.
         * Ansonsten richten sich die geschriebenen Artefakte nach dem Log-Level.
         * (Beeinflusst ebenfalls das MVS Logging - wird auf maximum gesetzt)
         * <br>
         * Die Artefakte werden in das Arbeitsverzeichnis geschrieben. Dies ist unnütz,
         * wenn das Arbeitsverzeichnis automatisch gelöscht wird.
         *
         * @param artifacts Gibt an, ob alle Artefakte geschrieben werden sollen
         */
        void setArtifacts(bool artifacts);

        /**
         * Gibt an, mit welchem Verfahren Übereinstimmungen in Bildern und damit Tiefenkarten berechnet werden.
         * Genutzt werden können die Werte der MATCHING_* Konstanten.
         * Dies sind z.B. MATCHING_DEFAULT und MATCHING_SGM (Semi Global Matching).
         *
         * @param matchingStrategy Der Matching Algorithmus
         */
        void setMatchingStrategy(int matchingStrategy);

        /**
         * Gibt an, ob das Mesh verfeinert werden soll.
         * Trifft nur zu, wenn überhaupt ein Mesh erstellt wird.
         *
         * @param refineMesh Wenn true, wird das Mesh verfeinert
         */
        void setRefineMesh(bool refineMesh);

        /**
         * Gibt an, ob das Mesh mit einer Textur versehen werden soll.
         * Trifft nur zu, wenn überhaupt ein Mesh erstellt wird.
         *
         * @param textureMesh Wenn true, wird das Mesh mit einer Textur versehen
         */
        void setTextureMesh(bool textureMesh);

        /**
         * Gibt an, ob das Mesh die voll Auflösung oder eine automatisch ggf. reduzierte Auflösung nutzen soll.
         *
         * @param decimateMesh Wenn true, wird die Auflösung ggf. automatisch reduziert
         */
        void setDecimateMesh(bool decimateMesh);

        /**
         * Nachdem die Struktur einer Szene mit dem SfM Algorithmus (oder andererseits) wiederhergestellt wurde,
         * wird die Szene mit dem MVS Algorithmus weiter rekonstruiert. <br>
         * Es wird mindestens die Punktwolke verdichtet.
         */
        void densifyScene();

        /**
         * Es wird ein Mesh generiert, dieses ggf. verfeinert
         * und ggf. mit einer Textur versehen.
         */
        void meshScene();

        [[nodiscard]] const openMVS::Scene &getScene() const;

    private:
        /**
         * Setzt das Arbeitsverzeichnis.
         * Wird dieses nicht explizit gesetzt, wird versucht das Temporärverzeichnis des Betriebsystems zu nutzen.
         * Das Arbeitsverzeichnis wird ggf. erstellt und sollte keine Daten enthalten, da dieses wieder gelöscht wird.
         *
         * @param workdir Das Arbeitsverzeichnis
         */
        void setWorkdir(const filesystem::path &workdir);

        /**
         * Initialisiert OpenMVS.
         */
        void init();

    };

}

#endif //PHOTOGRAMMETRIE_MVS_H
