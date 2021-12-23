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

#ifndef PHOTOGRAMMETRIE_BUNDLEADJUSTMENT_H
#define PHOTOGRAMMETRIE_BUNDLEADJUSTMENT_H

#include "photogrammetrie/Photogrammetrie.h"
#include "photogrammetrie/common/Scene.h"
#include "photogrammetrie/util/AppLogger.h"

namespace photogrammetrie {

    struct CeresPCE {
        shared_ptr<PointcloudElement> pce;

        /**
         * [x, y, z]
         */
        double coordinates[3]{0, 0, 0};

        /**
         * Liest die PCE Koordinaten in das Ceres Array
         */
        void read();

        /**
         * Schreibt die Ceres Koordinaten in das PCE
         */
        void write();
    };

    struct CeresCameraShot {
        shared_ptr<CameraShot> shot;

        /**
         * [3xRotation(x,y,z), 3xTranslation(x,y,z)]
         */
        double pose[6]{0, 0, 0, 0, 0, 0};

        /**
         * Liest die CameraShot Pose in das Ceres Array
         */
        void read();

        /**
         * Schreibt die Ceres Pose in den CameraShot
         */
        void write();
    };

    /**
     * Führt eine Bündelblockausgleichung durch
     *
     * @author Marvin Bruns
     */
    class BundleAdjustment {
    private:
        static const AppLogger logger;

    public:
        /**
         * Führt eine Bündelblockausgleichung einer Szene durch.
         *
         * @param scene Die Szene
         * @param ceresPointcloud Die optimierten Elemente der Punktwolke (out)
         * @param ceresCameras Die optimierten Kameras (out)
         * @param ceresCameraShots Die optimierten Kameraaufnahmen (out)
         * @param summary Das Ergebnis der Optimierung (out)
         */
        static void
        doBundleAdjustment(const Scene &scene, vector<shared_ptr<CeresPCE>> &ceresPointcloud,
                           vector<shared_ptr<ICamera>> &ceresCameras,
                           vector<shared_ptr<CeresCameraShot>> &ceresCameraShots, ceres::Solver::Summary &summary);

        /**
         * Führt eine Bündelblockausgleichung einer Szene durch
         * und wendet die Ergebnisse direkt auf die Scene an.
         *
         * @param scene Die Szene (in, out)
         * @return true, wenn die Bündelblockausgleichung erfolgreich war (d.h. Ceres Summery termination_type ist CONVERGENCE)
         */
        static bool doBundleAdjustment(Scene &scene);
    };

}


#endif //PHOTOGRAMMETRIE_BUNDLEADJUSTMENT_H
