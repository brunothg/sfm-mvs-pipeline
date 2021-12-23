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

#include "CeresUtils.h"

#include <ceres/ceres.h>
#include <thread>
#include "AppConfig.h"

namespace photogrammetrie {

    once_flag CeresUtils::initOnceFlag;

    void CeresUtils::init() {
        call_once(CeresUtils::initOnceFlag, CeresUtils::initOnce);
    }

    void CeresUtils::initOnce() {
        auto appName = AppConfig::appName;
        google::InitGoogleLogging(appName.c_str());
    }

    void CeresUtils::solve(ceres::Problem &problem, ceres::Solver::Options &options, ceres::Solver::Summary &summary) {
        CeresUtils::init();
        ceres::Solve(options, &problem, &summary);
    }

    void CeresUtils::defaultOptions(ceres::Solver::Options &options) {
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.max_num_iterations = 5000;
        options.minimizer_progress_to_stdout = true;
        options.logging_type = ceres::LoggingType::SILENT;
        options.eta = 0.01;
        options.num_threads = max(1, (int) thread::hardware_concurrency());
    }

    void CeresUtils::solve(ceres::Problem &problem, ceres::Solver::Summary &summary) {
        ceres::Solver::Options options;
        CeresUtils::defaultOptions(options);
        CeresUtils::solve(problem, options, summary);
    }

}