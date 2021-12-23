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

#include "AppLogger.h"

#include <iostream>
#include <chrono>
#include <ctime>
#include <algorithm>
#include <unistd.h>

#include "AppConfig.h"

namespace photogrammetrie {

    int AppLogger::loglevel = AppLogger::LOG_INFO;
    bool AppLogger::alwaysColored = false;
    mutex AppLogger::sync;
    chrono::system_clock::time_point AppLogger::startTime = chrono::system_clock::now();

    AppLogger::AppLogger(const string &name) {
        AppLogger::name = name;
    }

    void AppLogger::log(const string &message, const int logToLevel, const bool force) const {
        if (!force && logToLevel < AppLogger::loglevel) {
            return;
        }

        time_t time = chrono::system_clock::to_time_t(chrono::system_clock::now());
        string timeString = string(ctime(&time));
        timeString.erase(remove(timeString.begin(), timeString.end(), '\n'), timeString.end());

        string loglevel_name;
        switch (logToLevel) {
            case AppLogger::LOG_ERROR:
                loglevel_name = "ERROR";
                break;
            case AppLogger::LOG_WARN:
                loglevel_name = "WARN";
                break;
            case AppLogger::LOG_INFO:
                loglevel_name = "INFO";
                break;
            case AppLogger::LOG_DEBUG:
                loglevel_name = "DEBUG";
                break;
            case AppLogger::LOG_TRACE:
                loglevel_name = "TRACE";
                break;
            default:
                loglevel_name = "Custom-" + to_string(logToLevel);
        }

        auto elapsedTime = chrono::duration_cast<chrono::seconds>(
                chrono::system_clock::now() - AppLogger::startTime).count();

        AppLogger::sync.lock();
        if (alwaysColored || isatty(fileno(stdout))) {
            cout
                    << "\n"
                    << "\033[1;35m" << "[" << loglevel_name << "]"
                    << "\033[1;33m" << "[" << elapsedTime << "]"
                    << "\033[1;31m" << "[" << timeString << ": " << AppConfig::appName << "-" << AppConfig::appVersion
                    << ": " << AppLogger::name << "]"
                    << "\033[0;34m" << " \n" << message
                    << "\033[0m" << "\n" << endl;
        } else {
            cout
                    << "\n"
                    << "[" << loglevel_name << "]"
                    << "[" << elapsedTime << "]"
                    << "[" << timeString << ": " << AppConfig::appName << "-" << AppConfig::appVersion
                    << ": " << AppLogger::name << "]"
                    << " \n" << message
                    << "\n" << endl;
        }
        AppLogger::sync.unlock();
    }

    void AppLogger::error(const string &message) const {
        this->log(message, AppLogger::LOG_ERROR);
    }

    void AppLogger::warn(const string &message) const {
        this->log(message, AppLogger::LOG_WARN);
    }

    void AppLogger::info(const string &message) const {
        this->log(message, AppLogger::LOG_INFO);
    }

    void AppLogger::debug(const string &message) const {
        this->log(message, AppLogger::LOG_DEBUG);
    }

    void AppLogger::trace(const string &message) const {
        this->log(message, AppLogger::LOG_TRACE);
    }

}