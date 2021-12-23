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

#include "AppArgs.h"

#include <string>

namespace photogrammetrie {

    const char AppArgs::DIVIDER = '=';
    const char AppArgs::FLAG_SET = '1';
    const char AppArgs::FLAG_UNSET = '0';

    void AppArgs::parseArgs(const int argc, const char **argv) {
        this->args.clear();

        for (int i = 0; i < argc; i++) {
            std::string arg = std::string(argv[i]);
            if (arg.size() < 3 ||
                arg[0] != '-') { // Minimal -Pa Parameter mit Typ P und Namen a
                this->args.insert(std::pair("", arg)); // Sonst ein unbenannter Parameter
                continue;
            }

            auto divider_pos = arg.find(AppArgs::DIVIDER);

            std::string type = arg.substr(0, 2);
            std::string key = arg.substr(2, (divider_pos != std::string::npos) ? divider_pos - 2 : std::string::npos);
            std::string value = (divider_pos == std::string::npos) ? "" : arg.substr(divider_pos + 1);

            if (type == "-P") { // Parameter
                this->args.insert(std::pair<std::string, std::string>(key, value));
            } else if (type == "--") { // Flag
                this->args.insert(std::pair<std::string, std::string>(key, std::string(1, AppArgs::FLAG_SET)));
            }
        }

    }

    std::string AppArgs::getArg(const std::string key, const std::string defaultValue) const {
        auto m_args = this->args.equal_range(key);
        for (auto it = m_args.first; it != m_args.second; it++) {
            return it->second;
        }

        return defaultValue;
    }

    std::list<std::string> AppArgs::getArgs(const std::string key) const {
        std::list<std::string> args_result;

        auto m_args = this->args.equal_range(key);
        for (auto it = m_args.first; it != m_args.second; it++) {
            args_result.push_back(it->second);
        }

        return args_result;
    }

    int AppArgs::getArgCount(const std::string key) const {
        return this->getArgs(key).size();
    }

    bool AppArgs::isFlag(const std::string key) const {
        return getArg(key, std::to_string(AppArgs::FLAG_UNSET)) == std::string(1, AppArgs::FLAG_SET);
    }

    std::string AppArgs::toString() const {
        std::string ret = "";

        for (std::pair<std::string, std::string> arg : this->args) {
            if (ret.size() > 0) {
                ret += "\n";
            }

            ret += arg.first + " -> " + arg.second;
        }

        return ret;
    }


}