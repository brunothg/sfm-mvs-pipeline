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

#include "AppStatistics.h"

#include <fstream>
#include "sys/sysinfo.h"
#include <unistd.h>
#include <algorithm>

namespace photogrammetrie {

    const AppLogger AppStatistics::logger = AppLogger("AppStatistics");

    AppStatistics::AppStatistics(filesystem::path logPath) : logPath(move(logPath)) {

    }

    AppStatistics::~AppStatistics() {
        stop();
    }

    void AppStatistics::start() {
        if (running) {
            return;
        }

        AppStatistics::logger.debug("Starte Statistikaufzeichnung");

        running = true;
        selfThread = thread([this] {
            this->run();
        });
    }

    void AppStatistics::stop() {
        if (!running) {
            return;
        }

        running = false;
        selfThread.join();


        AppStatistics::logger.debug("Stoppe Statistikaufzeichnung");
    }

    long AppStatistics::getCurrentSysTimeMillis() {
        auto time = chrono::system_clock::now().time_since_epoch();
        auto ms = (time.count() * chrono::system_clock::period::num * 1000) / chrono::system_clock::period::den;

        return ms;
    }

    void AppStatistics::run() {
        auto startTimeMs = AppStatistics::getCurrentSysTimeMillis();
        auto previousTimeMs = startTimeMs;
        const int cores = max(1, (int) thread::hardware_concurrency());

        ofstream out(logPath);
        out << fixed << "time_ms"
            << ",totalVirtualMem" << ",totalVirtualMemUsed"
            << ",vm_usage" << ",resident_set"
            << ",%_cpu_time" << ",%_cpu"
            << ",num_threads" << ",nice" << ",state"
            << endl;
        unsigned long ustime_before = 0;
        while (running) {
            auto currentTimeMs = AppStatistics::getCurrentSysTimeMillis();
            auto elapsedTimeMs = currentTimeMs - startTimeMs;
            auto deltaTimeMs = currentTimeMs - previousTimeMs;
            previousTimeMs = currentTimeMs;

            struct sysinfo memInfo{};
            sysinfo (&memInfo);

            unsigned long long totalVirtualMem = memInfo.totalram;
            totalVirtualMem += memInfo.totalswap;
            totalVirtualMem *= memInfo.mem_unit;

            unsigned long long totalVirtualMemUsed = memInfo.totalram - memInfo.freeram;
            totalVirtualMemUsed += memInfo.totalswap - memInfo.freeswap;
            totalVirtualMemUsed *= memInfo.mem_unit;

            int pid = 0;
            string comm = "unknown";
            char state = '-';
            int ppid = 0, pgrp = 0, session = 0, tty_nr = 0, tpgid = 0;
            unsigned int flags = 0;
            unsigned long minflt = 0, cminflt = 0, majflt = 0, cmajflt = 0, utime = 0, stime = 0;
            long cutime = 0, cstime = 0, priority = 0, nice = 0, num_threads = 0, itrealvalue = 0;
            unsigned long long starttime = 0;
            unsigned long vsize = 0;
            long rss = 0;

            if (filesystem::exists("/proc/self/stat")) {
                ifstream stat_stream("/proc/self/stat", ios_base::in);
                stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr
                            >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
                            >> utime >> stime >> cutime >> cstime >> priority >> nice
                            >> num_threads >> itrealvalue >> starttime >> vsize >> rss;
                stat_stream.close();
            }

            long page_size = sysconf(_SC_PAGE_SIZE);
            long clk_tck = sysconf(_SC_CLK_TCK);
            unsigned long delta_clk = (deltaTimeMs * clk_tck) / 1000;

            auto vm_usage = vsize;
            auto resident_set = rss * page_size;

            auto ustime = utime + stime;
            auto ustime_delta = ustime - ustime_before;
            ustime_before = ustime;

            num_threads -= 1;

            auto percentage_time = (delta_clk == 0)
                                   ? 0
                                   : min((double) ustime_delta / (double) delta_clk, (double) num_threads);
            auto percentage_cpu = (delta_clk == 0)
                                  ? 0
                                  : ((double) ustime_delta / (double) delta_clk) / cores;

            out << elapsedTimeMs
                << "," << totalVirtualMem << "," << totalVirtualMemUsed
                << "," << vm_usage << "," << resident_set
                << "," << percentage_time << "," << percentage_cpu
                << "," << num_threads << "," << nice << "," << state
                << endl;

            this_thread::sleep_for(chrono::milliseconds(interval));
        }
        out.close();
    }

}
