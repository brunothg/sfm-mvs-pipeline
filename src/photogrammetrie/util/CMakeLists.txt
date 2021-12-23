# Photogrammetrie - SfM/MVS 3D reconstruction from 2D photos
# Copyright (C) 2021  Marvin Bruns
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.2.0)

# build app config
configure_file(${CMAKE_CURRENT_LIST_DIR}/AppConfig.in.h ${CMAKE_CURRENT_LIST_DIR}/AppConfig.h)

# source files
target_sources(PhotogrammetrieLib
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/Endians.cpp
        ${CMAKE_CURRENT_LIST_DIR}/OpenMpUtils.cpp
        ${CMAKE_CURRENT_LIST_DIR}/AppLogger.cpp
        ${CMAKE_CURRENT_LIST_DIR}/MathUtils.cpp
        ${CMAKE_CURRENT_LIST_DIR}/OpenCvUtils.cpp
        ${CMAKE_CURRENT_LIST_DIR}/PclUtils.cpp
        ${CMAKE_CURRENT_LIST_DIR}/CeresUtils.cpp
        ${CMAKE_CURRENT_LIST_DIR}/OpenMvsUtils.cpp
        ${CMAKE_CURRENT_LIST_DIR}/SceneUtils.cpp
        )
