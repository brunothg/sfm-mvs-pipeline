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

# source files
target_sources(PhotogrammetrieLib
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/CameraShot.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Scene.cpp
        ${CMAKE_CURRENT_LIST_DIR}/ICamera.cpp
        ${CMAKE_CURRENT_LIST_DIR}/SimpleCamera.cpp
        ${CMAKE_CURRENT_LIST_DIR}/DistortionCamera.cpp
        ${CMAKE_CURRENT_LIST_DIR}/SimpleRadialCamera.cpp
        ${CMAKE_CURRENT_LIST_DIR}/BundleAdjustment.cpp
        )