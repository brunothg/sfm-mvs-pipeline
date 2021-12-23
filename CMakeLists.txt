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

# set the project name and version
project(Photogrammetrie VERSION 1.3.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# tests
include(CTest)
enable_testing()

# OpenCV
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "Found OpenCV ${OpenCV_VERSION}: ${OpenCV_LIBRARIES}")

# Ceres
find_package(Ceres 1 REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
message(STATUS "Found Ceres ${CERES_VERSION}: ${CERES_LIBRARIES}")

# Boost
find_package(Boost REQUIRED COMPONENTS iostreams program_options system serialization)
include_directories(${Boost_INCLUDE_DIRS})
message(STATUS "Found BOOST ${Boost_VERSION}: ${Boost_LIBRARIES}")

# CGAL
find_package(CGAL REQUIRED)

# openMVS
set(OpenMVS_DIR /usr/local/lib/CMake/OpenMVS)
find_package(OpenMVS REQUIRED)
include_directories(${OpenMVS_INCLUDE_DIRS})
add_definitions(${OpenMVS_DEFINITIONS})
message(STATUS "Found OpenMVS ${OpenMVS_VERSION}: ${OpenMVS_LIBRARIES}")

# PCL
find_package(PCL 1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
message(STATUS "Found PCL ${PCL_VERSION}: ${PCL_LIBRARIES}")

# OpenMP
find_package(OpenMP REQUIRED)

# profile settings
string( TOUPPER "${CMAKE_BUILD_TYPE}" UPPER_CMAKE_BUILD_TYPE )
message("Build type: ${UPPER_CMAKE_BUILD_TYPE}")
if (UPPER_CMAKE_BUILD_TYPE STREQUAL "DEBUG")
    message("Debug Mode")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Og")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Og")
else(UPPER_CMAKE_BUILD_TYPE STREQUAL "RELEASE")
    message("Release Mode")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif ()
message("C_FLAGS: ${CMAKE_C_FLAGS}")
message("CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

# packaging
SET(CPACK_GENERATOR "DEB")
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "Marvin Bruns")
SET(CPACK_PACKAGE_VENDOR "Marvin Bruns")
SET(CPACK_PACKAGE_VERSION_MAJOR ${Photogrammetrie_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${Photogrammetrie_VERSION_MINOR})
SET(CPACK_PACKAGE_VERSION_PATCH ${Photogrammetrie_VERSION_PATCH})
SET(CPACK_PACKAGE_DESCRIPTION "Masterarbeit - Generierung von Höhendaten durch Photogrammetrie")
SET(CPACK_DEBIAN_PACKAGE_DEPENDS "")
INCLUDE(CPack)

# app config
set(app_NAME "${CMAKE_PROJECT_NAME}")
set(app_VERSION "${Photogrammetrie_VERSION_MAJOR}.${Photogrammetrie_VERSION_MINOR}.${Photogrammetrie_VERSION_PATCH}")

# add directories
add_subdirectory(src)
