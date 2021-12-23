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

## Photogrammetrie Lib
add_library(PhotogrammetrieLib)
target_compile_definitions(PhotogrammetrieLib PUBLIC _USE_OPENCV)
target_include_directories(PhotogrammetrieLib PUBLIC .)
target_link_libraries( PhotogrammetrieLib stdc++fs ${OpenCV_LIBRARIES} ${CERES_LIBRARIES} ${OpenMVS_LIBRARIES} ${PCL_LIBRARIES} OpenMP::OpenMP_CXX )

add_subdirectory(photogrammetrie)


## CLI App
add_executable(Photogrammetrie "")
target_compile_definitions(Photogrammetrie PUBLIC )
set_target_properties(Photogrammetrie PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin
        )
target_include_directories(Photogrammetrie PRIVATE .)
target_link_libraries( Photogrammetrie stdc++fs PhotogrammetrieLib )

# OpenMP
if(OpenMP_CXX_FOUND)
    target_link_libraries(Photogrammetrie OpenMP::OpenMP_CXX)
endif()

# installer
install(TARGETS Photogrammetrie
        COMPONENT photogrammetrie
        DESTINATION bin
        )

add_subdirectory(cli)
