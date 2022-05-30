# SfM-MVS Pipeline

Experimental SfM MVS pipeline

[![FU Hagen 3D Modell mit Photogrammetrie (SfM/MVS)](https://img.youtube.com/vi/_74J3FLdIpA/0.jpg)](https://www.youtube.com/watch?v=_74J3FLdIpA)
[![Wald 3D Modell mit Photogrammetrie (SfM/MVS)](https://img.youtube.com/vi/tVZjjC4iHy8/0.jpg)](https://www.youtube.com/watch?v=tVZjjC4iHy8)

## Structure

- bin  
  Binary output
- build  
  Build folder for CMake. Contains subfolders for debug and release
- docs  
  Build folder for softwaredocumentation (Doxygen)
- src  
  Folder for source code

  
## Building

The project can be built using the "build.sh" script.
Alternatively, there is a Makefile that also uses the build script in the background.
Since the libraries used contain partial support for CUDA, this should be installed beforehand if required.
- "build.sh configure" installs all required dependencies
   (or tries this; should be executed with sudo rights)
- "build.sh build" builds the project (the results can be found in the bin folder)
- "build.sh fast-test" executes a simple program call (rebuilds the project beforehand)
- "build.sh docs" creates the software documentation


## Run

When executing the program, parameters can be used to control the program flow.
These can be displayed with --help.

## Dependencies

- [OpenCV](https://opencv.org/)
- [openMVS](https://github.com/cdcseacave/openMVS)
- [Ceres](http://ceres-solver.org/)
- [PCL](https://pointclouds.org/)
- [openMP](https://www.openmp.org/)
- [Boost](https://www.boost.org/)
- [CGAL](https://www.cgal.org/)
