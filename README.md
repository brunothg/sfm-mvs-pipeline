# SfM-MVS Pipeline

Experimental SfM MVS pipeline


## Struktur

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