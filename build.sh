#!/bin/bash

set -e


buildDocs () {
    mkdir -p docs
    rm -R docs
    doxygen Doxyfile

    cd docs/latex
    make pdf
    cd ../..

}

installDocDependencies () {
    echo "Install documentation dependencies"

    sudo apt -y install doxygen graphviz
    sudo apt -y install texlive-full texlive-fonts-recommended texlive-fonts-extra texlive-latex-extra
    sudo apt -y install biber xindy perl
    sudo apt -y install plantuml graphviz
}

installOptional () {
  echo "Install optional dependencies"

  sudo apt -y install gdal-bin libgdal-dev
  sudo apt -y install groovy imagemagick
  #sudo apt -y install ninja-build
  sudo apt -y install meshlab
}

installDevDeps () {
    sudo apt -y install build-essential
    sudo apt -y install make
    sudo apt -y install cmake
    sudo apt -y install gcc g++
    sudo apt -y install libpcl-dev
    sudo apt -y install libomp-dev
    sudo apt -y install libceres-dev
    #sudo apt -y install liblz4-1 liblz4-dev
    #sudo apt -y install libflann-dev

    installOpenCv
    installOpenMVS
}

installOpenMVS () {
    echo "Install OpenMVS"

    if [ -f ".omvs.installed" ]; then
        echo "OpenMVS already installed"
        return
    fi

    OPENMVS_VERSION=v1.1.1
    CURRENT_DIR=$('pwd')
    N_PROC=$('nproc')

    sudo apt -y install git cmake libpng-dev libjpeg-dev libtiff-dev libglu1-mesa-dev
    sudo apt -y install libboost-iostreams-dev libboost-program-options-dev libboost-system-dev libboost-serialization-dev
    sudo apt -y install libeigen3-dev
    #sudo apt -y install libopencv-dev
    sudo apt -y install libcgal-dev libcgal-qt5-dev
    sudo apt -y install libceres-dev libatlas-base-dev libsuitesparse-dev
    sudo apt -y install freeglut3-dev libglew-dev libglfw3-dev

    #VCGLib
    sudo rm -f -R vcglib
    git clone https://github.com/cdcseacave/VCG.git vcglib

    #OpenMVS
    sudo rm -f -R openMVS
    git clone https://github.com/cdcseacave/openMVS.git openMVS
    cd openMVS
    git checkout "$OPENMVS_VERSION"
    mkdir openMVS_build
    cd openMVS_build
    cmake -G "Unix Makefiles" -D_USE_OPENCV=1 -DCMAKE_BUILD_TYPE=Release -DVCG_ROOT="$CURRENT_DIR/vcglib" ..
    make "-j$N_PROC" && sudo make install && touch ../../.omvs.installed

    cd bin
    chmod +x ./*
    for omvsfile in *; do
       sudo cp "$omvsfile" "/usr/local/bin/oMVS-$omvsfile"
      echo "$omvsfile"
    done

    cd ..
    cd ..
    cd ..


}

installOpenCv () {
    echo "Install OpenCV"
    ###############################################################################
    # vgl. https://docs.opencv.org/master/d2/de6/tutorial_py_setup_in_ubuntu.html #
    # vgl. https://linuxize.com/post/how-to-install-opencv-on-ubuntu-20-04/       #
    ###############################################################################

    OPENCV_VERSION=4.5.1
    CURRENT_DIR=$('pwd')
    N_PROC=$('nproc')


    sudo apt -y install pkg-config

    if pkg-config --modversion opencv4 &> /dev/null; then
    	TEST_INSTALL=$(pkg-config --modversion opencv4)
    	if [ $TEST_INSTALL == $OPENCV_VERSION ]; then
        	echo "OpenCV already installed"
        	return 0
        else
            echo "Wrong version: $TEST_INSTALL"
    	fi
    else
        echo "No installation available"
    fi
    echo "Install version: $OPENCV_VERSION"

    sudo apt -y install build-essential
    sudo apt -y install cmake
    sudo apt -y install gcc g++
    sudo apt -y install python3-dev python3-numpy python3-pip
    sudo apt -y install libavcodec-dev libavformat-dev libswscale-dev
    sudo apt -y install libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev
    sudo apt -y install libgtk-3-dev
    sudo apt -y install libpng-dev
    sudo apt -y install libjpeg-dev
    sudo apt -y install libopenexr-dev
    sudo apt -y install libtiff-dev
    sudo apt -y install libwebp-dev
    sudo apt -y install libomp-dev
    sudo apt -y install libv4l-dev libxvidcore-dev libx264-dev
    sudo apt -y install gfortran
    sudo apt -y install libceres-dev
    sudo apt -y install openexr libopenexr-dev
    sudo apt -y install libatlas-base-dev
    sudo apt -y install libtbb2 libtbb-dev
    sudo apt -y install libdc1394-22-dev
    sudo apt -y install git
    sudo pip3 install numpy

    git clone https://github.com/opencv/opencv.git opencv
    cd opencv
    git checkout "$OPENCV_VERSION"
    cd ..

    git clone https://github.com/opencv/opencv_contrib.git opencv_contrib
    cd opencv_contrib
    git checkout "$OPENCV_VERSION"
    cd ..

    cd opencv
    mkdir -p build

    cd build
    cmake -G "Unix Makefiles" -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_OPENMP=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D OPENCV_GENERATE_PKGCONFIG=ON -D "OPENCV_EXTRA_MODULES_PATH=$CURRENT_DIR/opencv_contrib/modules" -D BUILD_EXAMPLES=ON ..

    make "-j$N_PROC" && sudo make install
    cd ..

    cd ..
    #rm -R opencv
    #rm -R opencv_contrib


    echo "Verify"
    echo "c++"
    pkg-config --modversion opencv4
    echo "python3"
    python3 -c "import cv2; print(cv2.__version__)"
}

configure () {
    installDocDependencies
    installDevDeps
}

build () {
    echo "Building ..."

    mkdir -p build/release

    GENERATOR="Unix Makefiles"
    if command -v ninja &> /dev/null ;then
        GENERATOR="Ninja"
    fi

    # Create makefile build and build
    cd build/release
    cmake -G "$GENERATOR" -DCMAKE_BUILD_TYPE=Release ../..
    cmake --build .
    cd ../..

    echo ".. built"
}

package () {
    build "$@"

    echo "package.."
    mkdir -p bin

    cd build/release

    cpack
    mv -f *.deb ../../bin

    cd ../..


    
    echo "..packaged"
}

install () {
    cd build/release
    sudo cmake --install .
    cd ../..
}

clean () {
    rm -Rf build
    rm -Rf bin
    rm -Rf docs
}

festTest () {
    build "$@"

    cd bin
    ./Photogrammetrie -Ploglevel=0 -Prun=photogrammetrie -Pimage="../images/insel"
    cd ..
}

all () {
    configure "$@"
    buildDocs "$@"
    build "$@"
    install "$@"
}

main () {
    local TARGET="$1";

    if [ "$TARGET" == "configure" ]; then
        configure "${@:2}"
    elif [ "$TARGET" == "build" ]; then
        build "${@:2}"
    elif [ "$TARGET" == "package" ]; then
        package "${@:2}"
    elif [ "$TARGET" == "install" ]; then
        install "${@:2}"
    elif [ "$TARGET" == "clean" ]; then
        clean "${@:2}"
    elif [ "$TARGET" == "fast-test" ]; then
        festTest "${@:2}"
    elif [ "$TARGET" == "optional" ]; then
        installOptional "${@:2}"
    elif [ "$TARGET" == "docs" ]; then
        buildDocs
    elif [ "$TARGET" == "all" ]; then
        all "${@:2}"
    else
        echo "Unknown run goal. Available: configure, build, fast-test, docs, optional, clean, install, package, all"
    fi
}
main "$@"
