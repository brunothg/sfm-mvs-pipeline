#!/bin/bash

echo "Photogrammetrie --forceColoredOutput -Ploglevel=2 -Prun=photogrammetrie -Pimage=./images -Pcamera-model=Distortion -Pfeature-detector=SIFT -Pfeature-limit=0 -Pomp-feature-threads=8 -Pfeature-matcher=FLANN -Pfeature-sequence=$1 -Pfeature-gridlength=$2 --colored --dense --mesh --stats --artifacts" > run.log

Photogrammetrie --forceColoredOutput -Ploglevel=2 -Prun=photogrammetrie -Pimage=./images -Pcamera-model=Distortion -Pfeature-detector=SIFT -Pfeature-limit=0 -Pomp-feature-threads=8 -Pfeature-matcher=FLANN -Pfeature-sequence=$1 -Pfeature-gridlength=$2 --colored --dense --mesh --stats --artifacts | tee out.log

./run-pcl-stats.sh
