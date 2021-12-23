#!/bin/bash

echo "Photogrammetrie --forceColoredOutput -Ploglevel=2 -Prun=photogrammetrie -Pimage=./images -Pcamera-model=SimpleRadial -Pfeature-detector=SIFT -Pfeature-limit=30000 -Pomp-feature-threads=4 -Pfeature-sequence=$1 -Pfeature-gridlength=$2 --colored --dense --mesh --stats --artifacts" > run.log

Photogrammetrie --forceColoredOutput -Ploglevel=2 -Prun=photogrammetrie -Pimage=./images -Pcamera-model=SimpleRadial -Pfeature-detector=SIFT -Pfeature-limit=30000 -Pomp-feature-threads=4 -Pfeature-sequence=$1 -Pfeature-gridlength=$2 --colored --dense --mesh --stats --artifacts | tee out.log

./run-pcl-stats.sh
