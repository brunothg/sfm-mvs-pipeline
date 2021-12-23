#!/bin/bash

echo "Photogrammetrie --forceColoredOutput -Ploglevel=2 -Prun=photogrammetrie -Pimage=./images -Pcamera-model=SimpleRadial -Pfeature-detector=SIFT -Pfeature-limit=0 -Pfeature-matcher=FLANN -Pomp-feature-threads=8 --colored --dense --mesh --stats --artifacts" > run.log

Photogrammetrie --forceColoredOutput -Ploglevel=2 -Prun=photogrammetrie -Pimage=./images -Pcamera-model=SimpleRadial -Pfeature-detector=SIFT -Pfeature-limit=0 -Pfeature-matcher=FLANN -Pomp-feature-threads=8 --colored --dense --mesh --stats --artifacts | tee out.log

./run-pcl-stats.sh
