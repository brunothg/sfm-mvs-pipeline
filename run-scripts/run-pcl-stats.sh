#!/bin/bash

cd reconstruction

if [ -f "pointcloud_sparse.ply" ]; then
	echo "Write stats for sparse pointcloud"
	Photogrammetrie -Prun=pcl-stats -Pinput=pointcloud_sparse.ply -Pstats=pointcloud_sparse.stats.csv -Pneighbors=pointcloud_sparse.neighbors.csv -Pquality=pointcloud_sparse.quality.ply
fi

if [ -f "pointcloud_dense.ply" ]; then
	echo "Write stats for dense pointcloud"
	Photogrammetrie -Prun=pcl-stats -Pinput=pointcloud_dense.ply -Pstats=pointcloud_dense.stats.csv -Pneighbors=pointcloud_dense.neighbors.csv -Pquality=pointcloud_dense.quality.ply
fi

if [ -f "mesh_dense.ply" ]; then
	echo "Write stats for mesh"
	Photogrammetrie -Prun=pcl-stats -Pinput=mesh_dense.ply -Pstats=mesh_dense.stats.csv -Pneighbors=mesh_dense.neighbors.csv -Pquality=mesh_dense.quality.ply
fi

cd ..
