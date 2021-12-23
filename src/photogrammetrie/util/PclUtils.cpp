/*
* Photogrammetrie - SfM/MVS 3D reconstruction from 2D photos
* Copyright (C) 2021  Marvin Bruns
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Affero General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Affero General Public License for more details.
*
* You should have received a copy of the GNU Affero General Public License
* along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "PclUtils.h"

#include <memory>
#include <cstdint>
#include <ostream>
#include <opencv2/core/affine.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "photogrammetrie/util/MathUtils.h"
#include "photogrammetrie/util/Endians.h"

namespace photogrammetrie {

    void
    PclUtils::toPCL(const vector<shared_ptr<PointcloudElement>> &pce, pcl::PointCloud<pcl::PointXYZRGB> &pcl,
                    const cv::Scalar &color) {

#pragma omp parallel for default(none) shared(pce, color, pcl)
        for (auto &pcElement : pce) {
            pcl::PointXYZRGB pcPoint;

            auto &pceCoordinates = pcElement->getCoordinates();
            pcPoint.x = pceCoordinates.x;
            pcPoint.y = pceCoordinates.y;
            pcPoint.z = pceCoordinates.z;

            if (color[0] == -1) {
                auto rgba = pcElement->getRgba();
                pcPoint.rgba = PclUtils::toRGBA(rgba[0], rgba[1], rgba[2],
                                                rgba[3]);
            } else {
                pcPoint.rgba = PclUtils::toRGBA(color[0], color[1], color[2], color[3]);
            }

#pragma omp critical
            {
                pcl.push_back(pcPoint);
            }
        }
    }

    void PclUtils::toPCL(const vector<shared_ptr<PointcloudElement>> &pce, pcl::PointCloud<pcl::PointXYZ> &pcl) {
        for (auto &pcElement : pce) {
            pcl::PointXYZ pcPoint;

            auto &pceCoordinates = pcElement->getCoordinates();
            pcPoint.x = pceCoordinates.x;
            pcPoint.y = pceCoordinates.y;
            pcPoint.z = pceCoordinates.z;

            pcl.push_back(pcPoint);
        }
    }

    vector<pair<pcl::PointXYZ, double>>
    PclUtils::getNearestNeighbors(const pcl::PointXYZ &point, const pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree, int N) {
        auto pointcloud = kdtree.getInputCloud();

        std::vector<int> pointIdxNKNSearch(N);
        std::vector<float> pointNKNSquaredDistance(N);
        int neighborCount = kdtree.nearestKSearch(point, N, pointIdxNKNSearch, pointNKNSquaredDistance);

        vector<pair<pcl::PointXYZ, double>> neighbors;
        for (size_t i = 0; i < neighborCount; i++) {
            neighbors.emplace_back((*pointcloud)[pointIdxNKNSearch[i]], sqrt((double) pointNKNSquaredDistance[i]));
        }

        return neighbors;
    }

    void
    PclUtils::writeToNeighborPLY(const pcl::PolygonMesh &mesh, const filesystem::path &path, int maxEqual,
                                 double distanceCutoff, bool binary) {
        maxEqual = max(0, maxEqual);
        double maxDistance = 0;
        const bool use_binary = binary && (sizeof(float) == 4) && (sizeof(int32_t) == 4) && (sizeof(char) == 1) &&
                                (sizeof(u_char) == 1);

        vector<pair<pcl::PointXYZ, double>> distances;
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

            auto size = cloud->size();
            if (size > 1) {
                pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
                kdtree.setInputCloud(cloud);

                auto getNearestNeighbor = [&kdtree, &maxEqual](
                        const pcl::PointXYZ &point) -> pair<pcl::PointXYZ, double> {
                    int N = 2 + maxEqual;

                    auto neighbors = PclUtils::getNearestNeighbors(point, kdtree, N);
                    for (auto neighbor : neighbors) {
                        if (neighbor.second <= 0) {
                            continue;
                        }

                        return neighbor;
                    }

                    return pair(point, 0);
                };

                for (auto &point : (*cloud)) {
                    auto neighbor = getNearestNeighbor(point);
                    if (distanceCutoff > 0 && neighbor.second > distanceCutoff) {
                        neighbor.second = distanceCutoff;
                    }
                    auto &distance = neighbor.second;

                    if (distance > maxDistance) {
                        maxDistance = distance;
                    }
                    distances.emplace_back(point, distance);
                }
            }
        }


        ofstream ofsc(path, ofstream::binary);

        size_t pointCount = distances.size();
        size_t faceCount = mesh.polygons.size();
        // write PLY header
        ofsc << fixed << "ply" << "\n" <<
             "format "
             << ((use_binary)
                 ? ((Endians::getEndian() == Endians::Endian::Little) ? "binary_little_endian" : "binary_big_endian")
                 : "ascii") << " 1.0" << "\n" <<
             "element vertex " << pointCount << "\n" <<
             "property float x" << "\n" <<
             "property float y" << "\n" <<
             "property float z" << "\n" <<
             "property uchar red" << "\n" <<
             "property uchar green" << "\n" <<
             "property uchar blue" << "\n" <<
             "property float quality" << "\n" <<
             "element face " << faceCount << "\n" <<
             "property list uchar int vertex_indices" << "\n" <<
             "property uchar red" << "\n" <<
             "property uchar green" << "\n" <<
             "property uchar blue" << "\n" <<
             "property float quality" << "\n" <<
             "end_header" << endl;

        for (auto &distanceElement : distances) {
            auto &point = distanceElement.first;
            float x = point.x, y = point.y, z = point.z, q = distanceElement.second;
            u_char r = 0, g = 0, b = 0;

            double qColorRatio = distanceElement.second / maxDistance;
            b = 255 * qColorRatio;
            r = 255 * (1.0 - qColorRatio);

            if (use_binary) {
                char buf[4 * sizeof(float) + 3 * sizeof(uchar)];
                copy(((char *) &x), ((char *) &x) + sizeof(float), buf + (0 * sizeof(float) + 0 * sizeof(uchar)));
                copy(((char *) &y), ((char *) &y) + sizeof(float), buf + (1 * sizeof(float) + 0 * sizeof(uchar)));
                copy(((char *) &z), ((char *) &z) + sizeof(float), buf + (2 * sizeof(float) + 0 * sizeof(uchar)));
                copy(((char *) &r), ((char *) &r) + sizeof(uchar), buf + (3 * sizeof(float) + 0 * sizeof(uchar)));
                copy(((char *) &g), ((char *) &g) + sizeof(uchar), buf + (3 * sizeof(float) + 1 * sizeof(uchar)));
                copy(((char *) &b), ((char *) &b) + sizeof(uchar), buf + (3 * sizeof(float) + 2 * sizeof(uchar)));
                copy(((char *) &q), ((char *) &q) + sizeof(float), buf + (3 * sizeof(float) + 3 * sizeof(uchar)));

                ofsc.write(buf, sizeof(buf));
            } else {
                ostringstream vertexLine;
                vertexLine
                        << fixed
                        << x << " " << y << " " << z
                        << " " << (int) r << " " << (int) g << " " << (int) b
                        << " " << q;

                ofsc << vertexLine.str() << "\n";
            }
        }
        ofsc.flush();

#pragma omp parallel for default(none) shared(mesh, distances, maxDistance, ofsc, use_binary)
        for (auto &face : mesh.polygons) {
            if (face.vertices.size() > 255) {
                continue;
            }

            u_char vertexCount = face.vertices.size();
            int32_t vertexIdx[vertexCount];
            copy(face.vertices.begin(), face.vertices.end(), vertexIdx);

            u_char r = 0, g = 0, b = 0;
            float q = 0;
            for (auto &idx : vertexIdx) {
                q += distances[idx].second;

                double qColorRatio = distances[idx].second / maxDistance;
                b += 255 * qColorRatio;
                r += 255 * (1.0 - qColorRatio);
            }
            q /= vertexCount;
            r /= vertexCount;
            g /= vertexCount;
            b /= vertexCount;

            if (use_binary) {
                char buf[1 * sizeof(float) + 4 * sizeof(uchar) + vertexCount * sizeof(int32_t)];
                copy(((char *) &vertexCount) + 0, ((char *) &vertexCount) + sizeof(u_char),
                     buf + (0 * sizeof(uchar) + 0 * sizeof(int32_t) + 0 * sizeof(float)));
                for (int i = 0; i < vertexCount; i++) {
                    copy(((char *) &vertexIdx[i]) + 0, ((char *) &vertexIdx[i]) + sizeof(int32_t),
                         buf + (1 * sizeof(uchar) + (0 + i) * sizeof(int32_t) + 0 * sizeof(float)));
                }
                copy(((char *) &r), ((char *) &r) + sizeof(u_char),
                     buf + (1 * sizeof(uchar) + (0 + vertexCount) * sizeof(int32_t) + 0 * sizeof(float)));
                copy(((char *) &g), ((char *) &g) + sizeof(u_char),
                     buf + (2 * sizeof(uchar) + (0 + vertexCount) * sizeof(int32_t) + 0 * sizeof(float)));
                copy(((char *) &b), ((char *) &b) + sizeof(u_char),
                     buf + (3 * sizeof(uchar) + (0 + vertexCount) * sizeof(int32_t) + 0 * sizeof(float)));
                copy(((char *) &q), ((char *) &q) + sizeof(float),
                     buf + (4 * sizeof(uchar) + (0 + vertexCount) * sizeof(int32_t) + 0 * sizeof(float)));

#pragma omp critical
                {
                    ofsc.write(buf, sizeof(buf));
                }
            } else {
                ostringstream faceLine;
                faceLine
                        << fixed
                        << (int) vertexCount;
                for (int i = 0; i < vertexCount; i++) {
                    faceLine << " " << (int) vertexIdx[i];
                }
                faceLine << " " << (int) r << " " << (int) g << " " << (int) b
                         << " " << q;

#pragma omp critical
                {
                    ofsc << faceLine.str() << "\n";
                }
            }
        }

        ofsc.flush();
        ofsc.close();
    }

    void
    PclUtils::writeToNeighborCSV(const pcl::PointCloud<pcl::PointXYZ> &pcl, const filesystem::path &outPath,
                                 double resolution, int maxEqual) {
        maxEqual = max(0, maxEqual);

        map<double, unsigned long long> distancesMap;
        vector<double> distances;

        auto size = pcl.size();
        if (size > 1) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            for (auto &point : pcl) {
                cloud->push_back(point);
            }

            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            kdtree.setInputCloud(cloud);

            auto getNearestNeighbor = [&kdtree, &maxEqual](
                    const pcl::PointXYZ &point) -> pair<pcl::PointXYZ, double> {
                int N = 2 + maxEqual;

                auto neighbors = getNearestNeighbors(point, kdtree, N);
                for (auto neighbor : neighbors) {
                    if (neighbor.second <= 0) {
                        continue;
                    }

                    return neighbor;
                }

                return pair(point, 0);
            };

			distances.reserve(size);
#pragma omp parallel for default(none) shared(cloud, getNearestNeighbor, distances)
            for (auto &point : (*cloud)) {
                auto neighbor = getNearestNeighbor(point);
#pragma omp critical
				{
					distances.push_back(neighbor.second);
				}
            }

        }

        // Auflösung automtaisch bestimmen (Varianz)
        if (resolution < 0) {
            double _, varianceDistance = 0;
            MathUtils::calculateStatistics(distances, _, _, _, varianceDistance, _, _);

            resolution = varianceDistance;
        }

        for (auto &distance : distances) {
            double distanceId = (resolution <= 0) ? distance : ceil(distance / resolution) * resolution;
            if (distancesMap.count(distanceId) == 0) {
                distancesMap.insert(make_pair(distanceId, 1));
            } else {
                distancesMap[distanceId] = distancesMap[distanceId] + 1;
            }
        }

        ofstream ofs(outPath);
        ofs << fixed << "Distance" << "," << "Count" << endl;

        for (auto &distance : distancesMap) {
            ofs << distance.first << "," << distance.second << "\n";
        }

        ofs.flush();
        ofs.close();
    }

    void PclUtils::writeToStatisticsCSV(const pcl::PointCloud<pcl::PointXYZ> &pcl, const filesystem::path &outPath,
                                        int maxEqual) {
        maxEqual = max(0, maxEqual);

        auto size = pcl.size();
        double maxDistance = 0, minDistance = -1, avgDistance = 0, medianDistance = 0, varianceDistance = 0, deviationDistance = 0;

        vector<double> distances;
        if (size > 1) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            for (auto &point : pcl) {
                cloud->push_back(point);
            }

            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            kdtree.setInputCloud(cloud);

            auto getNearestNeighbor = [&kdtree, &maxEqual](
                    const pcl::PointXYZ &point) -> pair<pcl::PointXYZ, double> {
                int N = 2 + maxEqual;

                auto neighbors = getNearestNeighbors(point, kdtree, N);
                for (auto neighbor : neighbors) {
                    if (neighbor.second <= 0) {
                        continue;
                    }

                    return neighbor;
                }

                return pair(point, 0);
            };

            distances.reserve(size);
#pragma omp parallel for default(none) shared(cloud, getNearestNeighbor, distances)
            for (auto &point : (*cloud)) {
                auto neighbor = getNearestNeighbor(point);
#pragma omp critical
				{
					distances.push_back(neighbor.second);
				}
            }
        }

        MathUtils::calculateStatistics(distances, minDistance, maxDistance, avgDistance, varianceDistance,
                                       deviationDistance, medianDistance);

        ofstream ofs(outPath);
        ofs << fixed << "N_Points" << "," << "Max_Distance" << "," << "Min_Distance"
            << "," << "Avg_Distance" << "," << "Median_Distance" << "," << "Variance_Distance" << ","
            << "Deviation_Distance" << endl;

        ofs << size << "," << maxDistance << "," << minDistance
            << "," << avgDistance << "," << medianDistance << "," << varianceDistance << "," << deviationDistance
            << endl;

        ofs.flush();
        ofs.close();
    }

    void PclUtils::writeToPLY(const filesystem::path &path, const vector<shared_ptr<PointcloudElement>> &pce,
                              const cv::Scalar &color, bool binary) {
        const bool use_binary = binary && (sizeof(float) == 4) && (sizeof(char) == 1) && (sizeof(u_char) == 1);

        ofstream ofsc(path, ofstream::binary);

        size_t pointCount = pce.size();
        // write PLY header
        ofsc << fixed << "ply" << "\n" <<
             "format "
             << ((use_binary)
                 ? ((Endians::getEndian() == Endians::Endian::Little) ? "binary_little_endian" : "binary_big_endian")
                 : "ascii") << " 1.0" << "\n" <<
             "element vertex " << pointCount << "\n" <<
             "property float x" << "\n" <<
             "property float y" << "\n" <<
             "property float z" << "\n" <<
             "property uchar red" << "\n" <<
             "property uchar green" << "\n" <<
             "property uchar blue" << "\n" <<
             "end_header" << endl;

#pragma omp parallel for default(none) shared(pce, color, ofsc, use_binary)
        for (auto &pceElement : pce) {
            auto &coordinates = pceElement->getCoordinates();
            cv::Scalar pceColor = (color[0] == -1) ? pceElement->getRgba() : color;

            if (use_binary) {
                char buf[3 * sizeof(float) + 3 * sizeof(uchar)];
                float x = coordinates.x, y = coordinates.y, z = coordinates.z;
                u_char r = pceColor[0], g = pceColor[1], b = pceColor[2];
                copy(((char *) &x), ((char *) &x) + sizeof(float), buf + (0 * sizeof(float) + 0 * sizeof(uchar)));
                copy(((char *) &y), ((char *) &y) + sizeof(float), buf + (1 * sizeof(float) + 0 * sizeof(uchar)));
                copy(((char *) &z), ((char *) &z) + sizeof(float), buf + (2 * sizeof(float) + 0 * sizeof(uchar)));
                copy(((char *) &r), ((char *) &r) + sizeof(uchar), buf + (3 * sizeof(float) + 0 * sizeof(uchar)));
                copy(((char *) &g), ((char *) &g) + sizeof(uchar), buf + (3 * sizeof(float) + 1 * sizeof(uchar)));
                copy(((char *) &b), ((char *) &b) + sizeof(uchar), buf + (3 * sizeof(float) + 2 * sizeof(uchar)));

#pragma omp critical
                {
                    ofsc.write(buf, sizeof(buf));
                }
            } else {
                ostringstream vertexLine;
                vertexLine
                        << fixed
                        << ((float) coordinates.x) << " " << ((float) coordinates.y) << " " << ((float) coordinates.z)
                        << " " << (int) ((u_char) pceColor[0]) << " " << (int) ((u_char) pceColor[1]) << " "
                        << (int) ((u_char) pceColor[2]);

#pragma omp critical
                {
                    ofsc << vertexLine.str() << "\n";
                }
            }
        }

        ofsc.flush();
        ofsc.close();
    }

    uint32_t PclUtils::toRGBA(uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
        return ((uint32_t) a << 24 | (uint32_t) r << 16 | (uint32_t) g << 8 | (uint32_t) b);
    }

    void PclUtils::writeToPLY(const filesystem::path &path, const vector<shared_ptr<CameraShot>> &allShots,
                              const cv::Scalar &color, bool onlyRecovered) {
        vector<shared_ptr<CameraShot>> shots = allShots;

        if (onlyRecovered) {
            shots.erase(remove_if(
                    shots.begin(), shots.end(),
                    [](const shared_ptr<CameraShot> &i) {
                        return !i->isRecovered();
                    }), shots.end());
        }

        ofstream ofsc(path);

        size_t shotCount = shots.size();

        // write PLY header
        ofsc << fixed << "ply" << "\n" <<
             "format ascii 1.0" << "\n" <<
             "element vertex " << (shotCount * 5L /* pro Kamera */ + 4 /* Koordinatensystem */) << "\n" <<
             "property float x" << "\n" <<
             "property float y" << "\n" <<
             "property float z" << "\n" <<
             "property uchar red" << "\n" <<
             "property uchar green" << "\n" <<
             "property uchar blue" << "\n" <<
             "element edge " << (shotCount * 8L /* pro Kamera */ + 3 /* Koordinatensystem */) << "\n" <<
             "property int vertex1" << "\n" <<
             "property int vertex2" << "\n" <<
             "property uchar red" << "\n" <<
             "property uchar green" << "\n" <<
             "property uchar blue" << "\n" <<
             "end_header" << endl;

        // write coordinate system points
        ofsc << 0 << " " << 0 << " " << 0 << " 0 0 0" << "\n";
        ofsc << 1 << " " << 0 << " " << 0 << " 255 0 0" << "\n";
        ofsc << 0 << " " << 1 << " " << 0 << " 0 255 0" << "\n";
        ofsc << 0 << " " << 0 << " " << 1 << " 0 0 255" << "\n";

        // write vertices
        for (auto &shot : shots) {
            vector<cv::Point3d> shotPoints; // focalPoint, topLeft, topRight, bottomLeft, bottomRight

            auto size = shot->getImageSize();
            double norm = 1.0 / max(size.width, size.height);
            double width = (double) size.width * norm, height = (double) size.height * norm;

            auto &camera = shot->getCamera();
            double centerX, centerY, focalLength;
            camera->getCenter(centerX, centerY);
            camera->getFocalLength(focalLength);
            centerX *= norm;
            centerY *= norm;
            focalLength *= norm;

            double leftX = -centerX;
            double rightX = width - centerX;
            double topY = -centerY;
            double bottomY = height - centerY;


            shotPoints.emplace_back(0, 0, 0);
            shotPoints.emplace_back(leftX, topY, focalLength);
            shotPoints.emplace_back(rightX, topY, focalLength);
            shotPoints.emplace_back(leftX, bottomY, focalLength);
            shotPoints.emplace_back(rightX, bottomY, focalLength);

            cv::Affine3d transformation(shot->getPose());
            for (auto &point : shotPoints) {
                cv::Point3d transformedPoint = transformation * point;
                ofsc << transformedPoint.x << " " << transformedPoint.y << " " << transformedPoint.z << " "
                     << (int) ((u_char) color[0])
                     << " " << (int) ((u_char) color[1]) << " " << (int) ((u_char) color[2]) << "\n";
            }
        }

        // write edges
        for (size_t i = 0; i < shots.size(); i++) {
            long startId = (long) (4 + (i * 5));

            long focalId = startId;
            long topLeftId = startId + 1;
            long topRightId = startId + 2;
            long bottomLeftId = startId + 3;
            long bottomRightId = startId + 4;

            // focal -> topLeft, topRight, bottomLeft, bottomRight
            ofsc << focalId << " " << topLeftId << " " << (int) ((u_char) color[0]) << " " << (int) ((u_char) color[1])
                 << " "
                 << (int) ((u_char) color[2]) << "\n";
            ofsc << focalId << " " << topRightId << " " << (int) ((u_char) color[0]) << " " << (int) ((u_char) color[1])
                 << " "
                 << (int) ((u_char) color[2]) << "\n";
            ofsc << focalId << " " << bottomLeftId << " " << (int) ((u_char) color[0]) << " "
                 << (int) ((u_char) color[1]) << " "
                 << (int) ((u_char) color[2]) << "\n";
            ofsc << focalId << " " << bottomRightId << " " << (int) ((u_char) color[0]) << " "
                 << (int) ((u_char) color[1]) << " "
                 << (int) ((u_char) color[2]) << "\n";

            // topLeft -> topRight -> bottomRight -> bottomLeft -> topLeft
            ofsc << topLeftId << " " << topRightId << " " << (int) ((u_char) color[0]) << " "
                 << (int) ((u_char) color[1]) << " "
                 << (int) ((u_char) color[2]) << "\n";
            ofsc << topRightId << " " << bottomRightId << " " << (int) ((u_char) color[0]) << " "
                 << (int) ((u_char) color[1]) << " "
                 << (int) ((u_char) color[2]) << "\n";
            ofsc << bottomRightId << " " << bottomLeftId << " " << (int) ((u_char) color[0]) << " "
                 << (int) ((u_char) color[1]) << " "
                 << (int) ((u_char) color[2])
                 << "\n";
            ofsc << bottomLeftId << " " << topLeftId << " " << (int) ((u_char) color[0]) << " "
                 << (int) ((u_char) color[1]) << " "
                 << (int) ((u_char) color[2]) << "\n";
        }

        // write coordinate system edges
        ofsc << 0 << " " << 1 << " 255 0 0" << "\n";
        ofsc << 0 << " " << 2 << " 0 255 0" << "\n";
        ofsc << 0 << " " << 3 << " 0 0 255" << "\n";

        ofsc.flush();
        ofsc.close();
    }

}