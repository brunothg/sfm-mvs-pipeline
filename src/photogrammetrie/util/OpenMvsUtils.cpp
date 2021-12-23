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

#include "OpenMvsUtils.h"

#include <regex>
#include <map>
#include <cstdio>
#include <cstdint>
#include <ostream>

#include "photogrammetrie/util/Endians.h"

namespace photogrammetrie {

    openMVS::Interface
    OpenMvsUtils::toOpenMVS(const photogrammetrie::Scene &scene, const filesystem::path &path, const string &filename, bool relativePaths) {
        filesystem::create_directories(path);
        auto interfaceFile = path / (""+filename);
        auto imagesDir = path / "images";
        filesystem::create_directories(imagesDir);

        auto shotToFileName = [](const shared_ptr<CameraShot> &shot) {
            return to_string((uintptr_t) addressof(*shot)) + ".png";
        };

        openMVS::Interface interface;


        // Erstelle eine Plattform pro Kamera (openMVS unterstützt Multi-Lense-Setups -> Platform)
        auto cameras = scene.getCameras();
        map<shared_ptr<ICamera>, size_t> cameraToId;
        interface.platforms.reserve(cameras.size());
        for (auto &camera: cameras) {
            cv::Mat_<double> K;
            camera->getK(K);

            auto resolution = camera->getResolution();

            openMVS::Interface::Platform::Camera mvsCamera;
            mvsCamera.width = resolution.width;
            mvsCamera.height = resolution.height;
            mvsCamera.K = K;
            // Position auf der Plattform (nicht in der Szene)
            mvsCamera.R = cv::Matx33d::eye();
            mvsCamera.C = cv::Point3d(0, 0, 0);

            openMVS::Interface::Platform mvsPlatform;
            mvsPlatform.cameras.push_back(mvsCamera);

            if (cameraToId.count(camera) == 0) {
                cameraToId.insert(make_pair(camera, interface.platforms.size()));
                interface.platforms.push_back(mvsPlatform);
            }
        }

        // Erstelle Bilder und Posen
        auto &shots = scene.getShots();
        map<shared_ptr<CameraShot>, size_t> shotToId;
        interface.images.reserve(shots.size());
        for (auto &shot : shots) {
            if (!shot->isRecovered() || cameraToId.count(shot->getCamera()) == 0) {
                continue;
            }

            openMVS::Interface::Image image;
            image.name = (relativePaths) ? filesystem::relative(imagesDir / shotToFileName(shot), path)
                                         : filesystem::absolute(imagesDir / shotToFileName(shot));
            image.platformID = cameraToId.at(shot->getCamera());
            image.cameraID = 0;
            image.poseID = interface.platforms[image.platformID].poses.size();

            openMVS::Interface::Platform::Pose pose;
            auto R = shot->getRotationMat();
            auto C = shot->getCenter();
            pose.R = R;
            pose.C = cv::Point3d(C.at<double>(0), C.at<double>(1), C.at<double>(2));

            if (shotToId.count(shot) == 0) {
                shotToId.insert(make_pair(shot, interface.images.size()));
                interface.platforms[image.platformID].poses.push_back(pose);
                interface.images.push_back(image);
            }
        }

        // Erstelle Punktwolke
        auto &pointcloud = scene.getPointcloud();
        interface.vertices.reserve(pointcloud.size());
        for (const auto &pce: pointcloud) {
            openMVS::Interface::Vertex vertex;
            vertex.X = pce->getCoordinates();

            vector<shared_ptr<CameraShot>> origins;
            pce->getOriginShots(origins);
            for (auto &origin : origins) {
                if (shotToId.count(origin) == 0) {
                    continue;
                }

                openMVS::Interface::Vertex::View view{
                        .imageID = 0,
                        .confidence = 0
                };
                view.imageID = shotToId.at(origin);
                vertex.views.push_back(view);
            }

            // Prüfen, ob genug Ursprünge vorhanden sind
            if (vertex.views.size() < 2) {
                continue;
            }

            // Die Ursprünge nach Id sortieren
            std::sort(vertex.views.begin(), vertex.views.end(),
                      [](auto &v1, auto &v2) { return v1.imageID < v2.imageID; });

            interface.vertices.push_back(vertex);
        }


        // Schreibe Undistorted Bilder
#pragma omp parallel for default(none) shared(shots, imagesDir, shotToFileName)
        for (auto &shot : shots) {
            if (!shot->isRecovered()) {
                continue;
            }

            auto camera = shot->getCamera();

            auto image = shot->loadImage(cv::IMREAD_UNCHANGED);
            cv::UMat undistortedImage;
            camera->undistort(image, undistortedImage);

            cv::imwrite(imagesDir / shotToFileName(shot), undistortedImage);
        }

        openMVS::ARCHIVE::SerializeSave(interface, interfaceFile);
        return interface;
    }

    openMVS::Scene OpenMvsUtils::toScene(const openMVS::Interface &interface, filesystem::path tempFile) {
        openMVS::Scene scene;

        if (tempFile.empty()) {
            auto tempDir = filesystem::temp_directory_path();
            unsigned long rnd = 1;

            while (rnd > 0) {
                try {
                    rnd++;

                    string tmpInterfaceFile = tempDir / (to_string(rnd) + "_omvsi");
                    if (filesystem::exists(tmpInterfaceFile)) {
                        continue;
                    }

                    openMVS::ARCHIVE::SerializeSave(interface, tmpInterfaceFile);

                    scene.Load(tmpInterfaceFile);

                    filesystem::remove(tmpInterfaceFile);

                    break;
                } catch (...) {}
            }
        } else {
            string tmpInterfaceFile = filesystem::weakly_canonical(tempFile).string();

            openMVS::ARCHIVE::SerializeSave(interface, tmpInterfaceFile);
            scene.Load(tmpInterfaceFile);
            filesystem::remove(tmpInterfaceFile);
        }


        return scene;
    }

    void OpenMvsUtils::resetImageResoloution(openMVS::Scene &scene, bool selectNeighborViews) {
        // Bilder neu laden, falls diese Skaliert wurden
        for (int_t idx = 0; idx < (int_t) scene.images.GetSize(); ++idx) {
            const auto idxImage((uint32_t) idx);
            openMVS::Image &imageData = scene.images[idxImage];
            if (!imageData.IsValid()) {
                continue;
            }
            // reset image resolution
            if (!imageData.ReloadImage(0, false)) {
                continue;
            }
            imageData.UpdateCamera(scene.platforms);
            // select neighbor views
            if (selectNeighborViews && imageData.neighbors.IsEmpty()) {
                IndexArr points;
                scene.SelectNeighborViews(idxImage, points);
            }
        }
    }

    void OpenMvsUtils::writeToPLY(const filesystem::path &path, const openMVS::PointCloud &pointcloud,
                                  const cv::Scalar &color, bool binary) {
        const bool use_binary = binary && (sizeof(float) == 4) && (sizeof(int32_t) == 4) && (sizeof(char) == 1) &&
                                (sizeof(u_char) == 1);
        const bool use_normals = !pointcloud.normals.IsEmpty();

        ofstream ofsc(path, ofstream::binary);

        size_t pointCount = pointcloud.points.size();

        // write PLY header
        ofsc << fixed << "ply" << "\n" <<
             "format "
             << ((use_binary)
                 ? ((Endians::getEndian() == Endians::Endian::Little) ? "binary_little_endian" : "binary_big_endian")
                 : "ascii") << " 1.0" << "\n" <<
             "element vertex " << pointCount << "\n" <<
             "property float x" << "\n" <<
             "property float y" << "\n" <<
             "property float z" << "\n";
        if (use_normals) {
            ofsc << fixed <<
                 "property float nx" << "\n" <<
                 "property float ny" << "\n" <<
                 "property float nz" << "\n";
        }
        ofsc << fixed <<
             "property uchar red" << "\n" <<
             "property uchar green" << "\n" <<
             "property uchar blue" << "\n";
        ofsc << "end_header" << endl;

#pragma omp parallel for default(none) shared(pointcloud, color, use_binary, use_normals, ofsc)
        for (size_t pIdx = 0; pIdx < pointcloud.points.size(); pIdx++) {
            auto &point = pointcloud.points[pIdx];

            float x = point.x, y = point.y, z = point.z;
            float nx = 0, ny = 0, nz = 0;
            u_char r = color[0], g = color[1], b = color[2];
            if (color[0] == -1) {
                if (!pointcloud.colors.IsEmpty() && pointcloud.colors.size() > pIdx) {
                    auto &pointColor = pointcloud.colors[pIdx];
                    r = pointColor.r;
                    g = pointColor.g;
                    b = pointColor.b;
                } else {
                    r = 0;
                    g = 0;
                    b = 0;
                }
            }

            if (use_normals && !pointcloud.normals.IsEmpty() && pointcloud.normals.size() > pIdx) {
                auto &pointNormals = pointcloud.normals[pIdx];
                nx = pointNormals.x;
                ny = pointNormals.y;
                nz = pointNormals.z;
            }

            if (use_binary) {
                char buf[((use_normals) ? 6 : 3) * sizeof(float) + 3 * sizeof(uchar)];
                copy(((char *) &x), ((char *) &x) + sizeof(float), buf + (0 * sizeof(float) + 0 * sizeof(uchar)));
                copy(((char *) &y), ((char *) &y) + sizeof(float), buf + (1 * sizeof(float) + 0 * sizeof(uchar)));
                copy(((char *) &z), ((char *) &z) + sizeof(float), buf + (2 * sizeof(float) + 0 * sizeof(uchar)));
                if (use_normals) {
                    copy(((char *) &nx), ((char *) &nx) + sizeof(float), buf + (3 * sizeof(float) + 0 * sizeof(uchar)));
                    copy(((char *) &ny), ((char *) &ny) + sizeof(float), buf + (4 * sizeof(float) + 0 * sizeof(uchar)));
                    copy(((char *) &nz), ((char *) &nz) + sizeof(float), buf + (5 * sizeof(float) + 0 * sizeof(uchar)));
                }
                copy(((char *) &r), ((char *) &r) + sizeof(uchar), buf + (((use_normals) ? 6 : 3) * sizeof(float) + 0 * sizeof(uchar)));
                copy(((char *) &g), ((char *) &g) + sizeof(uchar), buf + (((use_normals) ? 6 : 3) * sizeof(float) + 1 * sizeof(uchar)));
                copy(((char *) &b), ((char *) &b) + sizeof(uchar), buf + (((use_normals) ? 6 : 3) * sizeof(float) + 2 * sizeof(uchar)));

#pragma omp critical
                {
                    ofsc.write(buf, sizeof(buf));
                }
            } else {
                ostringstream vertexLine;
                vertexLine
                        << fixed
                        << x << " " << y << " " << z;
                if (use_normals) {
                    vertexLine << fixed << " " << nx << " " << ny << " " << nz;
                }
                vertexLine << fixed << " " << (int) r << " " << (int) g << " " << (int) b;

#pragma omp critical
                {
                    ofsc << vertexLine.str() << "\n";
                }
            }
        }

        ofsc.flush();
        ofsc.close();
    }

    void OpenMvsUtils::writeToPLY(const filesystem::path &path, const openMVS::Mesh &mesh, const cv::Scalar &color,
                                  bool binary) {
        const bool use_binary = binary && (sizeof(float) == 4) && (sizeof(int32_t) == 4) && (sizeof(char) == 1) &&
                                (sizeof(u_char) == 1);
        const bool use_texture = mesh.HasTexture() && !mesh.faceTexcoords.IsEmpty() && !mesh.textureDiffuse.empty();
        const string textureName = path.filename().string() + ".png";
        const filesystem::path texturePath = filesystem::absolute(path).parent_path() / textureName;

        if (use_texture) {
            cv::imwrite(texturePath, mesh.textureDiffuse);
        }

        ofstream ofsc(path, ofstream::binary);

        size_t pointCount = mesh.vertices.size();
        size_t faceCount = mesh.faces.size();

        // write PLY header
        ofsc << fixed << "ply" << "\n" <<
             "format "
             << ((use_binary)
                 ? ((Endians::getEndian() == Endians::Endian::Little) ? "binary_little_endian" : "binary_big_endian")
                 : "ascii") << " 1.0" << "\n" <<
             "comment " << ((use_texture) ? ("TextureFile " + textureName) : "no_texture") << "\n" <<
             "element vertex " << pointCount << "\n" <<
             "property float x" << "\n" <<
             "property float y" << "\n" <<
             "property float z" << "\n" <<
             "property uchar red" << "\n" <<
             "property uchar green" << "\n" <<
             "property uchar blue" << "\n" <<
             "element face " << faceCount << "\n" <<
             "property list uchar int vertex_indices" << "\n";
        if (use_texture) {
            ofsc << "property list uchar float texcoord" << "\n";
        }
        ofsc << "end_header" << endl;

        for (auto &vertex : mesh.vertices) {
            float x = vertex.x, y = vertex.y, z = vertex.z;
            u_char r = color[0], g = color[1], b = color[2];
            if (color[0] == -1) {
                r = (use_texture) ? 255 : 0;
                g = (use_texture) ? 255 : 0;
                b = (use_texture) ? 255 : 0;
            }

            if (use_binary) {
                char buf[3 * sizeof(float) + 3 * sizeof(uchar)];
                copy(((char *) &x), ((char *) &x) + sizeof(float), buf + (0 * sizeof(float) + 0 * sizeof(uchar)));
                copy(((char *) &y), ((char *) &y) + sizeof(float), buf + (1 * sizeof(float) + 0 * sizeof(uchar)));
                copy(((char *) &z), ((char *) &z) + sizeof(float), buf + (2 * sizeof(float) + 0 * sizeof(uchar)));
                copy(((char *) &r), ((char *) &r) + sizeof(uchar), buf + (3 * sizeof(float) + 0 * sizeof(uchar)));
                copy(((char *) &g), ((char *) &g) + sizeof(uchar), buf + (3 * sizeof(float) + 1 * sizeof(uchar)));
                copy(((char *) &b), ((char *) &b) + sizeof(uchar), buf + (3 * sizeof(float) + 2 * sizeof(uchar)));

                ofsc.write(buf, sizeof(buf));
            } else {
                ostringstream vertexLine;
                vertexLine
                        << fixed
                        << x << " " << y << " " << z
                        << " " << (int) r << " " << (int) g << " " << (int) b;

                ofsc << vertexLine.str() << "\n";
            }

        }
        ofsc.flush();

#pragma omp parallel for default(none) shared(mesh, use_binary, use_texture, ofsc)
        for (size_t fIdx = 0; fIdx < mesh.faces.size(); fIdx++) {
            auto &face = mesh.faces[fIdx];

            u_char vertexCount = 3;
            int32_t vertexIdx[]{static_cast<int32_t>(face.x), static_cast<int32_t>(face.y),
                                static_cast<int32_t>(face.z)};
            u_char texcoordCount = (use_texture) ? vertexCount * 2 : 0;
            float texcoords[texcoordCount];
            if (use_texture) {
                for (size_t tIdx = 0; tIdx < vertexCount; tIdx++) {
                    auto &texcoord = mesh.faceTexcoords[(fIdx * vertexCount) + tIdx];
                    texcoords[tIdx * 2 + 0] = texcoord.x;
                    texcoords[tIdx * 2 + 1] = texcoord.y;
                }
            }


            if (use_binary) {
                char buf[(1 + ((use_texture) ? 1 : 0)) * sizeof(uchar) + (vertexCount) * sizeof(int32_t) +
                         (texcoordCount) * sizeof(float)];
                copy(((char *) &vertexCount) + 0, ((char *) &vertexCount) + sizeof(u_char),
                     buf + (0 * sizeof(uchar) + 0 * sizeof(int32_t) + 0 * sizeof(float)));
                for (int i = 0; i < vertexCount; i++) {
                    copy(((char *) &vertexIdx[i]) + 0, ((char *) &vertexIdx[i]) + sizeof(int32_t),
                         buf + (1 * sizeof(uchar) + (0 + i) * sizeof(int32_t) + 0 * sizeof(float)));
                }

                if (use_texture) {
                    copy(((char *) &texcoordCount) + 0, ((char *) &texcoordCount) + sizeof(float),
                         buf + (1 * sizeof(uchar) + (0 + vertexCount) * sizeof(int32_t) + 0 * sizeof(float)));
                    for (int i = 0; i < texcoordCount; i++) {
                        copy(((char *) &texcoords[i]) + 0, ((char *) &texcoords[i]) + sizeof(float),
                             buf + (2 * sizeof(uchar) + (0 + vertexCount) * sizeof(int32_t) + (0 + i) * sizeof(float)));
                    }
                }

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

                if (use_texture) {
                    faceLine << " " << (int) texcoordCount;
                    for (int i = 0; i < texcoordCount; i++) {
                        faceLine << " " << texcoords[i];
                    }
                }

#pragma omp critical
                {
                    ofsc << faceLine.str() << "\n";
                }
            }
        }

        ofsc.flush();
        ofsc.close();
    }

}
