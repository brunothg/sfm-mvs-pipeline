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

#include "Scene.h"

#include <algorithm>
#include <utility>
#include "photogrammetrie/util/OpenCvUtils.h"

namespace photogrammetrie {

    ShotMatches::ShotMatches(const shared_ptr<CameraShot> &left, const shared_ptr<CameraShot> &right) : left(left),
                                                                                                        right(right) {
        if (this->left == nullptr || this->right == nullptr) {
            throw invalid_argument("Die Aufnahmen dürfen nicht null sein.");
        }
    }

    const shared_ptr<CameraShot> &ShotMatches::getLeft() const {
        return left;
    }

    const shared_ptr<CameraShot> &ShotMatches::getRight() const {
        return right;
    }

    bool ShotMatches::isRecovered() const {
        return left->isRecovered() && right->isRecovered();
    }

    const vector<cv::DMatch> &ShotMatches::getMatches() const {
        return matches;
    }

    void ShotMatches::setMatches(const vector<cv::DMatch> &newMatches) {
        matches = newMatches;
    }

    void ShotMatches::addMatch(const cv::DMatch &match) {
        matches.push_back(match);
    }

    void ShotMatches::alignFeatures(Features &alignedFeaturesLeft, Features &alignedFeaturesRight) const {
        auto &featureMatches = getMatches();
        auto &featuresLeft = getLeft()->getFeatures();
        auto &featuresRight = getRight()->getFeatures();

        // Matches sortieren/ausrichten
        for (auto &match : featureMatches) {
            alignedFeaturesLeft.keypoints.push_back(featuresLeft.keypoints[match.queryIdx]);
            alignedFeaturesLeft.descriptors.push_back(featuresLeft.descriptors.row(match.queryIdx));

            alignedFeaturesRight.keypoints.push_back(featuresRight.keypoints[match.trainIdx]);
            alignedFeaturesRight.descriptors.push_back(featuresRight.descriptors.row(match.trainIdx));
        }
    }

    void ShotMatches::getFeaturePoints(vector<pair<cv::Point2d, cv::Point2d>> &featurePoints) const {
        Features alignedFeaturesLeft, alignedFeaturesRight;
        alignFeatures(alignedFeaturesLeft, alignedFeaturesRight);

        for (size_t i = 0; i < alignedFeaturesLeft.keypoints.size(); i++) {
            featurePoints.emplace_back(alignedFeaturesLeft.keypoints[i].pt, alignedFeaturesRight.keypoints[i].pt);
        }
    }

    double ShotMatches::getHomographyInlierRatio() const {
        return homographyInlierRatio;
    }

    void ShotMatches::setHomographyInlierRatio(double newHomographyInlierRatio) {
        homographyInlierRatio = newHomographyInlierRatio;
    }

    string ShotMatches::toString() const {
        return "ShotMatch( \n" + left->toString() + "\n :: \n" + right->toString() + "\n, recovered: " +
               to_string(isRecovered()) + ", matches: " +
               to_string(matches.size()) + ", homographyInlierRatio: " + to_string(homographyInlierRatio) + ")";
    }

    PointcloudElement::PointcloudElement(cv::Point3d coordinates) : coordinates(std::move(coordinates)),
                                                                    color(cv::Scalar(-1)) {}

    void PointcloudElement::setRgba(const cv::Scalar &newColor) {
        if (newColor[0] != -1 &&
            (newColor[0] < 0 || newColor[1] < 0 || newColor[2] < 0 || newColor[3] < 0 || newColor[0] > 255 ||
             newColor[1] > 255 || newColor[2] > 255 || newColor[3] > 255)) {
            throw invalid_argument("Ungültige Farbwerte.");
        }

        color = newColor;
    }

    bool PointcloudElement::isRgbaCalculated() const {
        return color[0] != -1;
    }

    cv::Scalar PointcloudElement::getRgba() {
        if (isRgbaCalculated()) {
            return color;
        }

        long r = 0, g = 0, b = 0;
        long counter = 0;

        for (auto &origin : origins) {
            auto &leftFeatures = origin->getMatch()->getLeft()->getFeatures();
            auto &rightFeatures = origin->getMatch()->getRight()->getFeatures();

            auto &leftPoint = origin->getFeatures().first;
            auto &rightPoint = origin->getFeatures().second;

            auto leftImage = origin->getMatch()->getLeft()->loadMImage(cv::IMREAD_COLOR);
            auto rightImage = origin->getMatch()->getRight()->loadMImage(cv::IMREAD_COLOR);

            auto leftPixel = leftImage.at<cv::Vec3b>(leftPoint);
            auto rightPixel = rightImage.at<cv::Vec3b>(rightPoint);

            b += leftPixel[0] + rightPixel[0];
            g += leftPixel[1] + rightPixel[1];
            r += leftPixel[2] + rightPixel[2];
            counter += 2;

            break; // Dauert sonst viel zu lange und bringt nicht wirklich was
        }

        if (counter > 0) {
            r /= counter;
            g /= counter;
            b /= counter;
        }

        color = cv::Scalar(r, g, b, 255);
        return color;
    }

    void PointcloudElement::getOriginShots(vector<shared_ptr<CameraShot>> &originShots) const {
        set<shared_ptr<CameraShot>> distinctShots;
        for (auto &origin : origins) {
            distinctShots.insert(origin->getMatch()->getLeft());
            distinctShots.insert(origin->getMatch()->getRight());
        }
        originShots.reserve(distinctShots.size());
        originShots.insert(originShots.end(), distinctShots.begin(), distinctShots.end());
    }

    void PointcloudElement::getOriginPoints(vector<pair<shared_ptr<CameraShot>, cv::Point2d>> &originPoints) const {
        for (auto &origin : origins) {
            auto &leftShot = origin->getMatch()->getLeft();
            auto &rightShot = origin->getMatch()->getRight();

            auto &leftPoint = origin->getFeatures().first;
            auto &rightPoint = origin->getFeatures().second;

            auto found = find_if(originPoints.begin(), originPoints.end(),
                                 [&leftShot, &leftPoint](const pair<shared_ptr<CameraShot>, cv::Point2d> &i) {
                                     return i.first == leftShot && (i.second == cv::Point2d(leftPoint));
                                 });
            if (found == originPoints.end()) {
                originPoints.emplace_back(leftShot, cv::Point2d(leftPoint));
            }
            found = find_if(originPoints.begin(), originPoints.end(),
                            [&rightShot, &rightPoint](pair<shared_ptr<CameraShot>, cv::Point2d> &i) {
                                return i.first == rightShot && (i.second == cv::Point2d(rightPoint));
                            });
            if (found == originPoints.end()) {
                originPoints.emplace_back(rightShot, cv::Point2d(rightPoint));
            }
        }
    }

    const cv::Point3d &PointcloudElement::getCoordinates() const {
        return coordinates;
    }

    void PointcloudElement::setCoordinates(const cv::Point3d &newCoordinates) {
        coordinates = newCoordinates;
    }

    const vector<shared_ptr<PointcloudElement::Origin>> &PointcloudElement::getOrigins() const {
        return origins;
    }

    void PointcloudElement::setOrigins(const vector<shared_ptr<PointcloudElement::Origin>> &newOrigins) {
        origins = newOrigins;
    }

    shared_ptr<PointcloudElement::Origin>
    PointcloudElement::addOrigin(const shared_ptr<PointcloudElement::Origin> &origin) {
        auto existingOriginIt = find_if(origins.begin(), origins.end(),
                                        [&origin](shared_ptr<PointcloudElement::Origin> &i) {
                                            auto &iFeatures = i->getFeatures();
                                            auto &originFeatures = origin->getFeatures();
                                            return i->getMatch() == origin->getMatch() &&
                                                   ((iFeatures.first == originFeatures.first &&
                                                     iFeatures.second == originFeatures.second) ||
                                                    (iFeatures.first == originFeatures.second &&
                                                     iFeatures.second == originFeatures.first));
                                        });

        if (existingOriginIt == origins.end()) {
            origins.push_back(origin);
            return origin;
        } else {
            return *existingOriginIt;
        }
    }

    string PointcloudElement::toString() const {
        return "PointcloudElement(Position: " + OpenCvUtils::toString(coordinates) + ")";
    }

    PointcloudElement::Origin::Origin(const shared_ptr<ShotMatches> &match,
                                      const pair<cv::Point2d, cv::Point2d> &features) : match(match),
                                                                                        features(
                                                                                                features) {}

    const shared_ptr<ShotMatches> &PointcloudElement::Origin::getMatch() const {
        return match;
    }

    const pair<cv::Point2d, cv::Point2d> &PointcloudElement::Origin::getFeatures() const {
        return features;
    }


    ShotMatches3d2d::ShotMatches3d2d(const shared_ptr<CameraShot> &shot) : shot(shot) {
        if (this->shot == nullptr) {
            throw invalid_argument("Die Aufnahme darf nicht null sein.");
        }
    }

    const shared_ptr<CameraShot> &ShotMatches3d2d::getShot() const {
        return shot;
    }

    const vector<ShotMatches3d2d::Match> &ShotMatches3d2d::getMatches() const {
        return matches;
    }

    void ShotMatches3d2d::setMatches(const vector<ShotMatches3d2d::Match> &newMatches) {
        matches = newMatches;
    }

    void ShotMatches3d2d::addMatch(ShotMatches3d2d::Match &match) {
        matches.push_back(match);
    }

    void ShotMatches3d2d::getDistinct3d2dPoints(vector<cv::Point3d> &points3d, vector<cv::Point2d> &points2d) const {
        vector<pair<cv::Point3d, cv::Point2d>> points;
        for (auto &match : matches) {
            if (find_if(points.begin(), points.end(), [&match](pair<cv::Point3d, cv::Point2d> &i) {
                return i.first == match.getPoint3D()->getCoordinates() && i.second == match.getPoint2D();
            }) == points.end()) {
                points.emplace_back(match.getPoint3D()->getCoordinates(), match.getPoint2D());
            }
        }

        for (auto &point : points) {
            points3d.push_back(point.first);
            points2d.push_back(point.second);
        }
    }

    void ShotMatches3d2d::getDistinctShotMatches(vector<shared_ptr<ShotMatches>> &shotMatches) const {
        for (auto &match : matches) {
            if (find_if(shotMatches.begin(), shotMatches.end(), [&match](shared_ptr<ShotMatches> &i) {
                return i == match.getShotMatch();
            }) == shotMatches.end()) {
                shotMatches.push_back(match.getShotMatch());
            }
        }
    }

    ShotMatches3d2d::Match::Match(const shared_ptr<PointcloudElement> &point3D, cv::Point2d point2D,
                                  const shared_ptr<ShotMatches> &shotMatch) : point3d(point3D),
                                                                              point2d(std::move(point2D)),
                                                                              shotMatch(shotMatch) {
        if (this->point3d == nullptr || this->shotMatch == nullptr) {
            throw invalid_argument("Nullptr nicht erlaubt für point3d oder shotMatch.");
        }
    }

    const shared_ptr<PointcloudElement> &ShotMatches3d2d::Match::getPoint3D() const {
        return point3d;
    }

    const cv::Point2d &ShotMatches3d2d::Match::getPoint2D() const {
        return point2d;
    }

    const shared_ptr<ShotMatches> &ShotMatches3d2d::Match::getShotMatch() const {
        return shotMatch;
    }


    Scene::Scene() = default;

    Scene::~Scene() = default;

    void Scene::addCameraShot(const shared_ptr<CameraShot> &shot) {
        auto existingShotIt = find(shots.begin(), shots.end(), shot);
        if (existingShotIt == shots.end()) {
            shots.push_back(shot);
        }
    }

    const vector<shared_ptr<CameraShot>> &Scene::getShots() const {
        return shots;
    }

    shared_ptr<ShotMatches> Scene::addShotMatches(const shared_ptr<ShotMatches> &matches) {
        auto existingMatchIt = find_if(shotMatches.begin(), shotMatches.end(), [&matches](shared_ptr<ShotMatches> &i) {
            return i->getLeft() == matches->getLeft() && i->getRight() == matches->getRight();
        });

        if (existingMatchIt == shotMatches.end()) {
            shotMatches.push_back(matches);
            return matches;
        } else {
            return *existingMatchIt;
        }
    }

    const vector<shared_ptr<ShotMatches>> &Scene::getShotMatches() const {
        return shotMatches;
    }

    void Scene::addPointcloudElement(const shared_ptr<PointcloudElement> &element) {
        pointcloud.push_back(element);
    }

    const vector<shared_ptr<PointcloudElement>> &Scene::getPointcloud() const {
        return pointcloud;
    }

    void Scene::clearPointcloud() {
        pointcloud.clear();
    }

    vector<shared_ptr<ICamera>> Scene::getCameras() const {
        vector<shared_ptr<ICamera>> cameras;

        for (auto &shot : getShots()) {
            auto &camera = shot->getCamera();
            if (find(cameras.begin(), cameras.end(), camera) == cameras.end()) {
                cameras.push_back(camera);
            }
        }

        return cameras;
    }

    void Scene::find3d2dMatches(ShotMatches3d2d &matches) const {
        auto &shot = matches.getShot();

        auto &existingShotMatches = getShotMatches();
        // Suche in allen bekannten 3D Punkten der Punktwolke
#pragma omp parallel for default(none) shared(shot, existingShotMatches, matches) schedule(dynamic)
        for (auto &pce : getPointcloud()) {
            vector<pair<shared_ptr<CameraShot>, cv::Point2d>> originPoints;
            pce->getOriginPoints(originPoints);

            // Suche in allen Ursprüngen des 3D-Punktes nach Aufnahmen mit 2D-Übereinstimmungen
            for (auto &originPoint : originPoints) {
                auto &originShot = originPoint.first;
                auto &origin2dPoint = originPoint.second;

                // Hierzu kann es keine ShotMatches geben
                if (shot == originShot) {
                    continue;
                }

                // Ansonsten durchprobieren, ob es passende ShotMatches gibt
                auto existingShotMatchIt = find_if(existingShotMatches.begin(), existingShotMatches.end(),
                                                   [&shot, &originShot](const shared_ptr<ShotMatches> &i) {
                                                       return (i->getLeft() == shot && i->getRight() == originShot) ||
                                                              (i->getRight() == shot && i->getLeft() == originShot);
                                                   });
                if (existingShotMatchIt != existingShotMatches.end()) {
                    auto &existingShotMatch = *existingShotMatchIt;
                    auto &existingShotMatchMatches = existingShotMatch->getMatches();
                    auto existingShotMatchMatchIt = find_if(existingShotMatchMatches.begin(),
                                                            existingShotMatchMatches.end(),
                                                            [&existingShotMatch, &originShot, &origin2dPoint](
                                                                    const cv::DMatch &i) {
                                                                return cv::Point2d(
                                                                        ((existingShotMatch->getLeft() == originShot)
                                                                         ? existingShotMatch->getLeft()->getFeatures().keypoints[i.queryIdx]
                                                                         : existingShotMatch->getRight()->getFeatures().keypoints[i.trainIdx]).pt) ==
                                                                       origin2dPoint;
                                                            });

                    if (existingShotMatchMatchIt != existingShotMatchMatches.end()) {
                        auto &existingShotMatchMatch = *existingShotMatchMatchIt;
                        auto p2d = ((existingShotMatch->getLeft() == shot)
                                    ? existingShotMatch->getLeft()->getFeatures().keypoints[existingShotMatchMatch.queryIdx]
                                    : existingShotMatch->getRight()->getFeatures().keypoints[existingShotMatchMatch.trainIdx]).pt;
                        ShotMatches3d2d::Match new2d3dMatch(pce, p2d, existingShotMatch);
#pragma omp critical
                        {
                            matches.addMatch(new2d3dMatch);
                        }
                        continue;
                    }
                }
            }
        }
    }

    void Scene::mergePointcloud(const double &pointMergeDistance) {
        vector<shared_ptr<PointcloudElement>> tmpPointcloud = getPointcloud();
        pointcloud.clear();

        for (auto &pce : tmpPointcloud) {
            mergePointcloudElement(pce, pointMergeDistance);
        }
    }

    shared_ptr<PointcloudElement>
    Scene::mergePointcloudElement(const shared_ptr<PointcloudElement> &element, const double &pointMergeDistance) {
        shared_ptr<PointcloudElement> bestMergeCandidate;
        double bestMergeDistance = -1;

#pragma omp parallel for default(none) shared(bestMergeCandidate, bestMergeDistance, pointMergeDistance, element)
        for (auto &mergeCandidate : getPointcloud()) {
            const auto pointdistance = cv::norm(mergeCandidate->getCoordinates() - element->getCoordinates());
            if (pointdistance > pointMergeDistance) {
                continue;
            } else {
#pragma omp critical
                {
                    if (bestMergeDistance == -1 || bestMergeDistance > pointdistance) {
                        bestMergeDistance = pointdistance;
                        bestMergeCandidate = mergeCandidate;
                    }
                }
            }
        }

        // Mergen oder neu anlegen
        if (bestMergeCandidate == nullptr) {
            addPointcloudElement(element);
            bestMergeCandidate = element;
        } else {
            Scene::merge(bestMergeCandidate, element);
            for (auto &origin : element->getOrigins()) {
                bestMergeCandidate->addOrigin(origin);
            }
        }

        return bestMergeCandidate;
    }

    shared_ptr<PointcloudElement>
    Scene::mergePointcloudElement3d2d(const shared_ptr<PointcloudElement> &element, const double &pointMergeDistance,
                                      const double &featureMergeDistance) {
        auto &possibleShotMatches = getShotMatches();

        shared_ptr<PointcloudElement> bestMergeCandidate;
        double bestMergeDistance = -1;

        vector<shared_ptr<PointcloudElement>> mergeCandidates;
#pragma omp declare reduction (vec_push : std::vector<shared_ptr<PointcloudElement>> : omp_out.insert(omp_out.end(), omp_in.begin(), omp_in.end()))
#pragma omp parallel for default(none) shared(element, pointMergeDistance) reduction(vec_push: mergeCandidates)
        for (auto &mergeCandidate : getPointcloud()) {
            const auto pointdistance = cv::norm(mergeCandidate->getCoordinates() - element->getCoordinates());
            if (pointdistance > pointMergeDistance) {
                continue;
            }
            mergeCandidates.push_back(mergeCandidate);
        }


        for (auto &mergeCandidate : mergeCandidates) {
            const auto pointdistance = cv::norm(mergeCandidate->getCoordinates() - element->getCoordinates());

            // Suche 2D Übereinstimmung zwischen Sichten des Elements und der Merge-Kandidaten
            vector<pair<shared_ptr<CameraShot>, cv::Point2d>> elementOriginPoints, candidateOriginPoints;
            element->getOriginPoints(elementOriginPoints);
            mergeCandidate->getOriginPoints(candidateOriginPoints);

#pragma omp parallel for default(none) shared(mergeCandidate, pointdistance, elementOriginPoints, candidateOriginPoints, possibleShotMatches, bestMergeCandidate, bestMergeDistance, pointMergeDistance, featureMergeDistance, element) collapse(2) schedule(guided)
            for (auto &elementOriginPoint : elementOriginPoints) {
                for (auto &candidateOriginPoint : candidateOriginPoints) {
                    cv::Point2d leftPoint, rightPoint;
                    shared_ptr<ShotMatches> shotMatchOrigin;

                    // Suche passenden ShotMatch
                    for (auto &shotMatch : getShotMatches()) {
                        if (shotMatch->getLeft() == elementOriginPoint.first
                            && shotMatch->getRight() == candidateOriginPoint.first) {
                            shotMatchOrigin = shotMatch;
                            leftPoint = elementOriginPoint.second;
                            rightPoint = candidateOriginPoint.second;
                            break;
                        } else if (shotMatch->getLeft() == candidateOriginPoint.first
                                   && shotMatch->getRight() == elementOriginPoint.first) {
                            shotMatchOrigin = shotMatch;
                            leftPoint = candidateOriginPoint.second;
                            rightPoint = elementOriginPoint.second;
                            break;
                        }
                    }

                    // Passenden gefunden -> prüfe ob Merkmale übereinstimmen
                    if (shotMatchOrigin != nullptr) {
                        for (auto &originFeatureMatch : shotMatchOrigin->getMatches()) {
                            if (cv::Point2d(
                                    shotMatchOrigin->getLeft()->getFeatures().keypoints[originFeatureMatch.queryIdx].pt) ==
                                leftPoint
                                && cv::Point2d(
                                    shotMatchOrigin->getRight()->getFeatures().keypoints[originFeatureMatch.trainIdx].pt) ==
                                   rightPoint
                                && abs(originFeatureMatch.distance) <= featureMergeDistance) {

#pragma omp critical
                                {
                                    if (bestMergeDistance == -1 || bestMergeDistance > pointdistance) {
                                        bestMergeDistance = pointdistance;
                                        bestMergeCandidate = mergeCandidate;
                                    }
                                }

                                break;
                            }
                        }
                    }

                }
            }


        }


        // Mergen oder neu anlegen
        if (bestMergeCandidate == nullptr) {
            addPointcloudElement(element);
            bestMergeCandidate = element;
        } else {
            Scene::merge(bestMergeCandidate, element);
        }

        return bestMergeCandidate;
    }

    void Scene::merge(const shared_ptr<PointcloudElement> &parent, const shared_ptr<PointcloudElement> &child) {
        for (auto &origin : child->getOrigins()) {
            parent->addOrigin(origin);
        }
    }

    void Scene::colorizePointcloud(const cv::Scalar &defaultColor, const bool overwrite) {
        if (overwrite) {
#pragma omp parallel for default(none)
            for (auto &pce : pointcloud) {
                if (pce->isRgbaCalculated()) {
                    continue;
                }
                pce->setRgba(cv::Scalar(-1));
            }
        }

        for (auto &shot : shots) {
            auto image = shot->loadMImage(cv::IMREAD_COLOR);

#pragma omp parallel for default(none) shared(overwrite, shot, image)
            for (auto &pce : pointcloud) {
                if (!overwrite && pce->isRgbaCalculated()) {
                    continue;
                }

                auto origins = pce->getOrigins();
                auto originI = find_if(origins.begin(), origins.end(),
                                       [&shot](shared_ptr<PointcloudElement::Origin> &origin) {
                                           return origin->getMatch()->getLeft() == shot ||
                                                  origin->getMatch()->getRight() == shot;
                                       });
                if (originI == origins.end()) {
                    continue;
                }

                auto &point = ((*originI)->getMatch()->getLeft() == shot) ? (*originI)->getFeatures().first
                                                                          : (*originI)->getFeatures().second;
                auto pixel = image.at<cv::Vec3b>(point);

#pragma omp critical
                {
                    pce->setRgba(cv::Scalar(pixel[2], pixel[1], pixel[0], 255));
                }
            }
        }

#pragma omp parallel for default(none) shared(defaultColor)
        for (auto &pce : pointcloud) {
            if (pce->isRgbaCalculated()) {
                continue;
            }
            pce->setRgba(defaultColor);
        }
    }

    string Scene::toString() const {
        string s;

        s = "Cameras: \n";
        auto cameras = getCameras();
        for (auto &camera : cameras) {
            s += camera->toString() + "\n\n";
        }

        s += "\n\nShots: \n";
        auto &cameraShots = getShots();
        for (auto &shot : cameraShots) {
            s += shot->toString() + "\n\n";
        }

        s += "\n\n3D-Points: \n";
        auto &points = getPointcloud();
        for (auto &point : points) {
            s += point->toString() + "\n\n";
        }

        return s.substr(0, s.length() - 2);
    }

}