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

#include "OpenCvUtils.h"

#include <algorithm>
#include <sstream>
#include <opencv2/imgproc.hpp>

namespace photogrammetrie {

    string OpenCvUtils::toString(const cv::Mat &mat) {
        stringstream buffer;
        buffer << mat;
        return buffer.str();
    }

    string OpenCvUtils::toString(const cv::UMat &mat) {
        stringstream buffer;
        buffer << mat;
        return buffer.str();
    }

    vector<cv::Point2f> OpenCvUtils::toPoint2f(const vector<cv::KeyPoint> &keypoints) {
        vector<cv::Point2f> points;
        points.reserve(keypoints.size());

        for (auto &keypoint : keypoints) {
            points.push_back(keypoint.pt);
        }

        return points;
    }

    vector<cv::Point2d> OpenCvUtils::toPoint2d(const vector<cv::KeyPoint> &keypoints) {
        vector<cv::Point2d> points;

        for (auto &point2f : OpenCvUtils::toPoint2f(keypoints)) {
            points.push_back(cv::Point2d(point2f));
        }

        return points;
    }

    string OpenCvUtils::toString(const cv::Point3d &point) {
        return "Point3D(x=" + to_string(point.x) + ", y=" + to_string(point.y) + ", z=" + to_string(point.z) + ")";
    }

    string OpenCvUtils::toString(const cv::Point3f &point) {
        return "Point3D(x=" + to_string(point.x) + ", y=" + to_string(point.y) + ", z=" + to_string(point.z) + ")";
    }

    string OpenCvUtils::toString(const cv::Point2d &point) {
        return "Point2D(x=" + to_string(point.x) + ", y=" + to_string(point.y) + ")";
    }

    string OpenCvUtils::toString(const cv::Point2f &point) {
        return "Point2D(x=" + to_string(point.x) + ", y=" + to_string(point.y) + ")";
    }

    string OpenCvUtils::toString(const cv::Scalar &scalar) {
        return "Scalar(" + to_string(scalar[0]) + "," + to_string(scalar[1]) + "," + to_string(scalar[2]) + "," +
               to_string(scalar[3]) + ")";
    }

    string OpenCvUtils::toString(const cv::Size &size) {
        return "Size(" + to_string(size.width) + ", " + to_string(size.height) + ")";
    }

    bool OpenCvUtils::equals(const cv::DMatch &m1, const cv::DMatch &m2) {
        return equalsIgnoreDistance(m1, m2) && m1.distance == m2.distance;
    }

    bool OpenCvUtils::equalsIgnoreDistance(const cv::DMatch &m1, const cv::DMatch &m2) {
        return m1.trainIdx == m2.trainIdx && m1.queryIdx == m2.queryIdx && m1.imgIdx == m2.imgIdx;
    }

    void OpenCvUtils::scale(cv::InputArray src, cv::OutputArray dst, const double &scale, const int &interpolation,
                            const cv::Scalar &nullColor) {
        cv::Size srcSize = src.size();

        if (scale == 1.0) {
            src.copyTo(dst);
            return;
        }

        cv::Size dstSize((double) srcSize.width * scale, (double) srcSize.height * scale);
        cv::UMat scaled(dstSize, src.type(), nullColor);

        double factorX = (double) dstSize.width / (double) srcSize.width;
        double factorY = (double) dstSize.height / (double) srcSize.height;

        double factor = min(factorX, factorY);
        cv::Size dSize(dstSize.width * factor, dstSize.height * factor);
        cv::resize(src,
                   scaled(cv::Rect(abs(dstSize.width - dSize.width) / 2.0, abs(dstSize.height - dSize.height) / 2.0,
                                   dSize.width,
                                   dSize.height)), dSize);

        scaled.copyTo(dst);
    }

    void OpenCvUtils::scale(cv::InputArray src, cv::OutputArray dst, const cv::Size &dstSize, const int &interpolation,
                            const cv::Scalar &nullColor) {
        cv::Size srcSize = src.size();

        if (dstSize == srcSize) {
            src.copyTo(dst);
            return;
        }

        cv::UMat scaled(dstSize, src.type(), nullColor);

        double factorX = (double) dstSize.width / (double) srcSize.width;
        double factorY = (double) dstSize.height / (double) srcSize.height;

        double factor = min(factorX, factorY);
        cv::Size dSize(dstSize.width * factor, dstSize.height * factor);
        cv::resize(src,
                   scaled(cv::Rect(abs(dstSize.width - dSize.width) / 2.0, abs(dstSize.height - dSize.height) / 2.0,
                                   dSize.width,
                                   dSize.height)), dSize);

        scaled.copyTo(dst);
    }

    void OpenCvUtils::scalePoints(vector<cv::Point2d> &src, vector<cv::Point2d> &dst, const double &scale) {
        for (auto &srci : src) {
            dst.push_back((scale == 1.0) ? srci : srci * scale);
        }
    }

    cv::Rect OpenCvUtils::computeStereoMatcherROI(cv::Size &srcSize, const cv::Ptr<cv::StereoMatcher>& stereoMatcher) {
        int minDisparity = stereoMatcher->getMinDisparity();
        int numDisparities = stereoMatcher->getNumDisparities();
        int blockSize = stereoMatcher->getBlockSize();

        int halfBlockSize = blockSize / 2;
        int maxDisparity = minDisparity + numDisparities - 1;

        int xMin = maxDisparity + halfBlockSize;
        int xMax = srcSize.width + minDisparity - halfBlockSize;
        int yMin = halfBlockSize;
        int yMax = srcSize.height - halfBlockSize;

        cv::Rect roi(xMin, yMin, xMax - xMin, yMax - yMin);
        return roi;
    }

}