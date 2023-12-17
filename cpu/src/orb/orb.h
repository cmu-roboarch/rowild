/*
 * MIT License
 *
 * Copyright (c) 2023 Carnegie Mellon University
 *
 * This file is part of RoWild.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <list>
#include <opencv2/opencv.hpp>
#include <vector>

class MapPoint {
  public:
    cv::Point3f position;
    cv::Mat descriptor;
};

class KeyFrame {
  public:
    cv::Mat image;
    cv::Mat rotation, translation;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    std::vector<MapPoint *> map_points;
};

class SimpleORBSLAM {
  public:
    SimpleORBSLAM(cv::Mat &K);
    void processImage(const cv::Mat &image);

  private:
    cv::Ptr<cv::ORB> orb_detector;
    cv::BFMatcher matcher;
    std::list<KeyFrame *> keyframes;
    std::list<MapPoint *> map_points;
    cv::Mat camera_matrix;

    void track(const cv::Mat &image, KeyFrame *&current_frame);
    void mapManagement(KeyFrame *current_frame);
    cv::Point3f triangulate(const cv::Point2f &pt1, const cv::Point2f &pt2,
                            const cv::Mat &P1, const cv::Mat &P2);
};

SimpleORBSLAM::SimpleORBSLAM(cv::Mat &K) : camera_matrix(K) {
    orb_detector = cv::ORB::create();
    matcher = cv::BFMatcher(cv::NORM_HAMMING, true);
}

void SimpleORBSLAM::processImage(const cv::Mat &image) {
    KeyFrame *current_frame = new KeyFrame;
    track(image, current_frame);
    mapManagement(current_frame);
    keyframes.push_back(current_frame);
}

void SimpleORBSLAM::track(const cv::Mat &image, KeyFrame *&current_frame) {
    current_frame->image = image.clone();
    orb_detector->detectAndCompute(image, cv::noArray(),
                                   current_frame->keypoints,
                                   current_frame->descriptors);

    if (!keyframes.empty()) {
        std::vector<cv::DMatch> matches;
        matcher.match(keyframes.back()->descriptors, current_frame->descriptors,
                      matches);

        std::vector<cv::Point2f> prev_points, curr_points;

        for (auto &match : matches) {
            prev_points.push_back(
                keyframes.back()->keypoints[match.queryIdx].pt);
            curr_points.push_back(current_frame->keypoints[match.trainIdx].pt);
        }

        cv::Mat inliers_mask;
        cv::Mat E =
            cv::findEssentialMat(prev_points, curr_points, camera_matrix,
                                 cv::RANSAC, 0.999, 1.0, inliers_mask);
        cv::recoverPose(E, prev_points, curr_points, camera_matrix,
                        current_frame->rotation, current_frame->translation,
                        inliers_mask);
    }
}

void SimpleORBSLAM::mapManagement(KeyFrame *current_frame) {
    if (keyframes.size() >= 1) {
        KeyFrame *prev_frame = keyframes.back();

        cv::Mat eyeMat = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat zeroMat = cv::Mat::zeros(3, 1, CV_64F);
        cv::Mat P1_concat;
        cv::hconcat(eyeMat, zeroMat, P1_concat);
        cv::Mat P1 = camera_matrix * P1_concat;

        cv::Mat R_T_concat;
        cv::hconcat(current_frame->rotation, current_frame->translation,
                    R_T_concat);
        cv::Mat P2 = camera_matrix * R_T_concat;

        for (size_t i = 0; i < prev_frame->keypoints.size(); i++) {
            cv::Point3f pt3D =
                triangulate(prev_frame->keypoints[i].pt,
                            current_frame->keypoints[i].pt, P1, P2);
            MapPoint *mpt = new MapPoint;
            mpt->position = pt3D;
            map_points.push_back(mpt);
        }
    }
}

cv::Point3f SimpleORBSLAM::triangulate(const cv::Point2f &pt1,
                                       const cv::Point2f &pt2,
                                       const cv::Mat &P1, const cv::Mat &P2) {
    cv::Mat A(4, 4, CV_64F);

    A.row(0) = pt1.x * P1.row(2) - P1.row(0);
    A.row(1) = pt1.y * P1.row(2) - P1.row(1);
    A.row(2) = pt2.x * P2.row(2) - P2.row(0);
    A.row(3) = pt2.y * P2.row(2) - P2.row(1);

    cv::Mat u, w, vt;
    cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    cv::Mat X = vt.row(3).t();

    cv::Point3f triangulated_point;
    triangulated_point.x = X.at<float>(0) / X.at<float>(3);
    triangulated_point.y = X.at<float>(1) / X.at<float>(3);
    triangulated_point.z = X.at<float>(2) / X.at<float>(3);

    return triangulated_point;
}
