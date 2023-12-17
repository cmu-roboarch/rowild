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

#include "log.h"
#include "slam.h"
#include <vector>

struct PoseNode {
    double x, y, theta;
};

struct LandmarkNode {
    double x, y;
};

struct PosePoseConstraint {
    int pose1;
    int pose2;
    double dx, dy, dtheta;
};

struct PoseLandmarkConstraint {
    int pose;
    int landmark;
    double range;
    double bearing;
};

class GraphBasedSLAM : public SLAM {
  public:
    GraphBasedSLAM(int numLandmarks)
        : SLAM(numLandmarks), numLandmarks(numLandmarks) {}

    void motionUpdate(double x, double y, double theta) override {
        PoseNode newNode = {x, y, theta};
        poseGraph.push_back(newNode);

        // If it's not the first pose, create a relative motion constraint
        // between the last two poses
        if (poseGraph.size() > 1) {
            PosePoseConstraint c;
            c.pose1 = poseGraph.size() - 2;
            c.pose2 = poseGraph.size() - 1;
            c.dx = x;
            c.dy = y;
            c.dtheta = theta;
            posePoseConstraints.push_back(c);
        }
    }

    void measurementUpdate(const std::vector<double> &measurements) override {
        for (int i = 0; i < this->numLandmarks / 2; i++) {
            PoseLandmarkConstraint c;
            c.pose = poseGraph.size() - 1; // last pose
            c.landmark = i;
            c.range = measurements[2 * i];
            c.bearing = measurements[2 * i + 1];
            poseLandmarkConstraints.push_back(c);
        }
    }

    std::vector<double> getStatus() const override {
        std::vector<double> status;
        if (!poseGraph.empty()) {
            const PoseNode &lastPose = poseGraph.back();
            status.push_back(lastPose.x);
            status.push_back(lastPose.y);
        }

        for (int i = 0; i < this->numLandmarks / 2; i++) {
            double sumX = 0;
            double sumY = 0;
            int count = 0;
            for (const auto &c : poseLandmarkConstraints) {
                if (c.landmark == i) {
                    const PoseNode &p = poseGraph[c.pose];
                    sumX += p.x + c.range * cos(p.theta + c.bearing);
                    sumY += p.y + c.range * sin(p.theta + c.bearing);
                    count++;
                }
            }
            if (count > 0) {
                status.push_back(sumX / count);
                status.push_back(sumY / count);
            } else {
                status.push_back(0);
                status.push_back(0);
            }
        }

        return status;
    }

  private:
    std::vector<PoseNode> poseGraph;
    std::vector<LandmarkNode> landmarkGraph;
    std::vector<PosePoseConstraint> posePoseConstraints;
    std::vector<PoseLandmarkConstraint> poseLandmarkConstraints;

    int numLandmarks;
};
