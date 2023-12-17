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

#include "env.h"
#include <iostream>
#include <vector>

typedef std::vector<double> JOINT_CFG;
struct PointsObs;
struct AxesObs;

class LoCoBot {
  public:
    explicit LoCoBot(const Environment *env);
    std::vector<JOINT_CFG> planRRT(const JOINT_CFG &start,
                                   const JOINT_CFG &goal,
                                   double threshold = 0.25,
                                   int maxSamples = 3000,
                                   double goalBias = 0.05, int ppIters = 0);
    void initializePRM(double threshold = 0.25, int maxSamples = 3000);
    std::vector<JOINT_CFG> planPRM(const JOINT_CFG &start,
                                   const JOINT_CFG &goal);
    double calcPathCost(const std::vector<JOINT_CFG> &path) const;
    void printFullConfiguration() const;
    void printDynamics() const;
    void printPRMData() const;

  private:
    JOINT_CFG sampleRobotCfg() const;
    bool detectCollNode(const JOINT_CFG ang);
    bool detectCollEdge(const JOINT_CFG &ang1, const JOINT_CFG &ang2,
                        int numSteps = 5);
    void forwardKin(const JOINT_CFG &ang);
    void compCollBlockPoints(const JOINT_CFG &ang);
    double getNorm(const JOINT_CFG &q1, const JOINT_CFG &q2) const;

    const Environment *env;
    const int numJoints = 5;

    // Robot descriptor taken from URDF file
    // rpy xyz for each rigid link transform
    double rDesc[6][6] = {
        {0, 0, 0, 0.08, 0, 0.159}, // From robot base to joint1
        {0, 0, 0, 0, 0, 0.04125},         {0, 0, 0, 0.05, 0, 0.2},
        {0, 0, 0, 0.2002, 0, 0},          {0, 0, 0, 0.063, 0.0001, 0},
        {0, 0, 0, 0.106525, 0, 0.0050143} // From joint5 to end-eff. center
    };

    // Define the axis of rotation for each joint
    double axis[6][3]{{0, 0, 1}, {0, 1, 0},  {0, 1, 0},
                      {0, 1, 0}, {-1, 0, 0}, {0, 1, 0}};

    double TLink[6][4][4];  // Transforms for each link
    double TJoint[6][4][4]; // Transforms for each joint
    double TCurr[6][4][4];  // Coordinate frame of current

    double q[6] = {0, 0, 0, 0, 0, 0};
    const double qMin = -PI / 2;
    const double qMax = PI / 2;

    // Which joint frame the BB should be defined in
    double cIdx[4] = {1, 2, 3, 4};

    // rpy xyz poses of the robot arm blocks
    double cDesc[4][6] = {
        {0, 0, 0, 0, 0, 0.09},
        {0, 0, 0, 0.075, 0, 0},
        {0, 0, 0, 0.027, -0.012, 0},
        {0, 0, 0, 0.055, 0, 0.01},
    };

    // Dimensions of robot arm blocks (LWH of blocks)
    double cDim[4][3] = {
        {0.05, 0.05, 0.25},
        {0.25, 0.05, 0.05},
        {0.07, 0.076, 0.05},
        {0.11, 0.11, 0.07},
    };

    double TBlock[4][4][4];
    double TColl[4][4][4];
    double cPoints[4][9][3] = {};
    double cAxes[4][3][3] = {};

    std::vector<JOINT_CFG> prmVertices;
    std::vector<std::vector<int>> prmEdges;
    bool prmInitialized = false;
    double prmThreshold = -1.0;
};
