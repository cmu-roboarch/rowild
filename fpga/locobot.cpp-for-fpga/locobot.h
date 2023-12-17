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
#include "utils.h"

class LoCoBot {
 public:
     explicit LoCoBot(Environment *env);
     void planRRT(double (&start)[6], double (&goal)[6], double threshold, int maxSamples, double goalBias, int ppIters);

 private:
     void sampleRobotCfg(double cfg[6]);
     bool detectCollNode(double (&ang)[6]);
     bool detectCollEdge(double (&ang1)[6], double (&ang2)[6], int numSteps = 5);
     void forwardKin(double (&ang)[6]);
     void compCollBlockPoints(double (&ang)[6]);
     double getNorm(double (&q1)[6], double (&q2)[6]);

     Environment *env;
     int numJoints = 5;

     // Robot descriptor taken from URDF file
     // rpy xyz for each rigid link transform
     double rDesc[6][6] = {
         {0, 0, 0, 0.08, 0, 0.159},         // From robot base to joint1
         {0, 0, 0, 0, 0, 0.04125},
         {0, 0, 0, 0.05, 0, 0.2},
         {0, 0, 0, 0.2002, 0, 0},
         {0, 0, 0, 0.063, 0.0001, 0},
         {0, 0, 0, 0.106525, 0, 0.0050143}  // From joint5 to end-eff. center
     };

     // Define the axis of rotation for each joint
     double axis[6][3] {
         {0, 0, 1},
             {0, 1, 0},
             {0, 1, 0},
             {0, 1, 0},
             {-1, 0, 0},
             {0, 1, 0}
     };

     double TLink[6][4][4];     // Transforms for each link
     double TJoint[6][4][4];    // Transforms for each joint
     double TCurr[6][4][4];  // Coordinate frame of current

     double q[6] = {0, 0, 0, 0, 0, 0};
     double qMin = -PI/2;
     double qMax = PI/2;

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
};
