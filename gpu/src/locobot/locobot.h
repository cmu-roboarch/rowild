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
#include "rowild_utils.h"
#include <vector>

typedef std::vector<float> JOINT_CFG;
struct PointsObs;
struct AxesObs;

class LoCoBot {
  public:
    explicit LoCoBot(const Environment *env);
    std::vector<JOINT_CFG> planRRT(const JOINT_CFG &start,
                                   const JOINT_CFG &goal,
                                   float threshold = 0.25,
                                   float goalBias = 0.05);

  private:
    bool detectCollNode(const JOINT_CFG ang);
    const Environment *env;

    float TLink[6][4][4];  // Transforms for each link
    float TJoint[6][4][4]; // Transforms for each joint
    float TCurr[6][4][4];  // Coordinate frame of current

    float q[6] = {0, 0, 0, 0, 0, 0};

    float TBlock[4][4][4];
    float TColl[4][4][4];
    float cPoints[4][9][3] = {};
    float cAxes[4][3][3] = {};
};
