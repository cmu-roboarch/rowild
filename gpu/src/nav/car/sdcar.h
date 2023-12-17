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
#include "mapreader.h"
#include <string>
#include <vector>

using std::string;

class GPUCarRobotSolver;

class CarRobot {
  public:
    explicit CarRobot(std::string mapFile);
    ~CarRobot();
    std::vector<int> plan(int sx, int sy, double sthetaVal, int ex, int ey,
                          double ethetaVal, bool randomPoints, double hweight);

    int sx() const;
    int sy() const;
    int sthetaIdx() const;
    int ex() const;
    int ey() const;
    int ethetaIdx() const;
    int size() const;
    int mapX() const;
    int mapY() const;
    int toID(int x, int y, int thetaIdx) const;
    void toXYTheta(int id, int *x, int *y, int *thetaIdx) const;
    const uint8_t *graph() const;
    int getRobotLength() const;
    int getRobotWidth() const;
    int getNumThetaIndices() const;

  private:
    bool isStateFree(int x, int y, int thetaIdx) const;
    int convertThetaValToIdx(double thetaVal) const;
    double convertThetaIdxToVal(int thetaIdx) const;
    void setRandomStartEndPoints();
    void setStartEndPoints(int sx, int sy, double sthetaVal, int ex, int ey,
                           double ethetaVal);
    MapReader *mr;
    GPUCarRobotSolver *gpuSolver;

    bool gpuSuccessful;
    float gpuOptimal;
    std::vector<int> gpuSolution;
    int startX, startY, startThetaIdx;
    int endX, endY, endThetaIdx;

    std::vector<double> sinThetas, cosThetas;
    const int robotLength = 10, robotWidth = 4;
    const int numThetaIndices = 8;
};
