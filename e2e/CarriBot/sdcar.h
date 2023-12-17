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

#include "astar_bare.h"
#include "log.h"
#include "utils.h"
#include <string>
#include <vector>

class SelfDrivingCar : public AStar {
  public:
    SelfDrivingCar(std::string mapFile, int scaleMap, int robotL, int robotW,
                   std::string heuristic, double hWeight, bool deepening);
    ~SelfDrivingCar();
    bool isValid(const STATE &s) const;
    void updateMap(float robotX, float robotY, float angle, float measurement);

  private:
    std::vector<STATE> getNeighbors(const STATE &s) const;
    double getG(const STATE &prevS, const STATE &currS, double prevG,
                int dir) const;
    double getH(const STATE &s, const STATE &goal) const;

    bool isObstacle(const STATE &s) const;
    void readMap(std::string file, int scale);
    void showMap() const;

    std::function<double(STATE, STATE)> heuristicFunc;

    int mapX, mapY;
    int robotLength, robotWidth;
    int numThetas = 8; // Degree change: 360°/8 = 45°
    std::vector<double> sinThetas, cosThetas;
    std::vector<int> dX, dY, dTheta;
    float **env;
    float probFree = 0.3, probOccupied = 0.7;
    int numNeighbors;
};
