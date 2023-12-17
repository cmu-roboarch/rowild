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

#include <climits>

class GreedyPlanner {
  public:
    GreedyPlanner(double gx, double gy) : goalX(gx), goalY(gy) {}
    int plan(double x, double y) const;

  private:
    double goalX, goalY;
    double legSteps[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
};

int GreedyPlanner::plan(double x, double y) const {

    auto getDistance = [this](double x, double y) {
        return (x - goalX) * (x - goalX) + (y - goalY) * (y - goalY);
    };

    int bestLeg = 0;
    double bestDistance = INT_MAX;
    for (int i = 0; i < 4; i++) {
        double dist =
            getDistance(x + this->legSteps[i][0], y + this->legSteps[i][1]);
        if (dist < bestDistance) {
            bestLeg = i;
            bestDistance = dist;
        }
    }

    return bestLeg;
}
