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

class GPUDroneSolver;

class Drone {
  public:
    explicit Drone(std::string mapFile);
    ~Drone();
    std::vector<int> plan(int sx, int sy, int sz, int ex, int ey, int ez,
                          bool randomPoints, double hweight);

    int sx() const;
    int sy() const;
    int sz() const;
    int ex() const;
    int ey() const;
    int ez() const;
    int size() const;
    int mapX() const;
    int mapY() const;
    int mapZ() const;
    int toID(int x, int y, int z) const;
    void toXYZ(int id, int *x, int *y, int *z) const;
    bool isInRange(int x, int y, int z) const;
    const uint8_t *graph() const;

  private:
    MapReader *mr;
    GPUDroneSolver *gpuSolver;

    bool gpuSuccessful;
    float gpuOptimal;
    std::vector<int> gpuSolution;
};
