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

#include "frame.h"
#include "log.h"
#include <string>

#define MAX_NODES (100'000'000)

class Map {
  public:
    Map();
    ~Map();
    void fuse(Frame *f, double **intrinsicMatrix, double **T);
    void add(int num, double **nPoints, double **nNormals, double **nColors,
             double **T, bool **mergeBoard, int h, int w);
    void printMap(std::string header) const;

    int getNumPoints() const { return numPoints; }

    double **getPoints() const { return this->points; }

    double **getNormalMap() const { return this->normals; }

  private:
    int numPoints;
    bool initialized;
    double **points;
    double **normals;
    double **colors;
    double *weights;

    static constexpr const double distDiff = 0.03;
    static constexpr const double angleDiff = 5.0 * (PI / 180);
};
