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

#include <string>
#include "transforms.h"
#include "frame.h"


class Map {
 public:
     Map();
     void fuse(Frame f, double (&intrinsicMatrix)[3][3], double (&T)[4][4]);
     void add(int num, double (&nPoints)[MAX_NODES][3], double (&nNormals)[MAX_NODES][3], double (&nColors)[MAX_NODES][3], double (&T)[4][4], bool (&mergeBoard)[FRAME_HEIGHT][FRAME_WIDTH], int h, int w);

     int getNumPoints() const {
         return numPoints;
     }

     double (&getPoints())[MAX_NODES][3] {
         return this->points;
     }

     double (&getNormalMap())[MAX_NODES][3] {
         return this->normals;
     }

 private:
     int numPoints;
     bool initialized;
     double points[MAX_NODES][3];
     double normals[MAX_NODES][3];
     double colors[MAX_NODES][3];
     double weights[MAX_NODES];

     double distDiff;    // Threshold to filter correspondences by positions
     double angleDiff;   // Threshold to filter correspondences by normals
};
