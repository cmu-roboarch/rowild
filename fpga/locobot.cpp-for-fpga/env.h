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

#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include "utils.h"

class Environment {
 public:
     explicit Environment() {
         for (int o = 0; o < 8; o++) {
             double nums[9] = {};
             for (int x = 3; x < 9; x++) nums[x] = .22 * (o+1);

             double H[4][4];
             rpyxyzToH(nums[0], nums[1], nums[2], nums[3], nums[4], nums[5], H);

             double corners[9][3];
             double dim[3] = {nums[6], nums[7], nums[8]};
             blockDescToBoundingBox(H, dim, corners);

             double axes[3][3];
             for (int i = 0; i < 3; i++) {
                 for (int j = 0; j < 3; j++) {
                     axes[i][j] = H[j][i];
                 }
             }

             for (int ii = 0; ii < 9; ii++) {
                for (int jj = 0; jj < 3; jj++) {
                    pointsObs[o][ii][jj] = corners[ii][jj];
                }
             }

             for (int ii = 0; ii < 3; ii++) {
                for (int jj = 0; jj < 3; jj++) {
                    axesObs[o][ii][jj] = axes[ii][jj];
                }
             }
         }
     }

     double (&getPointsObs())[8][9][3] {
         return this->pointsObs;
     }

     double (&getAxesObs())[8][3][3] {
         return this->axesObs;
     }


 private:
     double pointsObs[8][9][3];
     double axesObs[8][3][3];
};
