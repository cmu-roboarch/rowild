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
#include "slam.h"
#include <cmath>
#include <vector>

class EKFSLAM : public SLAM {
  public:
    EKFSLAM(int numLandmarks, double sX2, double sY2, double sA2, double sB2,
            double sR2);
    void motionUpdate(double dx, double dy, double dtheta);
    void measurementUpdate(const std::vector<double> &measurements);
    std::vector<double> getStatus() const override;

  private:
    std::vector<double> mu;
    std::vector<std::vector<double>> Sigma;
    double sigX2, sigY2, sigAlpha2, sigBeta2, sigR2;

    std::vector<std::vector<double>> createMatrix(int rows, int cols,
                                                  double initVal = 0.0);
    std::vector<std::vector<double>>
    transpose(const std::vector<std::vector<double>> &mat);
    std::vector<std::vector<double>>
    matMul(const std::vector<std::vector<double>> &mat1,
           const std::vector<std::vector<double>> &mat2);
    std::vector<double> matMul(const std::vector<std::vector<double>> &mat,
                               const std::vector<double> &vec);
    std::vector<std::vector<double>>
    matAdd(const std::vector<std::vector<double>> &A,
           const std::vector<std::vector<double>> &B);
    std::vector<std::vector<double>>
    matSub(const std::vector<std::vector<double>> &A,
           const std::vector<std::vector<double>> &B);
    std::vector<std::vector<double>>
    inverse(const std::vector<std::vector<double>> &mat);
};
