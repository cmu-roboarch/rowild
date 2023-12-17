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

#include <random>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

typedef std::vector<double> STATE;

class MoveController {
  public:
    MoveController(double _dt, double _threshold, double _maxLinSpeed,
                   double _maxAngSpeed)
        : dt(_dt), threshold(_threshold), maxLinSpeed(_maxLinSpeed),
          maxAngSpeed(_maxAngSpeed) {}

    std::vector<std::pair<double, double>> getTrajectory(STATE start,
                                                         STATE goal);

  private:
    std::tuple<double, double, double>
    calcCtrlCmd(double dx, double dy, double theta, double thetaGoal) const;

    double Kp_rho = 9;    // Linear velocity gain
    double Kp_alpha = 15; // Angular velocity gain
    double Kp_beta = 3;   // Offset angular velocity gain
    double dt, threshold, maxLinSpeed, maxAngSpeed;
};
