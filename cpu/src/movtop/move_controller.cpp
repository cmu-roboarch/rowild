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

#include "move_controller.h"
#include "rowild_utils.h"
#include <math.h>
#include <utility>

std::vector<std::pair<double, double>>
MoveController::getTrajectory(STATE start, STATE goal) {
    assert(start.size() == goal.size());
    assert(start.size() == 3);

    std::vector<std::pair<double, double>> outTraj;

    double x = start[0];
    double y = start[1];
    double theta = start[2];

    double dx = goal[0] - x;
    double dy = goal[1] - y;

    double rho = sqrt(dx * dx + dy * dy);

    while (rho > this->threshold) {
        outTraj.push_back({x, y});
        dx = goal[0] - x;
        dy = goal[1] - y;

        auto [r, v, w] = this->calcCtrlCmd(dx, dy, theta, goal[2]);
        if (std::abs(v) > this->maxLinSpeed) {
            v = (v < 0 ? -1 : 1) * this->maxLinSpeed;
        }

        if (std::abs(w) > this->maxAngSpeed) {
            w = (w < 0 ? -1 : 1) * this->maxAngSpeed;
        }

        theta += w * this->dt;
        x += v * cos(theta) * this->dt;
        y += v * sin(theta) * this->dt;
        rho = r;
    }

    return outTraj;
}

std::tuple<double, double, double>
MoveController::calcCtrlCmd(double dx, double dy, double theta,
                            double thetaGoal) const {
    double rho = sqrt(dx * dx + dy * dy);
    double alpha = fmod(atan2(dy, dx) - theta + PI, TWO_PI) - PI;
    double beta = fmod(thetaGoal - theta - alpha + PI, TWO_PI) - PI;

    double v = this->Kp_rho * rho;
    double w = this->Kp_alpha * alpha - this->Kp_beta * beta;

    if ((alpha > PI / 2) || (alpha < -PI / 2)) {
        v = -v;
    }

    return {rho, v, w};
}
