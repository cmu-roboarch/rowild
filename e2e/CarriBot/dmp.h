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

#include "utils.h"
#include <cmath>
#include <vector>

class DmpController {
  private:
    float x, y, theta;

    const float stepSize = 1.0;
    const float turnAngle = PI / 4;

  public:
    DmpController(float initX, float initY, float initTheta)
        : x(initX), y(initY), theta(initTheta) {}

    void executeCommand(int command) {
        switch (command) {
        case 0:
            turnRight();
            break;
        case 1:
            turnLeft();
            break;
        case 2:
            moveForward();
            break;
        case 3:
            moveBackward();
            break;
        default:
            panic("Invalid command");
        }
    }

    std::vector<float> getState() { return {x, y, theta}; }

  private:
    void moveForward() {
        x += stepSize * cos(theta);
        y += stepSize * sin(theta);
    }

    void moveBackward() {
        x -= stepSize * cos(theta);
        y -= stepSize * sin(theta);
    }

    void turnRight() { theta += turnAngle; }

    void turnLeft() { theta -= turnAngle; }
};
