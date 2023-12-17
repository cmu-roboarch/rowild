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

#include "legged_robot.h"
#include "GPU-solver.hpp"
#include "log.h"
#include <fstream>
#include <iostream>

LeggedRobot::LeggedRobot(std::string mapFile) {
    this->gpuSolver = new GPULeggedRobotSolver(this);
    this->gpuSuccessful = false;

    this->mr = new MapReader();
    this->mr->readMap(mapFile);
}

LeggedRobot::~LeggedRobot() {
    delete this->gpuSolver;
    delete this->mr;
}

std::vector<int> LeggedRobot::plan(int sx, int sy, int ex, int ey,
                                   bool randomPoints, double heuristicWeight) {
    if (randomPoints) {
        this->mr->setRandomStartEndPoints();
    } else {
        this->mr->setStartEndPoints(sx, sy, ex, ey);
    }

    this->gpuSolver->initialize(heuristicWeight);
    this->gpuSolution.clear();
    this->gpuSuccessful = gpuSolver->solve();
    this->gpuSolver->getSolution(&gpuOptimal, &gpuSolution);
    return this->gpuSolution;
}

int LeggedRobot::sx() const { return this->mr->getStartX(); }

int LeggedRobot::sy() const { return this->mr->getStartY(); }

int LeggedRobot::ex() const { return this->mr->getEndX(); }

int LeggedRobot::ey() const { return this->mr->getEndY(); }

int LeggedRobot::size() const {
    return this->mr->getMapHeight() * this->mr->getMapWidth();
}

int LeggedRobot::width() const { return this->mr->getMapWidth(); }

int LeggedRobot::height() const { return this->mr->getMapHeight(); }

int LeggedRobot::toID(int x, int y) const { return x * this->width() + y; }

void LeggedRobot::toXY(int id, int *x, int *y) const {
    *x = id / this->width();
    *y = id % this->width();
}

bool LeggedRobot::isInRange(int x, int y) const {
    return 0 <= x && x < this->height() && 0 <= y && y < this->width();
}

const uint8_t *LeggedRobot::graph() const { return this->mr->getGraph(); }
