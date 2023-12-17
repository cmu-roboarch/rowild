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

#include "drone.h"
#include "GPU-solver.hpp"
#include "log.h"
#include <fstream>
#include <iostream>

Drone::Drone(std::string mapFile) {
    this->gpuSolver = new GPUDroneSolver(this);
    gpuSuccessful = false;

    this->mr = new MapReader();
    this->mr->readMap(mapFile);
}

Drone::~Drone() {
    delete this->gpuSolver;
    delete this->mr;
}

std::vector<int> Drone::plan(int sx, int sy, int sz, int ex, int ey, int ez,
                             bool randomPoints, double heuristicWeight) {
    if (randomPoints) {
        this->mr->setRandomStartEndPoints();
    } else {
        this->mr->setStartEndPoints(sx, sy, sz, ex, ey, ez);
    }

    this->gpuSolver->initialize(heuristicWeight);
    this->gpuSolution.clear();
    this->gpuSuccessful = gpuSolver->solve();
    this->gpuSolver->getSolution(&gpuOptimal, &gpuSolution);
    return this->gpuSolution;
}

int Drone::sx() const { return this->mr->getStartX(); }

int Drone::sy() const { return this->mr->getStartY(); }

int Drone::sz() const { return this->mr->getStartZ(); }

int Drone::ex() const { return this->mr->getEndX(); }

int Drone::ey() const { return this->mr->getEndY(); }

int Drone::ez() const { return this->mr->getEndZ(); }

int Drone::size() const {
    return this->mr->getMapX() * this->mr->getMapY() * this->mr->getMapZ();
}

int Drone::mapX() const { return this->mr->getMapX(); }

int Drone::mapY() const { return this->mr->getMapY(); }

int Drone::mapZ() const { return this->mr->getMapZ(); }

int Drone::toID(int x, int y, int z) const {
    return this->mr->xyzToIdx(x, y, z);
}

void Drone::toXYZ(int id, int *x, int *y, int *z) const {
    return this->mr->idxToXYZ(id, x, y, z);
}

bool Drone::isInRange(int x, int y, int z) const {
    return 0 <= x && x < this->mapX() && 0 <= y && y < this->mapY() && 0 <= z &&
           z < this->mapZ();
}

const uint8_t *Drone::graph() const { return this->mr->getGraph(); }
