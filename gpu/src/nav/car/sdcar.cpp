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

#include "sdcar.h"
#include "GPU-solver.hpp"
#include "log.h"
#include "rowild_utils.h"
#include <fstream>
#include <iostream>
#include <math.h>

CarRobot::CarRobot(std::string mapFile) {
    this->gpuSolver = new GPUCarRobotSolver(this);
    this->gpuSuccessful = false;

    this->mr = new MapReader(this->numThetaIndices);
    this->mr->readMap(mapFile);

    // Pre-compute sin and cos for fast execution
    for (int i = 0; i < this->numThetaIndices; i++) {
        this->sinThetas.push_back(sin(i * TWO_PI / this->numThetaIndices));
        this->cosThetas.push_back(cos(i * TWO_PI / this->numThetaIndices));
    }
}

CarRobot::~CarRobot() {
    delete this->gpuSolver;
    delete this->mr;
}

std::vector<int> CarRobot::plan(int sx, int sy, double sthetaVal, int ex,
                                int ey, double ethetaVal, bool randomPoints,
                                double heuristicWeight) {
    if (randomPoints) {
        this->setRandomStartEndPoints();
    } else {
        this->setStartEndPoints(sx, sy, sthetaVal, ex, ey, ethetaVal);
    }

    this->gpuSolver->initialize(heuristicWeight);
    this->gpuSolution.clear();
    this->gpuSuccessful = gpuSolver->solve();
    this->gpuSolver->getSolution(&gpuOptimal, &gpuSolution);
    return this->gpuSolution;
}

int CarRobot::sx() const { return this->startX; }

int CarRobot::sy() const { return this->startY; }

int CarRobot::sthetaIdx() const { return this->startThetaIdx; }

int CarRobot::ex() const { return this->endX; }

int CarRobot::ey() const { return this->endY; }

int CarRobot::ethetaIdx() const { return this->endThetaIdx; }

int CarRobot::size() const { return this->mr->getMapX() * this->mr->getMapY(); }

int CarRobot::mapX() const { return this->mr->getMapX(); }

int CarRobot::mapY() const { return this->mr->getMapY(); }

int CarRobot::toID(int x, int y, int thetaIdx) const {
    return this->mr->xythetaToIdx(x, y, thetaIdx);
}

void CarRobot::toXYTheta(int id, int *x, int *y, int *thetaIdx) const {
    return this->mr->idxToXYTheta(id, x, y, thetaIdx);
}

const uint8_t *CarRobot::graph() const { return this->mr->getGraph(); }

void CarRobot::setStartEndPoints(int sx, int sy, double sthetaVal, int ex,
                                 int ey, double ethetaVal) {
    int sthetaIdx = this->convertThetaValToIdx(sthetaVal);
    int ethetaIdx = this->convertThetaValToIdx(ethetaVal);

    assert(this->isStateFree(sx, sy, sthetaIdx));
    assert(this->isStateFree(ex, ey, ethetaIdx));

    this->startX = sx;
    this->startY = sy;
    this->startThetaIdx = sthetaIdx;
    this->endX = ex;
    this->endY = ey;
    this->endThetaIdx = ethetaIdx;
}

void CarRobot::setRandomStartEndPoints() {
    int startXYIdx, endXYIdx;
    bool free, diff;

    do {
        startXYIdx = this->mr->genRandFreeIdx();
        this->mr->idxToXYTheta(startXYIdx, &(this->startX), &(this->startY),
                               &(this->startThetaIdx));
        assert(this->mr->isIdxValid(startXYIdx));
        assert(this->mr->isPointFree(this->startX, this->startY));
        assert(0 <= this->startThetaIdx < this->numThetaIndices);

        free =
            this->isStateFree(this->startX, this->startY, this->startThetaIdx);
    } while (!free);

    do {
        endXYIdx = this->mr->genRandFreeIdx();
        this->mr->idxToXYTheta(endXYIdx, &(this->endX), &(this->endY),
                               &(this->endThetaIdx));
        assert(this->mr->isIdxValid(endXYIdx));
        assert(this->mr->isPointFree(this->endX, this->endY));
        assert(0 <= this->endThetaIdx < this->numThetaIndices);

        free = this->isStateFree(this->endX, this->endY, this->endThetaIdx);

        diff = (startXYIdx != endXYIdx) ||
               (this->startThetaIdx != this->endThetaIdx);
    } while (!free || !diff);
}

int CarRobot::getRobotLength() const {
    return this->robotLength; // X
}

int CarRobot::getRobotWidth() const {
    return this->robotWidth; // Y
}

int CarRobot::getNumThetaIndices() const { return this->numThetaIndices; }

bool CarRobot::isStateFree(int x, int y, int thetaIdx) const {
    assert(0 <= thetaIdx < this->numThetaIndices);
    double sine = this->sinThetas[thetaIdx];
    double cosine = this->cosThetas[thetaIdx];

    for (int i = 0; i <= this->robotLength; i++) {
        for (int j = 0; j <= this->robotWidth; j++) {
            int xbar = x + round(i * cosine);
            int ybar = y + round(j * sine);

            if (!this->mr->isInRange(xbar, ybar) ||
                !this->mr->isPointFree(xbar, ybar)) {
                return false;
            }
        }
    }

    return true;
}

int CarRobot::convertThetaValToIdx(double thetaVal) const {
    assert(0 <= thetaVal < TWO_PI);
    return round((thetaVal / TWO_PI) * this->numThetaIndices);
}

double CarRobot::convertThetaIdxToVal(int thetaIdx) const {
    assert(0 <= thetaIdx < this->numThetaIndices);
    return (TWO_PI * thetaIdx) / this->numThetaIndices;
}
