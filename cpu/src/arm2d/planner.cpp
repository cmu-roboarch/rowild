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

#include "planner.h"
#include <algorithm>
#include <vector>

Planner::Planner(const Environment *_env, int _dof, int _linkLength,
                 double _epsilon) {
    this->env = _env;
    this->envX = this->env->getMapX();
    this->envY = this->env->getMapY();
    this->dof = _dof;
    this->linkLength = _linkLength;
    this->epsilon = _epsilon; // Maximum movement & Close enough to goal

    srand(0);
}

void Planner::contXY2Cell(double x, double y, uint16_t *pX,
                          uint16_t *pY) const {
    double cellsize = 1.0;

    // Take the nearest cell
    *pX = static_cast<int>(x / cellsize);
    if (x < 0) *pX = 0;
    if (*pX >= this->envX) *pX = this->envX - 1;

    *pY = static_cast<int>(y / cellsize);
    if (y < 0) *pY = 0;
    if (*pY >= this->envY) *pY = this->envY - 1;
}

void Planner::getBresenhamParameters(int p1x, int p1y, int p2x, int p2y,
                                     Bresenham_Param *params) const {
    params->usingYIndex = 0;

    if (fabs(static_cast<double>(p2y - p1y) / static_cast<double>(p2x - p1x)) >
        1) {
        (params->usingYIndex)++;
    }

    if (params->usingYIndex) {
        params->y1 = p1x;
        params->x1 = p1y;
        params->y2 = p2x;
        params->x2 = p2y;
    } else {
        params->x1 = p1x;
        params->y1 = p1y;
        params->x2 = p2x;
        params->y2 = p2y;
    }

    if ((p2x - p1x) * (p2y - p1y) < 0) {
        params->flipped = 1;
        params->y1 = -params->y1;
        params->y2 = -params->y2;
    } else {
        params->flipped = 0;
    }

    if (params->x2 > params->x1) {
        params->increment = 1;
    } else {
        params->increment = -1;
    }

    params->deltaX = params->x2 - params->x1;
    params->deltaY = params->y2 - params->y1;

    params->incrE = 2 * params->deltaY * params->increment;
    params->incrNE = 2 * (params->deltaY - params->deltaX) * params->increment;
    params->dTerm = (2 * params->deltaY - params->deltaX) * params->increment;

    params->xIndex = params->x1;
    params->yIndex = params->y1;
}

void Planner::getCurrentPoint(Bresenham_Param *params, int *x, int *y) const {
    if (params->usingYIndex) {
        *y = params->xIndex;
        *x = params->yIndex;
        if (params->flipped) *x = -*x;
    } else {
        *x = params->xIndex;
        *y = params->yIndex;
        if (params->flipped) *y = -*y;
    }
}

bool Planner::getNextPoint(Bresenham_Param *params) const {
    if (params->xIndex == params->x2) return false;

    params->xIndex += params->increment;
    if (params->dTerm < 0 || (params->increment < 0 && params->dTerm <= 0)) {
        params->dTerm += params->incrE;
    } else {
        params->dTerm += params->incrNE;
        params->yIndex += params->increment;
    }

    return true;
}

bool Planner::isValidLineSegment(double x0, double y0, double x1,
                                 double y1) const {
    Bresenham_Param params;
    int nX, nY;
    uint16_t nX0, nY0, nX1, nY1;

    // Make sure the line segment is inside the environment
    if (x0 < 0 || x0 >= this->envX || x1 < 0 || x1 >= this->envX || y0 < 0 ||
        y0 >= this->envY || y1 < 0 || y1 >= this->envY) {
        return false;
    }

    contXY2Cell(x0, y0, &nX0, &nY0);
    contXY2Cell(x1, y1, &nX1, &nY1);

    // Iterate through the points on the segment
    getBresenhamParameters(nX0, nY0, nX1, nY1, &params);
    do {
        getCurrentPoint(&params, &nX, &nY);
        if (!this->env->isFree(nX, nY)) return false;
    } while (getNextPoint(&params));

    return true;
}

bool Planner::isValidArmConfiguration(double *angles) const {
    double x0, y0, x1, y1;

    // Iterate through all the links starting with the base
    x1 = this->envX / 2.0;
    y1 = 0;
    for (int i = 0; i < this->dof; i++) {
        // Compute the corresponding line segment
        x0 = x1;
        y0 = y1;
        x1 = x0 + this->linkLength * cos(MAX_ANGLE - angles[i]);
        y1 = y0 - this->linkLength * sin(MAX_ANGLE - angles[i]);

        // Check the validity of the corresponding line segment
        if (!isValidLineSegment(x0, y0, x1, y1)) return false;
    }

    return true;
}

double *Planner::generateRandomCfg() const {
    auto generateRandomAngle = []() -> double {
        return MIN_ANGLE + static_cast<double>(rand()) / RAND_MAX * MAX_ANGLE;
    };

    double *cfg = new double[this->dof];
    for (int i = 0; i < this->dof; i++)
        cfg[i] = generateRandomAngle();
    return cfg;
}

double Planner::getNorm(double *cfg1, double *cfg2) const {
    double norm = 0, diff;

    for (int i = 0; i < this->dof; i++) {
        diff = fabs(cfg1[i] - cfg2[i]);
        diff = std::min(diff, MAX_ANGLE - diff);
        norm += diff * diff;
        assert(cfg1[i] <= MAX_ANGLE && cfg2[i] <= MAX_ANGLE);
        assert(diff <= MAX_ANGLE / 2);
    }

    return sqrt(norm);
}

double Planner::getDistance(double *cfg1, double *cfg2, bool *bitmap) const {
    double distance = 0, frontDist, backDist, bestDist;
    for (int i = 0; i < this->dof; i++) {
        frontDist = fabs(cfg1[i] - cfg2[i]);
        backDist = MAX_ANGLE - frontDist;
        bestDist = std::min(frontDist, backDist);

        bitmap[i] = backDist < frontDist;
        distance = std::max(distance, bestDist);
    }

    return distance;
}

double *Planner::getMiddleCfg(double *cfg1, double *cfg2, int sampleNumber,
                              int totalSamples, bool *bitmap) const {
    double *midCfg = new double[this->dof];

    for (int j = 0; j < this->dof; j++) {
        if (bitmap[j]) {
            // The back path is better -> The interpolation should be on
            // the reverse direction
            if (cfg1[j] < cfg2[j]) {
                midCfg[j] = cfg1[j] + MAX_ANGLE +
                            static_cast<double>(sampleNumber) / totalSamples *
                                (cfg2[j] - cfg1[j] - MAX_ANGLE);
            } else {
                midCfg[j] = cfg1[j] + MAX_ANGLE +
                            static_cast<double>(sampleNumber) / totalSamples *
                                (cfg2[j] - cfg1[j] + MAX_ANGLE);
            }
            midCfg[j] = fmod(midCfg[j], MAX_ANGLE);
        } else {
            // The front path is better: A simple interpolation
            // between two angles
            midCfg[j] = cfg1[j] + static_cast<double>(sampleNumber) /
                                      totalSamples * (cfg2[j] - cfg1[j]);
        }
    }

    return midCfg;
}

bool Planner::canConnect(double *cfg1, double *cfg2) const {
    assert(isValidArmConfiguration(cfg1));
    assert(isValidArmConfiguration(cfg2));

    bool *bitmap = new bool[this->dof];
    // bitmap[i] == 0: The front path is better
    // bitmap[i] == 1: The back path is better

    double distance = getDistance(cfg1, cfg2, bitmap);
    assert(distance <= MAX_ANGLE / 2);

    int localSamples = static_cast<int>(distance / this->epsilon);
    if (unlikely(localSamples < 2)) return true;

    // i == 0 is the same as cfg1, and i == localSamples is the same as cfg2;
    // they have already been validated
    bool *v = new bool[localSamples - 1];

// Play with num_threads to get the best performance on your system
#pragma omp parallel for num_threads(2)
    for (int i = 1; i <= localSamples - 1; i++) {
        v[i - 1] = isValidArmConfiguration(
            getMiddleCfg(cfg1, cfg2, i, localSamples, bitmap));
    }
    delete[] bitmap;

    for (int i = 0; i < localSamples - 1; i++) {
        if (!v[i]) {
            return false;
        }
    }
    return true;
}

bool Planner::equalCfgs(double *cfg1, double *cfg2) const {
    typedef decltype(getVectorInstance<double>()) VECTOR_DATA_TYPE;
    constexpr int step = sizeof(VECTOR_DATA_TYPE) / sizeof(double);

    int firstScalIdx = this->dof / step;
    int r = this->dof - firstScalIdx;
    if (this->dof % step != 0) {
        for (int i = firstScalIdx; i < r; i++) {
            if (cfg1[i] != cfg2[i]) {
                return false;
            }
        }
    }

    for (int i = 0; i < firstScalIdx; i += step) {
        VECTOR_DATA_TYPE v1, v2;
        v1.load(cfg1 + i);
        v2.load(cfg2 + i);
        if (horizontal_or(v1 != v2)) return false;
    }

    return true;
}

double Planner::calcPathCost(std::vector<double *> const &path) const {
    if (unlikely(path.empty())) return -1.0;

    assert(path.size() != 1);

    double cost = 0;
    for (int i = 1; i < static_cast<int>(path.size()); i++) {
        cost += getNorm(path[i], path[i - 1]);
        assert(getNorm(path[i], path[i - 1]) > 0);
    }

    return cost;
}
