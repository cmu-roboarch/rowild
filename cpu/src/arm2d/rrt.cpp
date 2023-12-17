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

#include "rrt.h"
#include <algorithm>
#include <vector>

RRT::RRT(const Environment *_env, int _dof, int _linkLength, double _epsilon,
         double _radius, int _maxSamples, double _goalBias, int _ppIters)
    : Planner(_env, _dof, _linkLength, _epsilon) {
    this->radius = _radius;
    this->maxSamples = _maxSamples;
    this->goalBias = _goalBias;
    this->ppIters = _ppIters;

    assert(this->goalBias >= 0 && this->goalBias <= 1);
    assert(ppIters >= 0);
}

RRT::~RRT() {
    for (auto t : this->tree) {
        delete[] t.first;
    }
}

std::vector<double *> RRT::query(double *startCfg, double *goalCfg) {
    auto generateRandomRatio = []() {
        return static_cast<double>(rand()) / RAND_MAX;
    };

    std::vector<double *> path; // Final path

    tree.push_back(std::make_pair(startCfg, -1 /*No parent*/));
    int samples = 0;

    while (true) {
        double *cfg;
        if (generateRandomRatio() < this->goalBias) {
            cfg = goalCfg;
        } else {
            cfg = generateRandomCfg();
        }

        if (unlikely(extendTree(cfg, goalCfg, this->radius) ==
                     EXT_STATUS::Reached)) {
            int index = tree.size() - 1;
            while (tree[index].second != -1) {
                path.push_back(tree[index].first);
                index = tree[index].second;
            }
            path.push_back(tree[index].first);
            assert(index == 0);
            break;
        }

        samples++;
        if (unlikely(samples > this->maxSamples)) break;
    }

    tree.clear();

    // Post-processing
    if (this->ppIters > 0) {
        auto generateRandomInteger = [](int a, int b) {
            // Generates a random integer in [a, b]
            assert(b >= a);
            int r = a + (static_cast<double>(rand()) / RAND_MAX) * (b - a);
            return r;
        };

        for (int i = 0; i < this->ppIters; i++) {
            int pLength = static_cast<int>(path.size());
            if (unlikely(pLength < 4)) break;

            int anchorA = generateRandomInteger(0, pLength - 3);
            int anchorB = generateRandomInteger(anchorA + 1, pLength - 2);
            assert((anchorA >= 0) && (anchorA < anchorB) &&
                   (anchorB < pLength - 1));

            double *candidateA = path[anchorA];
            double *candidateB = path[anchorB];

            if (canConnect(candidateA, candidateB)) {
                while (anchorB > anchorA) {
                    path.erase(path.begin() + anchorB);
                    anchorB--;
                }
            }
        }
    }

    return path;
}

RRT::EXT_STATUS RRT::extendTree(double *newCfg, double *goalCfg,
                                double extensionEpsilon) {
    int neareastIndex = findNearestNeighbour(newCfg);
    double *nearCfg = tree[neareastIndex].first;
    assert(neareastIndex >= 0 && neareastIndex < static_cast<int>(tree.size()));
    assert(isValidArmConfiguration(nearCfg));

    // We need to extend the nearest node in the tree (nearCfg) towards the new
    // sample (newCfg). The extension 1) should not be larger than
    // extensionEpsilon, 2) should not cross any obstacle, and 3) should be
    // performed as large as possible
    bool isConnectable = false;

    double dist = getNorm(nearCfg, newCfg);
    bool isClose = (dist <= extensionEpsilon);

    if (isClose && isValidArmConfiguration(newCfg)) {
        isConnectable = canConnect(nearCfg, newCfg);
    }

    if (isConnectable) {
        tree.push_back(std::make_pair(newCfg, neareastIndex));
        if (unlikely(getNorm(newCfg, goalCfg) <= this->epsilon)) {
            return EXT_STATUS::Reached;
        } else {
            return EXT_STATUS::Advanced;
        }
    }

    assert(!isConnectable);

    double *extensionPoint =
        getExtensionNode(nearCfg, newCfg, extensionEpsilon);
    bool trapped = equalCfgs(nearCfg, extensionPoint);

    assert(getNorm(nearCfg, extensionPoint) <= extensionEpsilon);
    assert(!equalCfgs(newCfg, extensionPoint));
    assert(canConnect(nearCfg, extensionPoint));

    if (trapped) {
        delete[] extensionPoint;
        return EXT_STATUS::Trapped;
    }

    tree.push_back(std::make_pair(extensionPoint, neareastIndex));
    return EXT_STATUS::Advanced;
}

int RRT::findNearestNeighbour(double *cfg) const {
    double minDistance = this->dof * MAX_ANGLE; // A large value
    int bestIndex = -1;

    double dist;
    for (int i = 0; i < static_cast<int>(tree.size()); i++) {
        dist = getNorm(tree[i].first, cfg);
        if (dist < minDistance) {
            minDistance = dist;
            bestIndex = i;
        }
    }

    assert(minDistance > 0);

    return bestIndex;
}

double *RRT::getExtensionNode(double *oldCfg, double *newCfg,
                              double extensionEpsilon) const {
    // This function tries to connect oldCfg to newCfg. oldCfg belongs to the
    // tree, while newCfg is a new node. If the connection is possible (i.e.,
    // there's no obstacle along the way and the distance is not larger than
    // extensionEpsilon), then the function returns newCfg as the node that
    // should be added to the tree. Otherwise, it returns the best node along
    // the way; i.e., a node that is collision-free and is as close to newCfg
    // as possible.

    assert(isValidArmConfiguration(oldCfg));
    bool *bitmap = new bool[this->dof];
    // bitmap[i] == 0: Front path is better
    // bitmap[i] == 1: Back path is better

    double distance = getDistance(oldCfg, newCfg, bitmap);

    assert(distance <= MAX_ANGLE / 2);

    double *bestPoint = new double[this->dof];

    // Initialize with oldCfg
    std::copy(oldCfg, oldCfg + this->dof, bestPoint);

    int localSamples = static_cast<int>(distance / this->epsilon);

    if (unlikely(localSamples < 2)) {
        if (localSamples == 1 && isValidArmConfiguration(newCfg)) {
            std::copy(newCfg, newCfg + this->dof, bestPoint);
        }
        return bestPoint;
    }

    // i == 0 is the same as oldCfg; it has already been validated
    for (int i = 1; i <= localSamples; i++) {
        double *midCfg = getMiddleCfg(oldCfg, newCfg, i, localSamples, bitmap);
        if (!isValidArmConfiguration(midCfg) ||
            getNorm(oldCfg, midCfg) > extensionEpsilon) {
            break;
        }
        std::copy(midCfg, midCfg + this->dof, bestPoint);
    }

    assert(isValidArmConfiguration(bestPoint));

    delete[] bitmap;

    return bestPoint;
}
