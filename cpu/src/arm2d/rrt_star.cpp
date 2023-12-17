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

#include "rrt_star.h"
#include <algorithm>
#include <utility>
#include <vector>

RRTStar::RRTStar(const Environment *_env, int _dof, int _linkLength,
                 double _epsilon, double _radius, int _maxSamples,
                 double _goalBias)
    : Planner(_env, _dof, _linkLength, _epsilon) {
    this->radius = _radius;
    this->maxSamples = _maxSamples;
    this->goalBias = _goalBias;

    assert(this->goalBias >= 0 && this->goalBias <= 1);
}

RRTStar::~RRTStar() {
    for (auto t : this->tree) {
        delete[] t.cfg;
    }
}

std::vector<double *> RRTStar::query(double *startCfg, double *goalCfg) {
    auto generateRandomRatio = []() {
        return static_cast<double>(rand()) / RAND_MAX;
    };

    std::vector<double *> path; // Final path

    tree.push_back(RRTStar_Vertice(startCfg, -1 /*parent*/, 0 /*cost*/));
    int samples = 0;

    while (true) {
        double *cfg;
        if (generateRandomRatio() < this->goalBias) {
            cfg = goalCfg;
        } else {
            cfg = generateRandomCfg();
        }

        if (unlikely(extendRewireTree(cfg, goalCfg, this->radius) ==
                     EXT_STATUS::Reached)) {
            int index = tree.size() - 1;
            while (tree[index].parent != -1) {
                path.push_back(tree[index].cfg);
                index = tree[index].parent;
            }
            path.push_back(tree[index].cfg);
            assert(index == 0);
            break;
        }

        samples++;
        if (unlikely(samples > this->maxSamples)) break;
    }

    tree.clear();
    return path;
}

RRTStar::EXT_STATUS RRTStar::extendRewireTree(double *randCfg, double *goalCfg,
                                              double extensionEpsilon) {
    int neareastIndex = findNearestNeighbour(randCfg);
    double *nearCfg = tree[neareastIndex].cfg;
    assert(neareastIndex >= 0 && neareastIndex < static_cast<int>(tree.size()));
    assert(isValidArmConfiguration(nearCfg));

    double *newCfg = getExtensionNode(nearCfg, randCfg, extensionEpsilon);
    assert(getNorm(nearCfg, newCfg) <= extensionEpsilon);
    assert(canConnect(nearCfg, newCfg));

    if (equalCfgs(nearCfg, newCfg)) return EXT_STATUS::Trapped;

    // "Re-wiring"
    // Check all nodes in the close distance of newCfg and
    // find the path with minimum cost to newCfg
    int minIndex = neareastIndex;
    double minCost =
        tree[minIndex].cost + getNorm(tree[neareastIndex].cfg, newCfg);

    std::vector<std::pair<int, double>> neighbors;
    getNeighborNodes(neighbors, newCfg, this->radius);

    for (auto const &xNear : neighbors) {
        assert(xNear.first >= 0 && xNear.first < static_cast<int>(tree.size()));
        assert(xNear.second > 0 && xNear.second <= this->radius);

        // If the near node cannot be connected to the newCfg, then we cannot
        // improve the cost of newCfg using it.
        if (!canConnect(tree[xNear.first].cfg, newCfg)) continue;

        double newCost = tree[xNear.first].cost + xNear.second;
        if (newCost < minCost) {
            minIndex = xNear.first;
            minCost = newCost;
        }
    }

    assert(minCost <=
           tree[neareastIndex].cost + getNorm(tree[neareastIndex].cfg, newCfg));
    assert(minCost ==
           tree[minIndex].cost + getNorm(tree[minIndex].cfg, newCfg));
    assert(minCost > 0);

    tree.push_back(RRTStar_Vertice(newCfg, minIndex /*best parent*/, minCost));

    // The number of neighbors could be large if the radius is large. So
    // uncomment the pragma if your app needs a large radius

    // #pragma omp parallel for
    for (auto const &xNear : neighbors) {
        if (xNear.first == minIndex) continue;

        if (!canConnect(newCfg, tree[xNear.first].cfg)) continue;

        double newCost =
            tree.back().cost + getNorm(newCfg, tree[xNear.first].cfg);
        assert(tree.back().cost > 0);
        assert(getNorm(newCfg, tree[xNear.first].cfg) > 0);
        assert(newCost > 0);
        assert(newCost > minCost);

        if (newCost < tree[xNear.first].cost) {
            tree[xNear.first].cost = newCost;
            tree[xNear.first].parent = tree.size() - 1; // newCfg
        }
    }

    if (getNorm(newCfg, goalCfg) <= this->epsilon) {
        return EXT_STATUS::Reached;
    } else {
        return EXT_STATUS::Advanced;
    }
}

int RRTStar::findNearestNeighbour(double *cfg) const {
    double minDistance = this->dof * MAX_ANGLE; // A large value
    int bestIndex = -1;

    double dist;
    for (int i = 0; i < static_cast<int>(tree.size()); i++) {
        dist = getNorm(tree[i].cfg, cfg);
        if (dist < minDistance) {
            minDistance = dist;
            bestIndex = i;
        }
    }

    assert(minDistance > 0);

    return bestIndex;
}

double *RRTStar::getExtensionNode(double *oldCfg, double *newCfg,
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

void RRTStar::getNeighborNodes(std::vector<std::pair<int, double>> &neighbors,
                               double *newCfg, double radius) const {
    assert(!tree.empty());
    assert(neighbors.empty());

    for (int i = 0; i < static_cast<int>(tree.size()); i++) {
        double dist = getNorm(tree[i].cfg, newCfg);
        assert(dist > 0);
        if (dist <= radius) neighbors.push_back(std::make_pair(i, dist));
    }
}
