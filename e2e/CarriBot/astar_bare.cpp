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

#include "astar_bare.h"
#include "log.h"
#include "parallel_hashmap/phmap.h"
#include "utils.h"
#include <climits>
#include <limits.h>
#include <vector>

using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::nanoseconds;

AStar::AStar(double hWeight, bool deep) {
    this->heuristicWeight = hWeight;

    if (!deep) {
        this->runSearch = [this](STATE initial, STATE goal, uint64_t e) {
            return this->searchWithAstar(initial, goal, e);
        };
    } else {
        this->runSearch = [this](STATE initial, STATE goal, uint64_t e) {
            return this->searchWithIDAstar(initial, goal, e);
        };
    }
}

PATH AStar::run(STATE initial, STATE goal, uint64_t maxIterations) {
    if (!this->isValid(initial) || !this->isValid(goal)) return {};
    return this->runSearch(initial, goal, maxIterations);
}

PATH AStar::searchWithAstar(STATE initial, STATE goal, uint64_t maxIterations) {
    MIN_HEAP openList;
    BaseEPS<STATE> *visited = nullptr;
    if (initial.size() <= 3) {
        visited = new ShortEPS<STATE>(this->MAX_STATE_COORDINATE_BITS);
    } else {
        visited = new LongEPS<STATE>();
    }
    BaseEPM<STATE, double> *gVals = nullptr;
    if (initial.size() > 3) {
        gVals = new LongEPM<STATE, double>();
    } else {
        gVals = new ShortEPM<STATE, double>(this->MAX_STATE_COORDINATE_BITS);
    }

    Node *startNode =
        new Node(initial, 0, this->heuristicWeight * this->getH(initial, goal),
                 NULL, -1);
    openList.push(startNode);

    auto baseTime = high_resolution_clock::now();
    uint64_t numExpansions = 0;
    while (!openList.empty()) {
        Node *expNode = openList.top();
        openList.pop();

        for (auto e : expNode->s) {
            assert(e < (1 << this->MAX_STATE_COORDINATE_BITS));
        }

        if (visited->contains(expNode->s)) continue;
        visited->insert(expNode->s);

        if (++numExpansions >= maxIterations) break;

        if (unlikely(expNode->s == goal)) {
            assert(numExpansions <= maxIterations);
            auto t = high_resolution_clock::now();
            this->execTime =
                duration_cast<nanoseconds>(t - baseTime).count() * 1e-9;
            return this->getPath(expNode);
        }

        std::vector<STATE> neighbors = this->getNeighbors(expNode->s);

#pragma omp parallel for ordered num_threads(2)
        for (int dir = 0; dir < static_cast<int>(neighbors.size()); dir++) {
            STATE neighborState = neighbors[dir];
            assert(neighborState.size() == initial.size());
            if (visited->contains(neighborState)) continue;

            bool v = this->isValid(neighborState); // Could be time-consuming

            if (v) {
                double g =
                    this->getG(expNode->s, neighborState, expNode->g, dir);
                double f =
                    g + this->heuristicWeight * this->getH(neighborState, goal);

#pragma omp ordered
                if (!gVals->contains(neighborState) ||
                    g < gVals->getValue(neighborState)) {
                    gVals->insert(neighborState, g);
                    openList.push(new Node(neighborState, g, f, expNode, dir));
                }
            }
        }
    }

    assert(numExpansions <= maxIterations);
    auto t = high_resolution_clock::now();
    this->execTime = duration_cast<nanoseconds>(t - baseTime).count() * 1e-9;

    delete visited;
    delete gVals;
    delete startNode;

    return PATH();
}

PATH AStar::searchWithIDAstar(STATE initial, STATE goal,
                              uint64_t maxIterations) {
    double threshold = this->heuristicWeight * this->getH(initial, goal);
    Node *start = new Node(initial, 0, threshold, NULL, -1);

    Node *theGoalNode = new Node();

    while (maxIterations--) {
        double distance =
            this->deepSearch(initial, goal, start, 0, threshold, theGoalNode);
        if (distance == this->INF) break;
        if (distance < 0) {
            delete start;
            return this->getPath(theGoalNode);
        }
        threshold = distance;
    }

    delete start;

    return PATH();
}

double AStar::deepSearch(STATE initial, STATE goal, Node *node, double distance,
                         double threshold, Node *theGoalNode) {
    if (node->s == goal) return -distance;

    double c = distance + this->getH(node->s, goal);
    if (c > threshold) return c;

    double min = this->INF;
    std::vector<STATE> neighbors = this->getNeighbors(node->s);

    for (int dir = 0; dir < static_cast<int>(neighbors.size()); dir++) {
        STATE ns = neighbors[dir];
        assert(ns.size() == initial.size());
        if (!this->isValid(ns)) continue;

        double newG = this->getG(node->s, ns, node->g, dir);
        double t = this->deepSearch(
            initial, goal,
            new Node(ns, newG, newG + this->getH(ns, goal), node, dir), newG,
            threshold, theGoalNode);

        if (t < 0) {
            theGoalNode->s = ns;
            theGoalNode->g = newG;
            theGoalNode->f = newG + this->getH(ns, goal);
            theGoalNode->parent = node;
            theGoalNode->dir = dir;
            return t;
        }

        if (t < min) min = t;
    }

    return min;
}

double AStar::getExecTime() const { return this->execTime; }

PATH AStar::getPath(Node *n) const {
    PATH path;

    while (n) {
        path.push_back(n->dir);
        n = n->parent;
    }

    path.pop_back(); // dir is meaningless for the start node
    std::reverse(path.begin(), path.end());

    return path;
}
