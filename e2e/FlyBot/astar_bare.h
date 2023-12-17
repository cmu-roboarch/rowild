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

#include "astar_node.h"
#include "utils.h"
#include <bits/stdc++.h>
#include <functional>
#include <stdint.h>
#include <vector>

typedef std::vector<int> PATH;

class AStar {
  public:
    AStar(double hWeight, bool deep);
    PATH run(STATE initial, STATE goal, uint64_t maxIterations);
    double getExecTime() const;

  protected:
    virtual std::vector<STATE> getNeighbors(const STATE &s) const = 0;
    virtual bool isValid(const STATE &s) const = 0;
    virtual double getG(const STATE &prevState, const STATE &currState,
                        double prevG, int dir) const = 0;
    virtual double getH(const STATE &s, const STATE &goal) const = 0;
    const int MAX_STATE_COORDINATE_BITS = 16;

  private:
    std::function<PATH(STATE, STATE, uint64_t)> runSearch;
    PATH searchWithAstar(STATE initial, STATE goal, uint64_t maxIterations);
    PATH searchWithIDAstar(STATE initial, STATE goal, uint64_t maxIterations);
    double deepSearch(STATE initial, STATE goal, Node *node, double distance,
                      double threshold, Node *theGoalNode);
    PATH getPath(Node *n) const;

    double heuristicWeight;
    double execTime;
    const double FOUND = -1.0;
    const double INF = INT_MAX;
};
