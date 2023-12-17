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

#include "planner.h"
#include <list>
#include <utility>
#include <vector>

class PRM : public Planner {
  public:
    PRM(const Environment *env, int dof, int linkLength,
        double epsilon = PI / 20, double radius = PI / 3, int samples = 10000,
        int aStarWeight = 1);
    ~PRM();
    std::vector<double *> query(double *start, double *goal);

  private:
    struct Node {
        int pos;
        double g;
        double h;
        double f;
        Node *parent;

        Node(int _pos, double _g, double _h, Node *_p)
            : pos(_pos), g(_g), h(_h), parent(_p) {
            f = g + h; // NOTE: h must be the weighted heuristic
        }
    };

    struct Node_Comparator {
        bool operator()(const Node *left, const Node *right) {
            return left->f > right->f;
        }
    };

    void initializeGraph(int samples);
    void addVertice(double *cfg);

    double radius; // Neighborhood distance
    int aStarWeight;
    std::vector<std::pair<double *, std::list<int>>> graph;
};
