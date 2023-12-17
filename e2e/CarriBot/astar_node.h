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

#include <queue>
#include <vector>

typedef std::vector<int> STATE;

struct Node {
    STATE s; // X and Y in 2D path planning
    double g, f;
    Node *parent;
    int dir; // The expansion direction: parent[dir] == this

    Node() {
        s = {};
        g = f = 0;
        parent = NULL;
        dir = -1;
    }

    Node(STATE _s, double _g, double _f, Node *_parent, int _dir) {
        s = _s;
        g = _g;
        f = _f;
        parent = _parent;
        dir = _dir;
    }
};

struct NodeCmp {
    bool operator()(const Node *left, const Node *right) {
        return left->f > right->f;
    }
};

typedef std::priority_queue<Node *, std::vector<Node *>, NodeCmp> MIN_HEAP;
