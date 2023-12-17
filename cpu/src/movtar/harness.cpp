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

#include "args.h"
#include "env.h"
#include "zsim_hooks.h"
#include <fstream>
#include <limits.h>
#include <queue>
#include <string>
#include <utility>
#include <vector>

// The largest acceptable map
#define MAX_X (2500)
#define MAX_Y (2500)

#define EPSILON (1) // Weighted A*

int gVals2D[MAX_X][MAX_Y];             // Used as a heuristic for the 3D search
std::vector<std::pair<int, int>> path; // The final path

void outputPath(std::string fileName) {
    std::ofstream outputFile;
    outputFile.open(fileName);
    for (auto rit = path.rbegin(); rit != path.rend(); ++rit) {
        // Consistent with the map files (i.e., 1-based indexing)
        outputFile << "<" << rit->first + 1 << ", " << rit->second + 1 << ">"
                   << std::endl;
    }
    outputFile.close();
}

void backwardDijkstra2D(Environment *env) {
    // Run Dijkstra to get the G values of nodes in 2D. At the beginning, put
    // ALL points of target's trajectory into the OPEN list.

    // 8-connected graph
    const int NUM_2D_DIRS = 8;
    int dX[NUM_2D_DIRS] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dY[NUM_2D_DIRS] = {-1, 0, 1, -1, 1, -1, 0, 1};

    struct Node {
        int x;
        int y;
        int g;

        Node(int _x, int _y, int _g) : x(_x), y(_y), g(_g) {}
    };

    struct NodeComparator {
        bool operator()(const Node *left, const Node *right) {
            return left->g > right->g;
        }
    };

    std::priority_queue<Node *, std::vector<Node *>, NodeComparator> heap;
    bool closedList[MAX_X][MAX_Y] = {0};

    for (int i = 0; i < env->getMapX(); i++) {
        for (int j = 0; j < env->getMapY(); j++) {
            gVals2D[i][j] = INT_MAX;
        }
    }

    for (int i = 0; i < env->getTargetSteps(); i++) {
        int objX = env->getTargetX(i);
        int objY = env->getTargetY(i);

        heap.push(new Node(objX, objY, 0));
        gVals2D[objX][objY] = 0;
    }

    while (!heap.empty()) {
        Node *expNode = heap.top();
        heap.pop();

        // Re-expanding a node
        if (closedList[expNode->x][expNode->y]) {
            if (expNode->g >= gVals2D[expNode->x][expNode->y]) {
                continue;
            }
        }

        closedList[expNode->x][expNode->y] = true;

        for (int i = 0; i < NUM_2D_DIRS; i++) {
            int newX = expNode->x + dX[i];
            int newY = expNode->y + dY[i];

            if (closedList[newX][newY]) continue;
            if (!env->isValid(newX, newY)) continue;
            if (!env->isFree(newX, newY)) continue;

            int newCost = gVals2D[expNode->x][expNode->y] + 1;
            if (newCost < gVals2D[newX][newY]) {
                gVals2D[newX][newY] = newCost;
                heap.push(new Node(newX, newY, newCost));
            }
        }
    }
}

// These data structures are too large to be placed inside the function (stack
// overflow is very likely)
std::unordered_map<int, bool> closedList3D[MAX_X][MAX_Y];
std::unordered_map<int, int> gVals3D[MAX_X][MAX_Y];

inline bool isVisited(int x, int y, int t) {
    return closedList3D[x][y].find(t) != closedList3D[x][y].end();
}

inline void markVisited(int x, int y, int t) { closedList3D[x][y][t] = true; }

inline int getGVal(int x, int y, int t) {
    if (gVals3D[x][y].find(t) == gVals3D[x][y].end()) {
        return INT_MAX;
    } else {
        return gVals3D[x][y][t];
    }
}

inline void updateGVal(int x, int y, int t, int val) { gVals3D[x][y][t] = val; }

inline double getMin2DDist(int x1, int y1, int x2, int y2) {
    // Returns the minimum distance in an obstacle-free 2D space, with
    // 8-connected graph movement constraints
    int dX = std::abs(x1 - x2);
    int dY = std::abs(y1 - y2);
    int maxD = (dX > dY ? dX : dY);
    int minD = (dX > dY ? dY : dX);
    return 0.4142 /*sqrt(2)-1*/ * minD + maxD;
}

void aStar3D(Environment *env) {
    const int NUM_3D_DIRS = 9;
    // In 3D with time as the third dimension, there's one more action: in the
    // next step (dt=1), stay in the same place (dx=dy=0)
    int dX[NUM_3D_DIRS] = {-1, -1, -1, 0, 0, 0, 1, 1, 1};
    int dY[NUM_3D_DIRS] = {-1, 0, 1, -1, 0, 1, -1, 0, 1};

    struct Node {
        int x;
        int y;
        int t;
        int g;
        int h;
        int f;
        Node *parent;

        Node(int _x, int _y, int _t, int _g, int _h, Node *_p)
            : x(_x), y(_y), t(_t), g(_g), h(_h), parent(_p) {
            f = g + EPSILON * h;
        }
    };

    struct NodeComparator {
        bool operator()(const Node *left, const Node *right) {
            return left->f > right->f;
        }
    };

    std::priority_queue<Node *, std::vector<Node *>, NodeComparator> heap;

    int srcX = env->getRobotX(), srcY = env->getRobotY();
    heap.push(new Node(srcX, srcY, 0, 0, gVals2D[srcX][srcY], NULL));
    updateGVal(srcX, srcY, 0 /*time*/, 0);

    while (!heap.empty()) {
        Node *expNode = heap.top();
        heap.pop();

        if (isVisited(expNode->x, expNode->y, expNode->t)) {
            if (expNode->g >= getGVal(expNode->x, expNode->y, expNode->t)) {
                continue;
            }
        }

        markVisited(expNode->x, expNode->y, expNode->t);

        if (env->getTargetX(expNode->t) == expNode->x &&
            env->getTargetY(expNode->t) == expNode->y) {
            Node *n = expNode;
            while (n->parent != NULL) {
                path.push_back(std::make_pair(n->x, n->y));
                n = n->parent;
            }
            path.push_back(std::make_pair(n->x, n->y));
            break;
        }

        for (int i = 0; i < NUM_3D_DIRS; i++) {
            int newX = expNode->x + dX[i];
            int newY = expNode->y + dY[i];
            int newT = expNode->t + 1;

            if (!env->isValid(newX, newY) || newT >= env->getTargetSteps()) {
                continue;
            }
            if (!env->isFree(newX, newY)) continue;
            if (isVisited(newX, newY, newT)) continue;

            int objX = env->getTargetX(newT), objY = env->getTargetY(newT);
            if (getMin2DDist(newX, newY, objX, objY) >
                env->getTargetSteps() - newT) {
                continue;
            }

            int newCost = getGVal(expNode->x, expNode->y, expNode->t) +
                          env->getCellCost(newX, newY);
            if (newCost < getGVal(newX, newY, newT)) {
                updateGVal(newX, newY, newT, newCost);
                heap.push(new Node(newX, newY, newT, newCost,
                                   gVals2D[newX][newY], expNode));
            }
        }
    }

    if (path.empty()) info("Could not find a path");
}

int main(int argc, const char **argv) {
    using args::KVArg;
    using args::Parser;

    Parser parser(argv[0], argc, argv, false);
    KVArg<std::string> inputMapArg(parser, "map", "", "Input map file");
    KVArg<std::string> outputPathArg(parser, "output", "", "Output path file");

    if (!parser.parse()) assert(false);

    assert_msg(inputMapArg.found(), "Input map file is not provided");

    const char *inputFile = inputMapArg.value().c_str();
    const char *outputFile =
        outputPathArg.found() ? outputPathArg.value().c_str() : "/dev/null";

    /*
     * This program is extremely time-consuming when the map size is large. I
     * sacrifice everything (memory size, stack-heap, etc.) for performance!
     */
    Environment *env = new Environment(inputFile);
    assert(env->getMapX() <= MAX_X && env->getMapY() <= MAX_Y);

    zsim_roi_begin();
    backwardDijkstra2D(env);
    zsim_roi_end();

    zsim_roi_begin();
    aStar3D(env);
    zsim_roi_end();

    outputPath(outputFile);

    delete env;

    return 0;
}
