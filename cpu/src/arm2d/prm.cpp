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

#include "prm.h"
#include <list>
#include <queue>
#include <vector>

PRM::PRM(const Environment *_env, int _dof, int _linkLength, double _epsilon,
         double _radius, int _samples, int _aStarWeight)
    : Planner(_env, _dof, _linkLength, _epsilon) {
    this->radius = _radius;
    this->aStarWeight = _aStarWeight;

    initializeGraph(_samples);
}

PRM::~PRM() {
    for (auto g : this->graph) {
        delete[] g.first;
    }
}

void PRM::initializeGraph(int totalSamples) {
    int samples = 0;
    while (samples < totalSamples) {
        double *cfg = generateRandomCfg();
        if (!isValidArmConfiguration(cfg)) continue;
        addVertice(cfg);
        samples++;
    }
}

void PRM::addVertice(double *cfg) {
    int nodeId = graph.size();
    graph.push_back(std::make_pair(cfg, std::list<int>{}));

    for (int i = 0; i < nodeId /*Do not include the new node itself*/; i++) {
        double *neighbourCfg = graph[i].first;
        if (getNorm(neighbourCfg, cfg) > radius) continue;
        if (!canConnect(neighbourCfg, cfg)) continue;
        graph[i].second.push_back(nodeId);
        graph[nodeId].second.push_back(i);
    }
}

// This function is on the critical path and should be highly optimized
std::vector<double *> PRM::query(double *startCfg, double *goalCfg) {
    // Add start-goal to the graph
    addVertice(startCfg);
    addVertice(goalCfg);

    const int GRAPH_NODES = graph.size();
    int startPos = GRAPH_NODES - 2;
    int goalPos = GRAPH_NODES - 1;

    std::priority_queue<Node *, std::vector<Node *>, Node_Comparator> heap;
    std::vector<double *> path; // Final path

    bool visitedNodes[GRAPH_NODES] = {0};
    double gValues[GRAPH_NODES];

    for (int i = 0; i < GRAPH_NODES; i++)
        gValues[i] = INT_MAX;

    // Start node
    Node *node = new Node(startPos, 0,
                          this->aStarWeight * getNorm(graph[startPos].first,
                                                      graph[goalPos].first),
                          NULL);
    gValues[startPos] = 0;
    heap.push(node);

    while (!heap.empty()) {
        Node *expNode = heap.top();
        heap.pop();

        if (unlikely(visitedNodes[expNode->pos] &&
                     expNode->g >= gValues[expNode->pos])) {
            continue;
        }

        visitedNodes[expNode->pos] = true;

        if (unlikely(expNode->pos == goalPos)) {
            Node *n = expNode;
            while (n->parent != NULL) {
                path.push_back(graph[n->pos].first);
                n = n->parent;
            }
            path.push_back(graph[n->pos].first);

            assert(n->pos == startPos);
            break;
        }

        // If the number of neighbours (i.e., graph[expNode->pos].second) is
        // high, it may be beneficial to parallelize the following loop. But in
        // practice I found serial execution generally faster than a
        // multi-threaded one. So parallelize the loop at your own risk! Notice
        // you have to lock the heap or use a lock-free heap (check this:
        // https://github.com/jonatanlinden/PR)
        for (auto const &neighbourPos : graph[expNode->pos].second) {
            if (visitedNodes[neighbourPos]) continue;

            double newCost = expNode->g + getNorm(graph[expNode->pos].first,
                                                  graph[neighbourPos].first);
            if (newCost < gValues[neighbourPos]) {
                gValues[neighbourPos] = newCost;
                heap.push(new Node(neighbourPos, newCost,
                                   this->aStarWeight *
                                       getNorm(graph[neighbourPos].first,
                                               graph[goalPos].first),
                                   expNode));
            }
        }
    }

    // Remove start & goal
    graph.pop_back();
    graph.pop_back();
    delete[] node;

    return path;
}
