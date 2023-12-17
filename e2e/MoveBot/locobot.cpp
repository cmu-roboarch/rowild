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

#include <algorithm>
#include <cmath>
#include <iterator>
#include <string>
#include <vector>

#include "locobot.h"
#include "vcl/vectorclass.h"

LoCoBot::LoCoBot(const Environment *_env) {
    this->env = _env;

    double I4[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

    for (int i = 0; i < 6; i++) {
        rpyxyzToH(this->rDesc[i][0], this->rDesc[i][1], this->rDesc[i][2],
                  this->rDesc[i][3], this->rDesc[i][4], this->rDesc[i][5],
                  this->TLink[i]);
        std::copy(&I4[0][0], &I4[0][0] + 16, &this->TJoint[i][0][0]);
        std::copy(&I4[0][0], &I4[0][0] + 16, &this->TCurr[i][0][0]);
    }

    this->forwardKin({0, 0, 0, 0, 0});

    for (int i = 0; i < 4; i++) {
        rpyxyzToH(this->cDesc[i][0], this->cDesc[i][1], this->cDesc[i][2],
                  this->cDesc[i][3], this->cDesc[i][4], this->cDesc[i][5],
                  this->TBlock[i]);
        std::copy(&I4[0][0], &I4[0][0] + 16, &this->TColl[i][0][0]);
    }

    srand(0);
}

std::vector<JOINT_CFG> LoCoBot::planRRT(const JOINT_CFG &start,
                                        const JOINT_CFG &goal, double threshold,
                                        int maxSamples, double goalBias,
                                        int ppIters) {
    std::vector<JOINT_CFG> rrtVertices;
    std::vector<int> rrtEdges;
    rrtVertices.push_back(start);
    rrtEdges.push_back(0);
    bool foundSol = false;

    auto findNearest = [&rrtVertices, this](const JOINT_CFG &q) {
        int bestIdx = 0;
        double minDist = this->getNorm(rrtVertices[0], q);

        for (int idx = 1; idx < static_cast<int>(rrtVertices.size()); idx++) {
            double d = this->getNorm(rrtVertices[idx], q);
            if (d < minDist) {
                bestIdx = idx;
                minDist = d;
            }
        }

        return bestIdx;
    };

    while (static_cast<int>(rrtVertices.size()) < maxSamples) {
        JOINT_CFG qRand = this->sampleRobotCfg();
        if (static_cast<double>(rand()) / RAND_MAX < 0.05) qRand = goal;

        int idxNear = findNearest(qRand);
        assert(idxNear < static_cast<int>(rrtEdges.size()));
        JOINT_CFG qNear = rrtVertices[idxNear];
        assert(qNear.size() == qRand.size());

        JOINT_CFG qConnect;
        double randNearDist = this->getNorm(qRand, qNear);

        if (randNearDist > threshold) {
            double f = threshold / randNearDist;
            for (int i = 0; i < static_cast<int>(qRand.size()); i++) {
                qConnect.push_back(qNear[i] + f * (qRand[i] - qNear[i]));
            }
        } else {
            qConnect = qRand;
        }

        bool y = this->detectCollEdge(qConnect, qNear);
        if (!y) {
            rrtVertices.push_back(qConnect);
            rrtEdges.push_back(idxNear);
        }

        idxNear = findNearest(goal);
        if (this->getNorm(goal, rrtVertices[idxNear]) < threshold) {
            rrtVertices.push_back(goal);
            rrtEdges.push_back(idxNear);
            foundSol = true;
            break;
        }
    }

    assert(rrtVertices.size() == rrtEdges.size());

    std::vector<JOINT_CFG> path;
    if (foundSol) {
        // Last vertex is goal
        int c = static_cast<int>(rrtVertices.size()) - 1;
        path.push_back(rrtVertices[c]);
        do {
            c = rrtEdges[c];
            path.push_back(rrtVertices[c]);
        } while (c != 0);

        std::reverse(path.begin(), path.end());
    }

    if (ppIters > 0) {
        auto generateRandomInteger = [](int a, int b) {
            // Generates a random integer in [a, b]
            assert(b >= a);
            int r = a + (static_cast<double>(rand()) / RAND_MAX) * (b - a);
            return r;
        };

        for (int i = 0; i < ppIters; i++) {
            int pLength = static_cast<int>(path.size());
            if (unlikely(pLength < 4)) break;

            int anchorA = generateRandomInteger(0, pLength - 3);
            int anchorB = generateRandomInteger(anchorA + 1, pLength - 2);
            assert((anchorA >= 0) && (anchorA < anchorB) &&
                   (anchorB < pLength - 1));

            JOINT_CFG candidateA = path[anchorA];
            JOINT_CFG candidateB = path[anchorB];

            if (this->detectCollEdge(candidateA, candidateB)) {
                while (anchorB > anchorA) {
                    path.erase(path.begin() + anchorB);
                    anchorB--;
                }
            }
        }
    }

    return path;
}

void LoCoBot::initializePRM(double threshold, int maxSamples) {
    assert(!prmInitialized);
    this->prmThreshold = threshold;
    int numVertices = 0;
    while (numVertices < maxSamples) {
        JOINT_CFG q = this->sampleRobotCfg();
        if (this->detectCollNode(q)) continue;

        this->prmVertices.push_back(q);
        this->prmEdges.push_back({});
        numVertices++;

        for (int j = 0; j < numVertices - 1; j++) {
            if (this->getNorm(prmVertices[numVertices - 1], prmVertices[j]) <
                this->prmThreshold) {
                if (!this->detectCollEdge(prmVertices[numVertices - 1],
                                          prmVertices[j])) {
                    prmEdges[numVertices - 1].push_back(j);
                    prmEdges[j].push_back(numVertices - 1);
                }
            }
        }
    }

    assert(this->prmVertices.size() == this->prmEdges.size());
    assert(static_cast<int>(this->prmVertices.size()) == numVertices);
    prmInitialized = true;
}

std::vector<JOINT_CFG> LoCoBot::planPRM(const JOINT_CFG &start,
                                        const JOINT_CFG &goal) {
    assert(prmInitialized);
    assert(this->prmThreshold != -1.0);
    std::vector<int> neighInit, neighGoal, parent;
    std::vector<double> heuristic;

    for (int i = 0; i < static_cast<int>(this->prmVertices.size()); i++) {
        if (this->getNorm(this->prmVertices[i], start) < this->prmThreshold) {
            if (!this->detectCollEdge(this->prmVertices[i], start)) {
                neighInit.push_back(i);
            }
        }

        if (this->getNorm(prmVertices[i], goal) < this->prmThreshold) {
            if (!this->detectCollEdge(prmVertices[i], goal)) {
                neighGoal.push_back(i);
            }
        }

        heuristic.push_back(this->getNorm(prmVertices[i], goal));
        parent.push_back(0);
    }

    std::vector<int> activeNodes = neighInit;
    int bestScore = 0;
    auto isAnyCloseGoal = [&activeNodes, &neighGoal]() {
        constexpr int step = sizeof(Vec8i) / sizeof(int);
        Vec8i av;
        for (auto g : neighGoal) {
            for (int i = 0; i < static_cast<int>(activeNodes.size());
                 i += step) {
                av.load(&activeNodes[0] + i);
                if (horizontal_or(av == g)) return true;
            }
        }

        return false;
    };

    auto isInActive = [&activeNodes](int e) {
        constexpr int step = sizeof(Vec8i) / sizeof(int);
        Vec8i av;
        for (int i = 0; i < static_cast<int>(activeNodes.size()); i += step) {
            av.load(&activeNodes[0] + i);
            if (horizontal_or(av == e)) return true;
        }
        return false;
    };

    constexpr const int rNum = 1000;
    int bestCandid = -1, bestParent = -1;
    while (bestScore < rNum && !isAnyCloseGoal()) {
        bestScore = rNum;
        for (int i = 0; i < static_cast<int>(activeNodes.size()); i++) {
            for (int j = 0;
                 j < static_cast<int>(this->prmEdges[activeNodes[i]].size());
                 j++) {
                int edge = this->prmEdges[activeNodes[i]][j];
                if (!isInActive(edge)) {
                    if (heuristic[edge] < bestScore) {
                        bestScore = heuristic[edge];
                        bestCandid = edge;
                        bestParent = activeNodes[i];
                    }
                }
            }
        }

        if (bestScore < rNum) {
            activeNodes.push_back(bestCandid);
            parent[bestCandid] = bestParent;
        }
    }

    if (!isAnyCloseGoal()) {
        info("Failed to find a plan");
        return {};
    }

    std::vector<int> plan = {activeNodes[activeNodes.size() - 1]};
    int prevStep = parent[plan[0]];
    while (prevStep != 0) {
        plan.push_back(prevStep);
        prevStep = parent[plan.back()];
    }

    std::vector<JOINT_CFG> path;
    path.push_back(start);
    for (auto it = plan.rbegin(); it != plan.rend(); ++it) {
        path.push_back(this->prmVertices[(*it)]);
    }

    path.push_back(goal);
    return path;
}

JOINT_CFG LoCoBot::sampleRobotCfg() const {
    JOINT_CFG cfg;
    for (int i = 0; i < this->numJoints; i++) {
        double r = static_cast<double>(rand()) / RAND_MAX;
        cfg.push_back(this->qMin + (this->qMax - this->qMin) * r);
    }

    return cfg;
}

bool LoCoBot::detectCollNode(const JOINT_CFG ang) {
    this->compCollBlockPoints(ang);

    auto pointsObs = this->env->getPointsObs();
    auto axesObs = this->env->getAxesObs();
    assert(pointsObs.size() == axesObs.size());
    int numObstacles = static_cast<int>(axesObs.size());

    bool *checks = new bool[4 * numObstacles];

#pragma omp parallel for num_threads(4)
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < numObstacles; j++) {
            checks[i * numObstacles + j] =
                cuboidCuboidCollision(this->cPoints[i], this->cAxes[i],
                                      pointsObs[j].point, axesObs[j].axes);
        }
    }

    for (int i = 0; i < 4 * numObstacles; i++) {
        if (checks[i]) {
            delete[] checks;
            return true;
        }
    }

    delete[] checks;
    return false;
}

bool LoCoBot::detectCollEdge(const JOINT_CFG &ang1, const JOINT_CFG &ang2,
                             int numSteps) {
    assert(ang1.size() == ang2.size());

    double stride = 1.0 / (numSteps - 1);
    for (double s = 0; s <= 1; s += stride) {
        JOINT_CFG ang;
        for (size_t k = 0; k < ang1.size(); k++) {
            ang.push_back(ang1[k] + s * (ang2[k] - ang1[k]));
        }

        if (this->detectCollNode(ang)) return true;
    }

    return false;
}

void LoCoBot::forwardKin(const JOINT_CFG &ang) {
    for (int i = 0; i < this->numJoints; i++) {
        this->q[i] = ang[i];
    }

    auto isAxis = [this](int idx, const double(&c)[3]) {
        return std::equal(std::begin(this->axis[idx]),
                          std::end(this->axis[idx]), std::begin(c));
    };

    // Compute current joint and end effector coordinate frames
    // Notice not all joints rotate about the z axis!
    for (int i = 0; i < 6; i++) {
        if (isAxis(i, {0, 0, 1})) {
            double t[4][4] = {{cos(this->q[i]), -sin(this->q[i]), 0, 0},
                              {sin(this->q[i]), cos(this->q[i]), 0, 0},
                              {0, 0, 1, 0},
                              {0, 0, 0, 1}};
            std::copy(&t[0][0], &t[0][0] + 16, &this->TJoint[i][0][0]);
        } else if (isAxis(i, {-1, 0, 0})) {
            double t[4][4] = {{1, 0, 0, 0},
                              {0, cos(this->q[i]), sin(this->q[i]), 0},
                              {0, -sin(this->q[i]), cos(this->q[i]), 0},
                              {0, 0, 0, 1}};
            std::copy(&t[0][0], &t[0][0] + 16, &this->TJoint[i][0][0]);
        } else if (isAxis(i, {0, 1, 0})) {
            double t[4][4] = {{cos(this->q[i]), 0, sin(this->q[i]), 0},
                              {0, 1, 0, 0},
                              {-sin(this->q[i]), 0, cos(this->q[i]), 0},
                              {0, 0, 0, 1}};
            std::copy(&t[0][0], &t[0][0] + 16, &this->TJoint[i][0][0]);
        } else {
            panic("Axis rotation is not defined");
        }

        if (i == 0) {
            matrixMultiplication<double, 4, 4, 4, 4>(
                this->TLink[i], this->TJoint[i], this->TCurr[i]);
        } else {
            double temp[4][4];
            matrixMultiplication<double, 4, 4, 4, 4>(this->TCurr[i - 1],
                                                     this->TLink[i], temp);
            matrixMultiplication<double, 4, 4, 4, 4>(temp, this->TJoint[i],
                                                     TCurr[i]);
        }
    }
}

void LoCoBot::compCollBlockPoints(const JOINT_CFG &ang) {
    this->forwardKin(ang);

    // Compute current collision boxes for the arm
    for (int i = 0; i < 4; i++) {
        int idx = this->cIdx[i];
        matrixMultiplication<double, 4, 4, 4, 4>(
            this->TCurr[idx] /*Joint frame*/,
            this->TBlock[i] /*Local box transform*/, this->TColl[i]);

        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                this->cAxes[i][j][k] = this->TColl[i][k][j];
            }
        }

        blockDescToBoundingBox(this->TColl[i], this->cDim[i], this->cPoints[i]);
    }
}

double LoCoBot::getNorm(const JOINT_CFG &q1, const JOINT_CFG &q2) const {
    assert(q1.size() == q2.size());
    double dist = 0;
    for (int i = 0; i < static_cast<int>(q1.size()); i++) {
        dist += (q1[i] - q2[i]) * (q1[i] - q2[i]);
    }
    return sqrt(dist);
}

double LoCoBot::calcPathCost(const std::vector<JOINT_CFG> &path) const {
    if (unlikely(path.empty())) return -1.0;
    assert(path.size() != 1);
    double cost = 0;
    for (int i = 1; i < static_cast<int>(path.size()); i++) {
        cost += getNorm(path[i], path[i - 1]);
        assert(getNorm(path[i], path[i - 1]) > 0);
    }

    return cost;
}

void LoCoBot::printFullConfiguration() const {
    std::cout << "rDesc:" << std::endl;
    printMatrix<double, 6, 6>(this->rDesc);
    std::cout << std::endl << std::endl;

    std::cout << "axis:" << std::endl;
    printMatrix<double, 6, 3>(this->axis);
    std::cout << std::endl << std::endl;

    std::cout << "TLink:" << std::endl;
    print3dArray<double, 6, 4, 4>(this->TLink);
    std::cout << std::endl << std::endl;

    std::cout << "TJoint:" << std::endl;
    print3dArray<double, 6, 4, 4>(this->TJoint);
    std::cout << std::endl << std::endl;

    std::cout << "TCurr:" << std::endl;
    print3dArray<double, 6, 4, 4>(this->TCurr);
    std::cout << std::endl << std::endl;

    std::cout << "q:" << std::endl;
    printArray<double, 6>(this->q);
    std::cout << std::endl << std::endl;

    std::cout << "cDesc:" << std::endl;
    printMatrix<double, 4, 6>(this->cDesc);
    std::cout << std::endl << std::endl;

    std::cout << "cDim:" << std::endl;
    printMatrix<double, 4, 3>(this->cDim);
    std::cout << std::endl << std::endl;

    std::cout << "TBlock:" << std::endl;
    print3dArray<double, 4, 4, 4>(this->TBlock);
    std::cout << std::endl << std::endl;

    std::cout << "TColl:" << std::endl;
    print3dArray<double, 4, 4, 4>(this->TColl);
    std::cout << std::endl << std::endl;

    std::cout << "cPoints:" << std::endl;
    print3dArray<double, 4, 9, 3>(this->cPoints);
    std::cout << std::endl << std::endl;

    std::cout << "cAxes:" << std::endl;
    print3dArray<double, 4, 3, 3>(this->cAxes);
    std::cout << std::endl << std::endl;
}

void LoCoBot::printDynamics() const {
    std::cout << "TLink:" << std::endl;
    print3dArray<double, 6, 4, 4>(this->TLink);
    std::cout << std::endl << std::endl;

    std::cout << "TJoint:" << std::endl;
    print3dArray<double, 6, 4, 4>(this->TJoint);
    std::cout << std::endl << std::endl;

    std::cout << "TCurr:" << std::endl;
    print3dArray<double, 6, 4, 4>(this->TCurr);
    std::cout << std::endl << std::endl;

    std::cout << "q:" << std::endl;
    printArray<double, 6>(this->q);
    std::cout << std::endl << std::endl;

    std::cout << "TBlock:" << std::endl;
    print3dArray<double, 4, 4, 4>(this->TBlock);
    std::cout << std::endl << std::endl;

    std::cout << "TColl:" << std::endl;
    print3dArray<double, 4, 4, 4>(this->TColl);
    std::cout << std::endl << std::endl;

    std::cout << "cPoints:" << std::endl;
    print3dArray<double, 4, 9, 3>(this->cPoints);
    std::cout << std::endl << std::endl;

    std::cout << "cAxes:" << std::endl;
    print3dArray<double, 4, 3, 3>(this->cAxes);
    std::cout << std::endl << std::endl;
}

void LoCoBot::printPRMData() const {
    assert(prmInitialized);
    assert(prmThreshold != -1);
    assert(prmVertices.size() == prmEdges.size());

    std::cout << "PRM Vertices:" << std::endl;
    for (auto v : prmVertices) {
        for (auto c : v) {
            std::cout << c << " ";
        }
        std::cout << std::endl;
    }

    std::cout << std::endl << "PRM Edges:" << std::endl;
    for (auto e : prmEdges) {
        if (e.empty()) continue;
        for (auto i : e) {
            std::cout << i << " ";
        }
        std::cout << std::endl;
    }

    std::cout << std::endl << std::string(10, '-') << std::endl;
}
