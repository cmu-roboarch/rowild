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
#include <iterator>
#include <string>
#include "locobot.h"


unsigned int lfsr_random() {
    static unsigned int lfsr = 0xACE1u; // Initial seed
    unsigned int bit;

    bit  = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5)) & 1;
    return lfsr =  (lfsr >> 1) | (bit << 15);
}


float random_float_0_1() {
    // Generate an integer in the range 0 to UINT_MAX
    unsigned int rand_int = lfsr_random();

    // Scale and return as a float between 0 and 1
    return static_cast<float>(rand_int) / static_cast<float>(UINT_MAX);
}


LoCoBot::LoCoBot(Environment *_env) {
    this->env = _env;

    double I4[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

    for (int i = 0; i < 6; i++) {
        rpyxyzToH(this->rDesc[i][0], this->rDesc[i][1], this->rDesc[i][2], this->rDesc[i][3], this->rDesc[i][4], this->rDesc[i][5], this->TLink[i]);
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                TJoint[i][j][k] = I4[j][k];
                TCurr[i][j][k] = I4[j][k];
            }
        }
    }

    double temp[6] = {};
    this->forwardKin(temp);

    for (int i = 0; i < 4; i++) {
        rpyxyzToH(this->cDesc[i][0], this->cDesc[i][1], this->cDesc[i][2], this->cDesc[i][3], this->cDesc[i][4], this->cDesc[i][5], this->TBlock[i]);
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                TColl[i][j][k] = I4[j][k];
            }
        }
    }
}


void LoCoBot::planRRT(double (&start)[6], double (&goal)[6], double threshold, int maxSamples, double goalBias, int ppIters) {
    double rrtVertices[1000][6];
    int rrtEdges[1000];
    int numVertices = 0;
    int numEdges = 0;

    for (int i = 0; i < 6; i++) rrtVertices[0][i] = start[i];
    numVertices++;

    rrtEdges[0] = 0;
    numEdges++;

    auto findNearest = [&rrtVertices, &numVertices, this](double (&q)[6]) {
        int bestIdx = 0;
        double minDist = this->getNorm(rrtVertices[0], q);

        for (int idx = 1; idx < numVertices; idx++) {
            double d = this->getNorm(rrtVertices[idx], q);
            if (d < minDist) {
                bestIdx = idx;
                minDist = d;
            }
        }

        return bestIdx;
    };

    while (numVertices < maxSamples) {
        double qRand[6];
        sampleRobotCfg(qRand);
        if (random_float_0_1() < 0.05) for (int i = 0; i < 6; i++) qRand[i] = goal[i];

        int idxNear = findNearest(qRand);
        double qNear[6];
        for (int i = 0; i < 6; i++) qNear[i] = rrtVertices[idxNear][i];

        double qConnect[6];
        double randNearDist = this->getNorm(qRand, qNear);

        if (randNearDist > threshold) {
            double f = threshold / randNearDist;
            for (int i = 0; i < 6; i++) {
                qConnect[i] = qNear[i] + f * (qRand[i] - qNear[i]);
            }
        } else {
            for (int i = 0; i < 6; i++) qConnect[i] = qRand[i];
        }

        bool y = this->detectCollEdge(qConnect, qNear);
        if (!y) {
            for (int i = 0; i < 6; i++) rrtVertices[numVertices][i] = qConnect[i];
            numVertices++;

            rrtEdges[numEdges] = idxNear;
            numEdges++;
        }

        idxNear = findNearest(goal);
        if (this->getNorm(goal, rrtVertices[idxNear]) < threshold) {
            for (int i = 0; i < 6; i++) rrtVertices[numVertices][i] = goal[i];
            numVertices++;

            rrtEdges[numEdges] = idxNear;
            numEdges++;

            break;
        }
    }
}


void LoCoBot::sampleRobotCfg(double cfg[6]) {
    for (int i = 0; i < this->numJoints; i++) {
        double r = random_float_0_1();
        cfg[i] = this->qMin + (this->qMax - this->qMin) * r;
    }
}


bool LoCoBot::detectCollNode(double (&ang)[6]) {
    this->compCollBlockPoints(ang);

    auto pointsObs = this->env->getPointsObs();
    auto axesObs = this->env->getAxesObs();
    int numObstacles = 8;

    bool checks[32] = {};

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < numObstacles; j++) {
            checks[i*numObstacles + j] =
                cuboidCuboidCollision(this->cPoints[i], this->cAxes[i],
                        pointsObs[j], axesObs[j]);
        }
    }

    for (int i = 0; i < 4 * numObstacles; i++) {
        if (checks[i]) {
            return true;
        }
    }

    return false;
}


bool LoCoBot::detectCollEdge(double (&ang1)[6], double (&ang2)[6], int numSteps) {
    double stride = 1.0 / (numSteps - 1);
    for (double s = 0; s <= 1; s += stride) {
        double ang[6];
        for (size_t k = 0; k < 6; k++) {
            ang[k] = ang1[k] + s*(ang2[k] - ang1[k]);
        }

        if (this->detectCollNode(ang)) return true;
    }

    return false;
}


void LoCoBot::forwardKin(double (&ang)[6]) {
    for (int i = 0; i < this->numJoints; i++) {
        this->q[i] = ang[i];
    }

    auto isAxis = [this](int idx, double (&c)[3]) {
        for (int i = 0; i < 3; i++) {
            if (this->axis[idx][i] != c[i]) {
                return false;
            }
        }
        return true;
    };

    double temp1[3] = {0, 0, 1};
    double temp2[3] = {-1, 0, 0};
    double temp3[3] = {0, 1, 0};
    for (int i = 0; i < 6; i++) {
        if (isAxis(i, temp1)) {
            double t[4][4] = {
                {cos(this->q[i]), -sin(this->q[i]), 0, 0},
                {sin(this->q[i]), cos(this->q[i]), 0, 0},
                {0, 0, 1, 0},
                {0, 0, 0, 1}
            };

            for (int j = 0; j < 4; j++) {
                for (int k = 0; k < 4; k++) {
                    TJoint[i][j][k] = t[j][k];
                }
            }

        } else if (isAxis(i, temp2)) {
            double t[4][4] = {
                {1, 0, 0, 0},
                {0, cos(this->q[i]), sin(this->q[i]), 0},
                {0, -sin(this->q[i]), cos(this->q[i]), 0},
                {0, 0, 0, 1}
            };

            for (int j = 0; j < 4; j++) {
                for (int k = 0; k < 4; k++) {
                    TJoint[i][j][k] = t[j][k];
                }
            }

        } else if (isAxis(i, temp3)) {
            double t[4][4] = {
                {cos(this->q[i]), 0, sin(this->q[i]), 0},
                {0, 1, 0, 0},
                {-sin(this->q[i]), 0, cos(this->q[i]), 0},
                {0, 0, 0, 1}
            };

            for (int j = 0; j < 4; j++) {
                for (int k = 0; k < 4; k++) {
                    TJoint[i][j][k] = t[j][k];
                }
            }

        }

        if (i == 0) {
            matrixMultiplication<double, 4, 4, 4, 4>(this->TLink[i], this->TJoint[i], this->TCurr[i]);
        } else {
            double temp[4][4];
            matrixMultiplication<double, 4, 4, 4, 4>(this->TCurr[i-1], this->TLink[i], temp);
            matrixMultiplication<double, 4, 4, 4, 4>(temp, this->TJoint[i], TCurr[i]);
        }
    }
}


void LoCoBot::compCollBlockPoints(double (&ang)[6]) {
    this->forwardKin(ang);
    for (int i = 0; i < 4; i++) {
        int idx = this->cIdx[i];
        matrixMultiplication<double, 4, 4, 4, 4>(this->TCurr[idx], this->TBlock[i], this->TColl[i]);

        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                this->cAxes[i][j][k] = this->TColl[i][k][j];
            }
        }

        blockDescToBoundingBox(this->TColl[i], this->cDim[i], this->cPoints[i]);
    }
}


double LoCoBot::getNorm(double (&q1)[6], double (&q2)[6]) {
    double dist = 0;
    for (int i = 0; i < 6; i++) {
        dist += (q1[i] - q2[i]) * (q1[i] - q2[i]);
    }
    return sqrt(dist);
}
