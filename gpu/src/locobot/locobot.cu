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
#include <cuda_runtime.h>
#include <curand_kernel.h>
#include <iterator>
#include <string>
#include <vector>

#include "locobot.h"
#include "rowild_utils.h"

#define NUM_BLOCKS 32
#define THREADS_PER_BLOCK 32
#define MAX_NODES 1000
#define DOF 5

struct Node {
    float angles[DOF];
    int parent;
};

__device__ void computeForwardKin(const float q[DOF], float TCurr[6][4][4],
                                  const float TLink[6][4][4]) {

    float axis[6][3]{{0, 0, 1}, {0, 1, 0},  {0, 1, 0},
                     {0, 1, 0}, {-1, 0, 0}, {0, 1, 0}};

    for (int i = 0; i < 6; i++) {
        float TJoint[4][4];

        if (axis[i][0] == -1 && axis[i][1] == 0 && axis[i][2] == 0) {
            // Rotation about the x-axis
            TJoint[0][0] = 1;
            TJoint[0][1] = 0;
            TJoint[0][2] = 0;
            TJoint[0][3] = 0;
            TJoint[1][0] = 0;
            TJoint[1][1] = cosf(q[i]);
            TJoint[1][2] = sinf(q[i]);
            TJoint[1][3] = 0;
            TJoint[2][0] = 0;
            TJoint[2][1] = -sinf(q[i]);
            TJoint[2][2] = cosf(q[i]);
            TJoint[2][3] = 0;
            TJoint[3][0] = 0;
            TJoint[3][1] = 0;
            TJoint[3][2] = 0;
            TJoint[3][3] = 1;

        } else if (axis[i][0] == 0 && axis[i][1] == 1 && axis[i][2] == 0) {
            // Rotation about the y-axis
            TJoint[0][0] = cosf(q[i]);
            TJoint[0][1] = 0;
            TJoint[0][2] = sinf(q[i]);
            TJoint[0][3] = 0;
            TJoint[1][0] = 0;
            TJoint[1][1] = 1;
            TJoint[1][2] = 0;
            TJoint[1][3] = 0;
            TJoint[2][0] = -sinf(q[i]);
            TJoint[2][1] = 0;
            TJoint[2][2] = cosf(q[i]);
            TJoint[2][3] = 0;
            TJoint[3][0] = 0;
            TJoint[3][1] = 0;
            TJoint[3][2] = 0;
            TJoint[3][3] = 1;

        } else {
            // Rotation about the z-axis
            TJoint[0][0] = cosf(q[i]);
            TJoint[0][1] = -sinf(q[i]);
            TJoint[0][2] = 0;
            TJoint[0][3] = 0;
            TJoint[1][0] = sinf(q[i]);
            TJoint[1][1] = cosf(q[i]);
            TJoint[1][2] = 0;
            TJoint[1][3] = 0;
            TJoint[2][0] = 0;
            TJoint[2][1] = 0;
            TJoint[2][2] = 1;
            TJoint[2][3] = 0;
            TJoint[3][0] = 0;
            TJoint[3][1] = 0;
            TJoint[3][2] = 0;
            TJoint[3][3] = 1;
        }

        float temp[4][4];
        if (i == 0) {
            sqMatMult<float, 4>(TLink[0], TJoint, TCurr[0]);
        } else {
            sqMatMult<float, 4>(TCurr[i - 1], TLink[i], temp);
            sqMatMult<float, 4>(temp, TJoint, TCurr[i]);
        }
    }
}

__device__ bool checkCollision(float (*TLink)[4][4], float (*TJoint)[4][4],
                               float (*TCurr)[4][4], float (*TBlock)[4][4],
                               float (*TColl)[4][4], float (*cPoints)[9][3],
                               float (*cAxes)[3][3], const Node &newNode,
                               float *pointsObs, float *axesObs,
                               int numObstacles) {
    computeForwardKin(newNode.angles, TCurr, TLink);

    // Dimensions of robot arm blocks (LWH of blocks)
    float cDim[4][3] = {{0.05, 0.05, 0.25},
                        {0.25, 0.05, 0.05},
                        {0.07, 0.076, 0.05},
                        {0.11, 0.11, 0.07}};

    // Compute current collision boxes for the arm
    for (int i = 0; i < 4; i++) {
        sqMatMult<float, 4>(TCurr[i + 1], TBlock[i], TColl[i]);

        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                cAxes[i][j][k] = TColl[i][k][j];
            }
        }

        blockDescToBoundingBox<float>(TColl[i], cDim[i], cPoints[i]);
    }

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < numObstacles; j++) {
            if (cuboidCuboidCollision<float>(
                    cPoints[i], cAxes[i], (float(*)[3]) & pointsObs[j * 3 * 9],
                    (float(*)[3]) & axesObs[j * 3 * 3])) {
                return true;
            }
        }
    }

    return false;
}

__global__ void rrtKernel(float (*TLink)[4][4], float (*TJoint)[4][4],
                          float (*TCurr)[4][4], float (*TBlock)[4][4],
                          float (*TColl)[4][4], float (*cPoints)[9][3],
                          float (*cAxes)[3][3], Node *nodes,
                          int *currentNumNodes, float *randAngles,
                          Node goalNode, float goalThreshold, bool *reached,
                          float goalBiasProbability, float *pointsObs,
                          float *axesObs, int numObstacles) {
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx >= *currentNumNodes) return;

    curandState state;
    curand_init(idx, idx, 0, &state);

    float prob = curand_uniform(&state);
    if (prob < goalBiasProbability) {
        for (int d = 0; d < DOF; ++d) {
            randAngles[idx * DOF + d] = goalNode.angles[d];
        }
    } else {
        for (int d = 0; d < DOF; ++d) {
            randAngles[idx * DOF + d] = (curand_uniform(&state) - 0.5) * M_PI;
        }
    }

    float minDist = 1e10f;
    int minIdx = -1;
    for (int i = 0; i < *currentNumNodes; ++i) {
        float dist = 0.0f;
        for (int d = 0; d < DOF; ++d) {
            float delta = nodes[i].angles[d] - randAngles[idx * DOF + d];
            dist += delta * delta;
        }
        if (dist < minDist) {
            minDist = dist;
            minIdx = i;
        }
    }

    Node newNode;
    newNode.parent = minIdx;
    for (int d = 0; d < DOF; ++d) {
        float angleDiff = randAngles[idx * DOF + d] - nodes[minIdx].angles[d];
        newNode.angles[d] = nodes[minIdx].angles[d] + (1.0f * angleDiff);
    }

    if (!checkCollision(TLink, TJoint, TCurr, TBlock, TColl, cPoints, cAxes,
                        newNode, pointsObs, axesObs, numObstacles)) {
        int index = atomicAdd(currentNumNodes, 1);
        nodes[index] = newNode;

        if (!*reached) {
            float distToGoal = 0.0f;
            for (int d = 0; d < DOF; ++d) {
                float delta = newNode.angles[d] - goalNode.angles[d];
                distToGoal += delta * delta;
            }
            if (distToGoal < goalThreshold * goalThreshold) {
                *reached = true;
            }
        }
    }
}

LoCoBot::LoCoBot(const Environment *_env) {
    this->env = _env;

    float I4[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

    // Robot descriptor taken from URDF file
    // rpy xyz for each rigid link transform
    float rDesc[6][6] = {
        {0, 0, 0, 0.08, 0, 0.159}, // From robot base to joint1
        {0, 0, 0, 0, 0, 0.04125},         {0, 0, 0, 0.05, 0, 0.2},
        {0, 0, 0, 0.2002, 0, 0},          {0, 0, 0, 0.063, 0.0001, 0},
        {0, 0, 0, 0.106525, 0, 0.0050143} // From joint5 to end-eff. center
    };

    for (int i = 0; i < 6; i++) {
        rpyxyzToH(rDesc[i][0], rDesc[i][1], rDesc[i][2], rDesc[i][3],
                  rDesc[i][4], rDesc[i][5], this->TLink[i]);
        std::copy(&I4[0][0], &I4[0][0] + 16, &this->TJoint[i][0][0]);
        std::copy(&I4[0][0], &I4[0][0] + 16, &this->TCurr[i][0][0]);
    }

    double axis[6][3]{{0, 0, 1}, {0, 1, 0},  {0, 1, 0},
                      {0, 1, 0}, {-1, 0, 0}, {0, 1, 0}};
    auto isAxis = [&axis](int idx, const double(&c)[3]) {
        return std::equal(std::begin(axis[idx]), std::end(axis[idx]),
                          std::begin(c));
    };

    for (int i = 0; i < 6; i++) {
        if (isAxis(i, {0, 0, 1})) {
            float t[4][4] = {{cosf(0), -sinf(0), 0.0f, 0.0f},
                             {sinf(0), cosf(0), 0.0f, 0.0f},
                             {0.0f, 0.0f, 1, 0.0f},
                             {0.0f, 0.0f, 0.0f, 1}};
            std::copy(&t[0][0], &t[0][0] + 16, &this->TJoint[i][0][0]);
        } else if (isAxis(i, {-1, 0, 0})) {
            float t[4][4] = {{1, 0.0f, 0.0f, 0.0f},
                             {0.0f, cosf(0), sinf(0), 0.0f},
                             {0.0f, -sinf(0), cosf(0), 0.0f},
                             {0.0f, 0.0f, 0.0f, 1}};
            std::copy(&t[0][0], &t[0][0] + 16, &this->TJoint[i][0][0]);
        } else if (isAxis(i, {0, 1, 0})) {
            float t[4][4] = {{cosf(0), 0.0f, sinf(0), 0.0f},
                             {0.0f, 1, 0.0f, 0.0f},
                             {-sinf(0), 0.0f, cosf(0), 0.0f},
                             {0.0f, 0.0f, 0.0f, 1}};
            std::copy(&t[0][0], &t[0][0] + 16, &this->TJoint[i][0][0]);
        } else {
            panic("Axis rotation is not defined");
        }

        if (i == 0) {
            cpuMatMul<float, 4, 4, 4, 4>(this->TLink[i], this->TJoint[i],
                                         this->TCurr[i]);
        } else {
            float temp[4][4];
            cpuMatMul<float, 4, 4, 4, 4>(this->TCurr[i - 1], this->TLink[i],
                                         temp);
            cpuMatMul<float, 4, 4, 4, 4>(temp, this->TJoint[i], TCurr[i]);
        }
    }

    // rpy xyz poses of the robot arm blocks
    float cDesc[4][6] = {
        {0, 0, 0, 0, 0, 0.09},
        {0, 0, 0, 0.075, 0, 0},
        {0, 0, 0, 0.027, -0.012, 0},
        {0, 0, 0, 0.055, 0, 0.01},
    };

    for (int i = 0; i < 4; i++) {
        rpyxyzToH(cDesc[i][0], cDesc[i][1], cDesc[i][2], cDesc[i][3],
                  cDesc[i][4], cDesc[i][5], this->TBlock[i]);
        std::copy(&I4[0][0], &I4[0][0] + 16, &this->TColl[i][0][0]);
    }
}

std::vector<JOINT_CFG> LoCoBot::planRRT(const JOINT_CFG &start,
                                        const JOINT_CFG &goal, float threshold,
                                        float goalBias) {
    Node *d_nodes;
    float *d_randAngles;
    int *d_numNodes;
    bool *d_reached;

    int h_numNodes = 1;
    bool h_reached = false;

    cudaMalloc(&d_nodes, sizeof(Node) * MAX_NODES);
    cudaMalloc(&d_randAngles,
               sizeof(float) * DOF * NUM_BLOCKS * THREADS_PER_BLOCK);
    cudaMalloc(&d_numNodes, sizeof(int));
    cudaMalloc(&d_reached, sizeof(bool));

    cudaMemcpy(d_numNodes, &h_numNodes, sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_reached, &h_reached, sizeof(bool), cudaMemcpyHostToDevice);

    Node h_node;
    h_node.parent = -1;
    for (int d = 0; d < DOF; ++d)
        h_node.angles[d] = start.at(d);
    cudaMemcpy(d_nodes, &h_node, sizeof(Node), cudaMemcpyHostToDevice);

    Node goalNode;
    for (int d = 0; d < DOF; ++d)
        goalNode.angles[d] = goal.at(d);

    float(*d_TLink)[4][4], (*d_TJoint)[4][4], (*d_TCurr)[4][4];
    float(*d_TBlock)[4][4], (*d_TColl)[4][4], (*d_cPoints)[9][3],
        (*d_cAxes)[3][3];

    cudaMalloc((void **)&d_TLink, sizeof(this->TLink));
    cudaMalloc((void **)&d_TJoint, sizeof(this->TJoint));
    cudaMalloc((void **)&d_TCurr, sizeof(this->TCurr));
    cudaMalloc((void **)&d_TBlock, sizeof(this->TBlock));
    cudaMalloc((void **)&d_TColl, sizeof(this->TColl));
    cudaMalloc((void **)&d_cPoints, sizeof(this->cPoints));
    cudaMalloc((void **)&d_cAxes, sizeof(this->cAxes));

    cudaMemcpy(d_TLink, this->TLink, sizeof(this->TLink),
               cudaMemcpyHostToDevice);
    cudaMemcpy(d_TJoint, this->TJoint, sizeof(this->TJoint),
               cudaMemcpyHostToDevice);
    cudaMemcpy(d_TCurr, this->TCurr, sizeof(this->TCurr),
               cudaMemcpyHostToDevice);
    cudaMemcpy(d_TBlock, this->TBlock, sizeof(this->TBlock),
               cudaMemcpyHostToDevice);
    cudaMemcpy(d_TColl, this->TColl, sizeof(this->TColl),
               cudaMemcpyHostToDevice);
    cudaMemcpy(d_cPoints, this->cPoints, sizeof(this->cPoints),
               cudaMemcpyHostToDevice);
    cudaMemcpy(d_cAxes, this->cAxes, sizeof(this->cAxes),
               cudaMemcpyHostToDevice);

    auto h_pointsObs = this->env->getPointsObs();
    auto h_axesObs = this->env->getAxesObs();
    assert(h_pointsObs.size() == h_axesObs.size());
    int numObstacles = static_cast<int>(h_axesObs.size());

    int pObsSize = numObstacles * 9 * 3;
    float *h_flattenedPointsObs = new float[pObsSize];
    for (int i = 0; i < numObstacles; i++) {
        for (int j = 0; j < 9; j++) {
            for (int k = 0; k < 3; k++) {
                h_flattenedPointsObs[i * 3 * 9 + j * 3 + k] =
                    h_pointsObs[i].point[j][k];
            }
        }
    }
    float *d_flattenedPointsObs;
    cudaMalloc(&d_flattenedPointsObs, pObsSize * sizeof(float));
    cudaMemcpy(d_flattenedPointsObs, h_flattenedPointsObs,
               pObsSize * sizeof(float), cudaMemcpyHostToDevice);

    int aObsSize = numObstacles * 3 * 3;
    float *h_flattenedAxesObs = new float[aObsSize];
    for (int i = 0; i < numObstacles; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                h_flattenedAxesObs[i * 3 * 3 + j * 3 + k] =
                    h_axesObs[i].axes[j][k];
            }
        }
    }
    float *d_flattenedAxesObs;
    cudaMalloc(&d_flattenedAxesObs, aObsSize * sizeof(float));
    cudaMemcpy(d_flattenedAxesObs, h_flattenedAxesObs, aObsSize * sizeof(float),
               cudaMemcpyHostToDevice);

    while (!h_reached && h_numNodes < MAX_NODES) {
        rrtKernel<<<NUM_BLOCKS, THREADS_PER_BLOCK>>>(
            d_TLink, d_TJoint, d_TCurr, d_TBlock, d_TColl, d_cPoints, d_cAxes,
            d_nodes, d_numNodes, d_randAngles, goalNode, threshold, d_reached,
            goalBias, d_flattenedPointsObs, d_flattenedAxesObs, numObstacles);

        cudaDeviceSynchronize();
        cudaMemcpy(&h_reached, d_reached, sizeof(bool), cudaMemcpyDeviceToHost);
        cudaMemcpy(&h_numNodes, d_numNodes, sizeof(int),
                   cudaMemcpyDeviceToHost);
    }

    Node *h_nodes = new Node[MAX_NODES];
    cudaMemcpy(h_nodes, d_nodes, sizeof(Node) * MAX_NODES,
               cudaMemcpyDeviceToHost);

    std::vector<JOINT_CFG> path;
    int currentIndex = h_numNodes - 1;
    while (currentIndex != -1) {
        path.push_back(std::vector<float>(h_nodes[currentIndex].angles,
                                          h_nodes[currentIndex].angles + DOF));
        currentIndex = h_nodes[currentIndex].parent;
    }

    delete[] h_nodes;
    delete[] h_flattenedPointsObs;
    delete[] h_flattenedAxesObs;

    cudaFree(d_TLink);
    cudaFree(d_TJoint);
    cudaFree(d_TCurr);
    cudaFree(d_TBlock);
    cudaFree(d_TColl);
    cudaFree(d_cPoints);
    cudaFree(d_cAxes);
    cudaFree(d_nodes);
    cudaFree(d_randAngles);
    cudaFree(d_numNodes);
    cudaFree(d_reached);
    cudaFree(d_flattenedPointsObs);
    cudaFree(d_flattenedAxesObs);

    return path;
}
