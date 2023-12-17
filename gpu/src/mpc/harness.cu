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
#include "log.h"
#include "rowild_utils.h"
#include "timer.h"
#include <cuda_runtime.h>
#include <fstream>
#include <iomanip>
#include <iostream>

// Drone Parameters
const float gGravity = 9.81;

__constant__ float goal[3] = {10.0, 10.0, 10.0};

__device__ void droneDynamics(float *state, const float *control,
                              const float dt) {
    state[0] += dt * state[3];
    state[1] += dt * state[4];
    state[2] += dt * state[5];
    state[3] += dt * (control[0] - gGravity);
    state[4] += dt * control[1];
    state[5] += dt * control[2];
}

__device__ float computeCost(const float *state) {
    return sqrtf((goal[0] - state[0]) * (goal[0] - state[0]) +
                 (goal[1] - state[1]) * (goal[1] - state[1]) +
                 (goal[2] - state[2]) * (goal[2] - state[2]));
}

__global__ void mpcKernel(float *controls, const float *initialState, int n,
                          float dt) {
    int idx = threadIdx.x;
    float state[6];
    memcpy(state, initialState, 6 * sizeof(float));

    float(*controlSequence)[3] = (float(*)[3])malloc(n * 3 * sizeof(float));

    // Initialize control sequence with current controls
    for (int i = 0; i < n; i++) {
        memcpy(controlSequence[i], &controls[3 * idx], 3 * sizeof(float));
    }

    float cost = 0.0;
    for (int i = 0; i < n; i++) {
        droneDynamics(state, controlSequence[i], dt);
        cost += computeCost(state);
    }

    // Gradient descent
    float alpha = 0.01;
    for (int iter = 0; iter < 100; iter++) {
        for (int i = 0; i < n; i++) {
            for (int dim = 0; dim < 3; dim++) {
                controlSequence[i][dim] -= alpha * (cost / n);
            }
        }
    }

    memcpy(&controls[3 * idx], controlSequence[0], 3 * sizeof(float));
    free(controlSequence);
}

int main(int argc, const char **argv) {
    using args::FlagArg;
    using args::KVArg;
    using args::Parser;

    Parser parser(argv[0], argc, argv, false);
    KVArg<int> horizonArg(parser, "horizon", "", "Prediction horizon");
    KVArg<float> timeStepArg(parser, "dt", "", "Time step (tick)");
    KVArg<std::string> outputArg(parser, "output", "", "Output log file");

    if (!parser.parse()) assert(false);

    int n = horizonArg.found() ? horizonArg.value() : 10;
    float dt = timeStepArg.found() ? timeStepArg.value() : 0.1;
    std::string outputFile =
        outputArg.found() ? outputArg.value() : "/dev/null";

    float hInitialState[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float *dInitialState;

    cudaMalloc((void **)&dInitialState, 6 * sizeof(float));
    cudaMemcpy(dInitialState, hInitialState, 6 * sizeof(float),
               cudaMemcpyHostToDevice);

    // Assume we're testing 1024 different initial control sequences
    float hControls[1024 * 3];
    float *dControls;
    cudaMalloc((void **)&dControls, 1024 * 3 * sizeof(float));
    cudaMemcpy(dControls, hControls, 1024 * 3 * sizeof(float),
               cudaMemcpyHostToDevice);

    timer t;

    // ROI begins
    mpcKernel<<<1, 1024>>>(dControls, dInitialState, n, dt);
    // ROI ends

    std::cout << "Execution time: " << t.elapsed() << std::endl;

    cudaMemcpy(hControls, dControls, 1024 * 3 * sizeof(float),
               cudaMemcpyDeviceToHost);

    cudaFree(dControls);
    cudaFree(dInitialState);

    // Write the output log
    std::ofstream outLogFile;
    outLogFile.open(outputFile);
    for (int i = 0; i < 1024; i++) {
        outLogFile << std::setprecision(3) << hControls[i + 0] << ", "
                   << hControls[i + 1] << ", " << hControls[i + 2] << std::endl;
    }
    outLogFile.close();

    return 0;
}
