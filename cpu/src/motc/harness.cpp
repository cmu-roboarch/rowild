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
#include "mpc.h"
#include "zsim_hooks.h"
#include <fstream>
#include <string>

TRAJ_VEC readInputCourse(std::string inputFile) {
    std::ifstream courseFile;
    courseFile.open(inputFile);
    assert(courseFile.good());

    TRAJ_VEC splineCourse;
    std::string line;
    std::getline(courseFile, line);
    assert(line == "X Y Yaw");

    double x, y, yaw;
    while (courseFile >> x >> y >> yaw) {
        splineCourse.push_back({x, y, yaw});
    }
    courseFile.close();

    return splineCourse;
}

int main(int argc, const char **argv) {
    using args::KVArg;
    using args::Parser;

    Parser parser(argv[0], argc, argv, false);
    KVArg<std::string> inputArg(parser, "input", "", "Input spline course");
    KVArg<double> timeArg(parser, "time", "", "Maximum simulation time");
    KVArg<double> tickArg(parser, "tick", "", "Time tick [s]");
    KVArg<double> distArg(parser, "dist", "", "Goal distance");
    KVArg<double> targetSpeedArg(parser, "tspeed", "", "Target speed");
    KVArg<double> stopSpeedArg(parser, "sspeed", "", "Stop speed");
    KVArg<int> itersArg(parser, "iters", "", "Maximum iterations");
    KVArg<int> neighborsArg(parser, "neighbors", "", "Number of neighbors");
    KVArg<std::string> outputArg(parser, "output", "", "Output trajectory");

    if (!parser.parse()) assert(false);

    assert_msg(inputArg.found(), "Input spline course is not provided");

    std::string inputFile = inputArg.value();
    double maxTime = timeArg.found() ? timeArg.value() : 500.0;
    double dt = tickArg.found() ? tickArg.value() : 0.2;
    double goalDistance = distArg.found() ? distArg.value() : 1.5;
    double targetSpeed =
        targetSpeedArg.found() ? targetSpeedArg.value() : 10.0 / 3.6;
    double stopSpeed = stopSpeedArg.found() ? stopSpeedArg.value() : 0.5 / 3.6;
    int maxIters = itersArg.found() ? itersArg.value() : 3;
    int neighbors = neighborsArg.found() ? neighborsArg.value() : 10;
    std::string outputFile =
        outputArg.found() ? outputArg.value() : "/dev/null";

    auto splineCourse = readInputCourse(inputFile);
    double dl = 1.0;

    ModelPredictiveControl *mpc =
        new ModelPredictiveControl(&splineCourse, targetSpeed, dt);

    // ROI begins
    zsim_roi_begin();

    auto traj = mpc->simulate(maxTime, goalDistance, stopSpeed, dl, maxIters,
                              neighbors);

    zsim_roi_end();
    // ROI ends

    // Write the output trajectory
    std::ofstream outTraj;
    outTraj.open(outputFile);
    for (auto s : *traj) {
        for (auto e : s) {
            outTraj << e << " ";
        }
        outTraj << std::endl;
    }
    outTraj.close();

    delete traj;
    delete mpc;

    return 0;
}
