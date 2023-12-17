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
#include "coin/IpIpoptApplication.hpp"
#include "drone.h"
#include "log.h"
#include "mpc.h"
#include <fstream>
#include <iostream>
#include <stddef.h>
#include <string>

std::vector<STATE> readDestinations(std::string fileName) {
    std::ifstream file(fileName);
    assert(file.good());

    std::vector<STATE> dests;

    int x, y, z;
    while (file >> x >> y >> z) {
        dests.push_back({x, y, z});
    }

    return dests;
}

int main(int argc, const char **argv) {
    using args::FlagArg;
    using args::KVArg;
    using args::Parser;

    Parser parser(argv[0], argc, argv, false);
    KVArg<std::string> inputMapArg(parser, "map", "", "Input map file");
    KVArg<std::string> destinationsFileArg(parser, "destinations", "",
                                           "File including the destinations");
    KVArg<double> heuristicWeightArg(parser, "weight", "",
                                     "Heuristic weight of A*");
    KVArg<std::string> aStarHeuristicArg(parser, "heuristic", "",
                                         "A* heuristic");
    FlagArg deepeningArg(parser, "deepening", "", "Enable IDA*");
    KVArg<uint64_t> maxIterationsArg(parser, "max-iters", "",
                                     "Maximum iterations");
    KVArg<int> ctrlStepsArg(
        parser, "step", "",
        "Step length in MPC run for consecutive points in the path");
    KVArg<std::string> outputPathArg(parser, "output", "", "Output path file");

    if (!parser.parse()) assert(false);

    assert_msg(inputMapArg.found(), "Input map file is not provided");
    assert_msg(destinationsFileArg.found(), "Destiations file is not provided");

    std::string inputMapFile = inputMapArg.value();
    std::string destsFile = destinationsFileArg.value();
    double heuristicWeight =
        heuristicWeightArg.found() ? heuristicWeightArg.value() : 4.0;
    std::string aStarHeuristicStr =
        aStarHeuristicArg.found() ? aStarHeuristicArg.value() : "Euclidean";
    bool idastar = deepeningArg.found();
    uint64_t maxIterations =
        maxIterationsArg.found() ? maxIterationsArg.value() : 100'000'000;
    int jStep = ctrlStepsArg.found() ? ctrlStepsArg.value() : 10;
    std::string outputFile =
        outputPathArg.found() ? outputPathArg.value() : "/dev/null";

    std::vector<PATH> pathsLog;
    auto locations = readDestinations(destsFile);

    Drone *flybot =
        new Drone(inputMapFile, aStarHeuristicStr, heuristicWeight, idastar);

    auto convertState = [](const std::vector<int> &v) {
        std::vector<double> cv;
        cv.reserve(v.size());
        for (int e : v) {
            cv.push_back(static_cast<double>(e));
        }

        return cv;
    };

    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

    // The solver parameters
    app->Options()->SetStringValue("hessian_approximation", "limited-memory");
    app->Options()->SetIntegerValue("print_level", 0);
    app->Options()->SetIntegerValue("max_iter", 2);
    app->Options()->SetNumericValue("tol", 1e-1);
    app->Options()->SetStringValue("linear_solver", "mumps");
    app->Options()->SetStringValue("derivative_test", "none");

    app->Initialize();

    // ROI begins
    for (size_t i = 1; i < locations.size(); i++) {
        STATE currentStart = locations[i - 1];
        STATE currentGoal = locations[i];
        std::vector<int> path =
            flybot->run(currentStart, currentGoal, maxIterations);

        STATE currState = currentStart, nextState;
        for (size_t j = 0; j < path.size(); j += jStep) {
            nextState = flybot->applyMovement(currState, path[j]);

            SmartPtr<TNLP> myNlp = new MpcController(convertState(currState),
                                                     convertState(nextState));
            app->OptimizeTNLP(myNlp);

            currState = nextState;
        }

        pathsLog.push_back(path);
    }
    // ROI ends

    // Write the output path
    std::ofstream pathFile;
    pathFile.open(outputFile);
    for (auto path : pathsLog) {
        for (auto dir : path) {
            pathFile << dir << std::endl;
        }
        pathFile << std::string(20, '-') << std::endl;
    }
    pathFile.close();

    return 0;
}
