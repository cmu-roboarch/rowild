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
#include "arm.h"
#include "planner.h"
#include "prm.h"
#include "rrt.h"
#include "rrt_star.h"
#include "zsim_hooks.h"
#include <string>
#include <vector>

#define PRINT_SEVERITY (2)

void outputPath(std::string fileName, double *startCfg, double *goalCfg,
                std::vector<double *> const &path, int dof, double cost);

int main(int argc, const char **argv) {
    using args::KVArg;
    using args::Parser;

    Parser parser(argv[0], argc, argv, false);
    KVArg<std::string> inputMapArg(parser, "map", "", "Input map file");
    KVArg<std::string> inputConfigArg(parser, "config", "",
                                      "Input config file");
    KVArg<std::string> plannerArg(parser, "planner", "",
                                  "Planner [PRM RRT RRTStar RRTPostProc]");
    KVArg<double> epsilonArg(
        parser, "epsilon", "",
        "Epsilon (close enough to the goal & minimum movement)");
    KVArg<double> radiusArg(parser, "radius", "", "Neighborhood distance");
    KVArg<int> samplesArg(parser, "samples", "",
                          "The maximum number of samples");
    KVArg<int> aStarWeightArg(parser, "weight", "", "A* weight for PRM");
    KVArg<double> goalBiasArg(parser, "bias", "",
                              "Bias towards goal in RRT and RRT*");
    KVArg<int> iterationsArg(parser, "iterations", "",
                             "Post-processing iterations");
    KVArg<std::string> outputPathArg(parser, "output", "", "Output path file");

    if (!parser.parse()) assert(false);

    assert_msg(inputMapArg.found(), "Input map file is not provided");
    assert_msg(inputConfigArg.found(), "Input config file is not provided");
    assert_msg(plannerArg.found(), "Planner is not provided");

    const char *inputFile = inputMapArg.value().c_str();
    const char *configFile = inputConfigArg.value().c_str();
    const char *plannerType = plannerArg.value().c_str();
    double epsilon = epsilonArg.found() ? epsilonArg.value() : PI / 20;
    double radius = radiusArg.found() ? radiusArg.value() : PI / 3;
    int samples = samplesArg.found() ? samplesArg.value() : 10000;
    int aStarWeight = aStarWeightArg.found() ? aStarWeightArg.value() : 1;
    double goalBias = goalBiasArg.found() ? goalBiasArg.value() : 0.05;
    int ppIterations = iterationsArg.found() ? iterationsArg.value() : 100;
    const char *outputFile =
        outputPathArg.found() ? outputPathArg.value().c_str() : "/dev/null";

    Environment *env = new Environment(inputFile);
    Arm *arm = new Arm(configFile);

    int armDof = arm->getDof();
    int armLinkLength = arm->getLinkLength();

    Planner *thePlanner = nullptr;
    if (strcmp(plannerType, "PRM") == 0) {
        thePlanner = new PRM(env, armDof, armLinkLength, epsilon, radius,
                             samples, aStarWeight);
    } else if (strcmp(plannerType, "RRT") == 0) {
        thePlanner = new RRT(env, armDof, armLinkLength, epsilon, radius,
                             samples, goalBias, 0 /*No post-processing*/);
    } else if (strcmp(plannerType, "RRTStar") == 0) {
        thePlanner = new RRTStar(env, armDof, armLinkLength, epsilon, radius,
                                 samples, goalBias);
    } else if (strcmp(plannerType, "RRTPostProc") == 0) {
        thePlanner = new RRT(env, armDof, armLinkLength, epsilon, radius,
                             samples, goalBias, ppIterations);
    } else {
        assert_msg(false, "Unknown planner: %s", plannerType);
    }

    // Clear the output path file before writing
    std::ofstream pathFile;
    pathFile.open(outputFile, std::ios_base::out | std::ios_base::trunc);
    pathFile << "Planner: " << plannerType << std::endl;
    pathFile.close();

    // Stats
    double __totalTime = 0;
    uint64_t __numQueries = 0;
    using std::chrono::duration_cast;
    using std::chrono::high_resolution_clock;
    using std::chrono::nanoseconds;

    for (int i = 0; i < static_cast<int>(arm->getGoalCfgs().size()); i++) {
        double *startCfg = arm->getStartCfgs()[i];
        double *goalCfg = arm->getGoalCfgs()[i];

        auto t0 = high_resolution_clock::now();

        // ROI begins
        zsim_roi_begin();

        std::vector<double *> path = thePlanner->query(startCfg, goalCfg);

        zsim_roi_end();
        // ROI ends

        auto t1 = high_resolution_clock::now();
        __totalTime += duration_cast<nanoseconds>(t1 - t0).count() * 1e-9;
        __numQueries++;

        // query() is supposed to return a path from goal to start; the path
        // should be reversed before sending to actuators. This is simply an
        // implementation choice; this way, I put the 'reversing' operations
        // off the critical path
        std::reverse(path.begin(), path.end());
        outputPath(outputFile, startCfg, goalCfg, path, armDof,
                   thePlanner->calcPathCost(path));

        // The allocated memory for PRM cfgs should not be deleted as they are
        // used for the next iterations.
        if (strcmp(plannerType, "PRM") != 0) {
            for (double *p : path)
                delete[] p;
        }
    }

    auto printStats = [__totalTime, __numQueries](int severity) {
        if (severity >= 1) {
            std::cout << "[Stats]"
                      << "  timePerQuery=" << __totalTime / __numQueries
                      << std::endl;
        }

        if (severity >= 2) {
            std::cout << "[Detailed Stats]"
                      << "  totalTime=" << __totalTime
                      << ", numQueries=" << __numQueries << std::endl;
        }
    };

    delete env;
    delete arm;
    delete thePlanner;

    printStats(PRINT_SEVERITY);

    return 0;
}

void outputPath(std::string fileName, double *startCfg, double *goalCfg,
                std::vector<double *> const &path, int dof, double cost) {
    auto printCfg = [dof](double *cfg) {
        std::string str = "";
        for (int i = 0; i < dof; i++)
            str += std::to_string(cfg[i]) + " ";
        return str;
    };

    std::ofstream pathFile;
    pathFile.open(fileName, std::ios_base::app);
    std::string dashLine = "--------------------------";
    pathFile << dashLine << dashLine << std::endl;
    pathFile << "Start CFG: " << printCfg(startCfg) << std::endl;
    pathFile << "Goal CFG: " << printCfg(goalCfg) << std::endl;
    pathFile << "Path:" << std::endl;
    for (const auto &cfg : path) {
        pathFile << printCfg(cfg) << std::endl;
    }
    pathFile << "Cost: " << cost << std::endl;
    pathFile << dashLine << dashLine << std::endl;
    pathFile.close();
}
