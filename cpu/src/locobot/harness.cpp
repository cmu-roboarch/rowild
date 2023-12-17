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

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "args.h"
#include "env.h"
#include "locobot.h"
#include "log.h"
#include "rowild_utils.h"
#include "zsim_hooks.h"

void outputPath(std::string fileName, const JOINT_CFG &startCfg,
                const JOINT_CFG &goalCfg, std::vector<JOINT_CFG> const &path,
                double pathCost);

int main(int argc, const char **argv) {
    using args::KVArg;
    using args::Parser;

    Parser parser(argv[0], argc, argv, false);
    KVArg<std::string> inputMapArg(parser, "map", "", "Input map file");
    KVArg<std::string> plannerArg(parser, "planner", "",
                                  "Planner [PRM RRT RRTPostProc]");
    KVArg<double> thresholdArg(parser, "thresh", "",
                               "Threshold (epsilon and radius)");
    KVArg<int> samplesArg(parser, "samples", "",
                          "The maximum number of samples");
    KVArg<double> goalBiasArg(parser, "bias", "",
                              "Bias towards goal in RRT and RRT*");
    KVArg<int> iterationsArg(parser, "iterations", "",
                             "Post-processing iterations");
    KVArg<std::string> outputPathArg(parser, "output", "", "Output path file");

    if (!parser.parse()) assert(false);

    assert_msg(inputMapArg.found(), "Input map file is not provided");
    assert_msg(plannerArg.found(), "Planner is not provided");

    const char *inputFile = inputMapArg.value().c_str();
    const char *plannerType = plannerArg.value().c_str();
    double goalBias = goalBiasArg.found() ? goalBiasArg.value() : 0.05;
    int ppIters = iterationsArg.found() ? iterationsArg.value() : 150;
    const char *outputFile =
        outputPathArg.found() ? outputPathArg.value().c_str() : "/dev/null";

    JOINT_CFG start = {-1.3962634, 0, 0, 0, 0};
    JOINT_CFG goal = {0, 1.04719755, -1.30899694, -1.30899694, 0};

    const Environment *env = new Environment(inputFile);
    LoCoBot *l = new LoCoBot(env);

    std::vector<JOINT_CFG> path;

    if (strcmp(plannerType, "PRM") == 0) {
        double threshold = thresholdArg.found() ? thresholdArg.value() : 2.0;
        int samples = samplesArg.found() ? samplesArg.value() : 1000;
        l->initializePRM(threshold, samples);

        // ROI begins
        zsim_roi_begin();
        path = l->planPRM(start, goal);
        zsim_roi_end();
        // ROI ends

    } else if (strcmp(plannerType, "RRT") == 0 ||
               strcmp(plannerType, "RRTPostProc") == 0) {
        double threshold = thresholdArg.found() ? thresholdArg.value() : 0.25;
        int samples = samplesArg.found() ? samplesArg.value() : 3000;

        if (strcmp(plannerType, "RRT") == 0) {
            ppIters = 0; // Overwrite
        } else {
            assert(ppIters > 0);
        }

        // ROI begins
        zsim_roi_begin();
        path = l->planRRT(start, goal, threshold, samples, goalBias, ppIters);
        zsim_roi_end();
        // ROI ends

    } else {
        panic("Unknown planner: %s", plannerType);
    }

    outputPath(outputFile, start, goal, path, l->calcPathCost(path));

    delete env;
    delete l;

    return 0;
}

void outputPath(std::string fileName, const JOINT_CFG &startCfg,
                const JOINT_CFG &goalCfg, std::vector<JOINT_CFG> const &path,
                double cost) {
    auto printCfg = [](const JOINT_CFG &cfg) {
        std::string str = "";
        for (auto c : cfg)
            str += std::to_string(c) + " ";
        return str;
    };

    std::ofstream pathFile;
    pathFile.open(fileName);
    pathFile << "Start CFG: " << printCfg(startCfg) << std::endl;
    pathFile << "Goal CFG: " << printCfg(goalCfg) << std::endl;
    pathFile << "Path:" << std::endl;
    for (auto cfg : path) {
        pathFile << printCfg(cfg) << std::endl;
    }

    pathFile << "Cost: " << cost << std::endl;

    std::string dashLine = "--------------------------";
    pathFile << dashLine << dashLine << std::endl;
    pathFile.close();
}
