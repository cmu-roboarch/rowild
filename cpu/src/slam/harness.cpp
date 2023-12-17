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
#include "ekf.h"
#include "fast.h"
#include "graph_based.h"
#include "log.h"
#include "slam.h"
#include "zsim_hooks.h"
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

std::vector<std::vector<double>> readLog(std::string inputLogFile) {
    std::ifstream logFile;
    logFile.open(inputLogFile);
    assert(logFile.good());

    std::vector<std::vector<double>> log;

    std::string line;
    while (std::getline(logFile, line)) {
        std::stringstream entrySS(line);

        std::vector<double> entry;
        double e;
        while (entrySS >> e) {
            entry.push_back(e);
        }

        log.push_back(entry);
    }
    logFile.close();

    return log;
}

int main(int argc, const char **argv) {
    using args::KVArg;
    using args::Parser;

    Parser parser(argv[0], argc, argv, false);
    KVArg<std::string> inputArg(parser, "log", "", "Input log file");
    KVArg<std::string> algorithmArg(parser, "alg", "",
                                    "SLAM algorithm [EKF Fast GraphBased]");
    KVArg<int> particlesArg(parser, "particles", "",
                            "Number of particles with FastSLAM");
    KVArg<std::string> outputArg(parser, "output", "", "Output log file");

    if (!parser.parse()) assert(false);

    assert_msg(inputArg.found(), "Input log file is not provided");
    assert_msg(algorithmArg.found(), "Input SLAM algorithm is not provided");

    std::string inputFile = inputArg.value();
    const char *slamAlgorithm = algorithmArg.value().c_str();
    int numParticles = particlesArg.found() ? particlesArg.value() : 100;
    std::string outputFile =
        outputArg.found() ? outputArg.value() : "/dev/null";

    // Uncertainty parameters
    double sigX2 = 0.25 * 0.25;
    double sigY2 = 0.1 * 0.1;
    double sigAlpha2 = 0.1 * 0.1;  // Rotation
    double sigBeta2 = 0.01 * 0.01; // Bearing angle
    double sigR2 = 0.08 * 0.08;    // Range

    auto inputLog = readLog(inputFile);

    std::vector<double> initialMeasurement = inputLog.front();
    int numLandmarks = static_cast<int>(initialMeasurement.size() / 2);
    inputLog.erase(inputLog.begin());

    SLAM *slam = nullptr;
    if (strcmp(slamAlgorithm, "EKF") == 0) {
        slam =
            new EKFSLAM(numLandmarks, sigX2, sigY2, sigAlpha2, sigBeta2, sigR2);
    } else if (strcmp(slamAlgorithm, "Fast") == 0) {
        slam = new FastSLAM(numParticles, numLandmarks, sigX2, sigY2, sigAlpha2,
                            sigBeta2, sigR2);
    } else if (strcmp(slamAlgorithm, "GraphBased") == 0) {
        slam = new GraphBasedSLAM(numLandmarks);
    } else {
        panic("Unknown algorithm: %s", slamAlgorithm);
    }

    std::vector<std::vector<double>> outputLog;

    // ROI begins
    zsim_roi_begin();

    outputLog.push_back(slam->getStatus());
    for (auto input : inputLog) {
        if (input.size() == 2) {
            // Control input
            slam->motionUpdate(input[0], input[1], input[2]);
        } else if (input.size() == 2 * static_cast<size_t>(numLandmarks)) {
            // Measurement input
            slam->measurementUpdate(input);
        } else {
            assert(false);
        }

        outputLog.push_back(slam->getStatus());
    }

    zsim_roi_end();
    // ROI ends

    // Write the output log
    std::ofstream outLogFile;
    outLogFile.open(outputFile);
    for (auto l : outputLog) {
        for (auto e : l) {
            outLogFile << std::setprecision(4) << e << " ";
        }
        outLogFile << std::endl;
    }
    outLogFile.close();
    delete slam;

    return 0;
}
