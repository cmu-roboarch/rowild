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
#include "move_controller.h"
#include "zsim_hooks.h"
#include <fstream>
#include <math.h>
#include <string>
#include <vector>

std::vector<STATE> theLog;

void loadLog(std::string inputFile) {
    std::ifstream logFile;
    logFile.open(inputFile);
    assert(logFile.good());

    std::string line;
    std::getline(logFile, line);
    assert(line == "X Y Theta");

    double x, y, theta;
    while (logFile >> x >> y >> theta) {
        theLog.push_back({x, y, theta});
    }

    logFile.close();
}

int main(int argc, const char **argv) {
    using args::KVArg;
    using args::Parser;

    Parser parser(argv[0], argc, argv, false);
    KVArg<std::string> logFileArg(parser, "log", "", "Input log file");
    KVArg<double> dtArg(parser, "dt", "", "Time step");
    KVArg<double> thresholdArg(parser, "threshold", "", "Close enough to goal");
    KVArg<double> maxLinSpeedArg(parser, "max-lin-speed", "",
                                 "Maximum linear speed");
    KVArg<double> maxAngSpeedArg(parser, "max-ang-speed", "",
                                 "Maximum angular speed");
    KVArg<std::string> outArg(parser, "output", "", "Output log file");

    if (!parser.parse()) assert(false);

    assert_msg(logFileArg.found(), "Input file is not provided");

    std::string logFileName = logFileArg.value();
    double dt = dtArg.found() ? dtArg.value() : 0.01;
    double threshold = thresholdArg.found() ? thresholdArg.value() : 0.001;
    double maxLinSpeed = maxLinSpeedArg.found() ? maxLinSpeedArg.value() : 15.0;
    double maxAngSpeed = maxAngSpeedArg.found() ? maxAngSpeedArg.value() : 7.0;
    std::string outputFile = outArg.found() ? outArg.value() : "/dev/null";

    loadLog(logFileName);
    assert(theLog.size() % 2 == 0);

    MoveController *ctrl =
        new MoveController(dt, threshold, maxLinSpeed, maxAngSpeed);
    std::vector<std::vector<std::pair<double, double>>> trajLog;

    // ROI begins
    zsim_roi_begin();
    for (int i = 0; i < static_cast<int>(theLog.size()); i += 2) {
        STATE start = theLog[i];
        STATE goal = theLog[i + 1];
        std::vector<std::pair<double, double>> traj =
            ctrl->getTrajectory(start, goal);
        trajLog.push_back(traj);
    }
    zsim_roi_end();
    // ROI ends

    // Write the output trajectory
    std::ofstream outTraj;
    outTraj.open(outputFile);
    for (std::vector<std::pair<double, double>> traj : trajLog) {
        for (std::pair<double, double> t : traj) {
            outTraj << t.first << " " << t.second << std::endl;
        }
        outTraj << "----------" << std::endl;
    }
    outTraj.close();

    return 0;
}
