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
#include "pid.h"
#include "zsim_hooks.h"
#include <fstream>
#include <math.h>
#include <string>
#include <vector>

std::vector<std::pair<double, double>> readInputFile(const char *fileName) {
    std::vector<std::pair<double, double>> valPairs;

    std::ifstream mFile;
    mFile.open(fileName);
    assert(mFile.good());

    std::string line;
    while (std::getline(mFile, line)) {
        double ang, val;
        std::stringstream ss(line);
        ss >> ang >> val;
        valPairs.push_back(std::make_pair(ang, val));
    }

    mFile.close();

    return valPairs;
}

int main(int argc, const char **argv) {
    using args::KVArg;
    using args::Parser;

    Parser parser(argv[0], argc, argv, false);
    KVArg<std::string> logFileArg(parser, "log", "", "Input log file");
    KVArg<double> kpArg(parser, "kp", "", "Proportional gain");
    KVArg<double> kiArg(parser, "ki", "", "Integral gain");
    KVArg<double> kdArg(parser, "kd", "", "Derivative gain");
    KVArg<std::string> outArg(parser, "output", "", "Output log file");

    if (!parser.parse()) assert(false);

    assert_msg(logFileArg.found(), "Input file is not provided");

    const char *logFileName = logFileArg.value().c_str();
    double kp = kpArg.found() ? kpArg.value() : 1.0;
    double ki = kiArg.found() ? kiArg.value() : 0.1;
    double kd = kdArg.found() ? kdArg.value() : 0.01;
    std::string outputFile = outArg.found() ? outArg.value() : "/dev/null";

    auto lVals = readInputFile(logFileName);
    PID *controller = new PID(kp, ki, kd);

    std::vector<double> ctrlLog;

    // ROI begins
    zsim_roi_begin();

    for (auto v : lVals) {
        ctrlLog.push_back(controller->calculate(v.first, v.second));
    }

    zsim_roi_end();
    // ROI ends

    // Write the output trajectory
    std::ofstream outLog;
    outLog.open(outputFile);
    for (double c : ctrlLog) {
        outLog << c << std::endl;
        ;
    }
    outLog.close();

    return 0;
}
