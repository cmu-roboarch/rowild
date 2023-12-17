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
#include "ogm.h"
#include "zsim_hooks.h"
#include <fstream>
#include <vector>

std::vector<std::pair<double, double>> readInputFile(const char *fileName) {
    std::vector<std::pair<double, double>> measurements;

    std::ifstream mFile;
    mFile.open(fileName);
    assert(mFile.good());

    std::string line;
    while (std::getline(mFile, line)) {
        double ang, val;
        std::stringstream ss(line);
        ss >> ang >> val;
        measurements.push_back(std::make_pair(ang, val));
    }

    mFile.close();

    return measurements;
}

int main(int argc, const char **argv) {
    using args::KVArg;
    using args::Parser;

    Parser parser(argv[0], argc, argv, false);
    KVArg<std::string> inputArg(parser, "input", "", "Input measurements");
    KVArg<int> gridSizeArg(parser, "size", "", "Grid size");
    KVArg<double> initialProbArg(parser, "initprob", "",
                                 "The initial probability");
    KVArg<double> occProbArg(parser, "occprob", "", "The occupied probability");
    KVArg<double> freeProbArg(parser, "freeprob", "", "The free probability");
    KVArg<std::string> outputGridArg(parser, "output", "", "Output grid file");

    if (!parser.parse()) assert(false);

    assert_msg(inputArg.found(), "Input measurement file is not provided");

    const char *inputFile = inputArg.value().c_str();
    int gridSize = gridSizeArg.found() ? gridSizeArg.value() : 100;
    double initProb = initialProbArg.found() ? initialProbArg.value() : 0.5;
    double occProb = occProbArg.found() ? occProbArg.value() : 0.7;
    double freeProb = freeProbArg.found() ? freeProbArg.value() : 0.3;
    const char *outputFile =
        outputGridArg.found() ? outputGridArg.value().c_str() : "/dev/null";

    auto measurements = readInputFile(inputFile);

    // Simulate robot at center of grid
    double robotX = gridSize / 2.0;
    double robotY = gridSize / 2.0;

    OccupancyGrid *ogm =
        new OccupancyGrid(gridSize, initProb, occProb, freeProb);

    // ROI begin
    zsim_roi_begin();

    for (auto m : measurements) {
        ogm->update(robotX, robotY, m.first, m.second);
    }

    zsim_roi_end();
    // ROI end

    // Output the final grid
    ogm->display(outputFile);

    return 0;
}
