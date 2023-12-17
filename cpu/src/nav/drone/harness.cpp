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
#include "drone.h"
#include "log.h"
#include "zsim_hooks.h"
#include <fstream>
#include <string>

int main(int argc, const char **argv) {
    using args::FlagArg;
    using args::KVArg;
    using args::Parser;

    Parser parser(argv[0], argc, argv, false);
    KVArg<std::string> inputMapArg(parser, "map", "", "Input map file");
    KVArg<double> heuristicWeightArg(parser, "weight", "",
                                     "Heuristic weight of A*");
    KVArg<std::string> aStarHeuristicArg(parser, "heuristic", "",
                                         "A* heuristic");
    FlagArg deepeningArg(parser, "deepening", "", "Enable IDA*");
    KVArg<uint64_t> maxIterationsArg(parser, "max-iters", "",
                                     "Maximum iterations");
    KVArg<std::string> outputPathArg(parser, "output", "", "Output path file");

    if (!parser.parse()) assert(false);

    assert_msg(inputMapArg.found(), "Input map file is not provided");

    std::string inputFile = inputMapArg.value();
    double heuristicWeight =
        heuristicWeightArg.found() ? heuristicWeightArg.value() : 1.0;
    std::string aStarHeuristicStr =
        aStarHeuristicArg.found() ? aStarHeuristicArg.value() : "Euclidean";
    bool idastar = deepeningArg.found();
    uint64_t maxIterations =
        maxIterationsArg.found() ? maxIterationsArg.value() : 100'000'000;
    std::string outputFile =
        outputPathArg.found() ? outputPathArg.value() : "/dev/null";

    STATE startPos = {30 /*X*/, 1 /*Y*/, 1 /*Z*/};
    STATE goalPos = {40 /*X*/, 5 /*Y*/, 6 /*Z*/};

    Drone drone(inputFile, startPos, goalPos, aStarHeuristicStr,
                heuristicWeight, idastar);

    // ROI begins
    zsim_roi_begin();

    auto path = drone.run(maxIterations);

    zsim_roi_end();
    // ROI ends

    // Write the output path
    std::ofstream pathFile;
    pathFile.open(outputFile);
    for (auto dir : path) {
        pathFile << dir << std::endl;
    }
    pathFile.close();

    return 0;
}
