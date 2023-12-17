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
#include "pp.h"
#include "zsim_hooks.h"
#include <fstream>
#include <math.h>
#include <string>
#include <vector>

std::vector<Point> readInputFile(const char *fileName) {
    std::vector<Point> path;

    std::ifstream pathFile;
    pathFile.open(fileName);
    assert(pathFile.good());

    std::string line;
    while (std::getline(pathFile, line)) {
        double x, y;
        std::stringstream ss(line);
        ss >> x >> y;
        path.push_back(Point(x, y));
    }

    pathFile.close();

    return path;
}

int main(int argc, const char **argv) {
    using args::KVArg;
    using args::Parser;

    Parser parser(argv[0], argc, argv, false);
    KVArg<std::string> pathFileArg(parser, "path", "", "Input path file");
    KVArg<double> distArg(parser, "dist", "", "Lookahead distance");
    KVArg<std::string> outArg(parser, "output", "", "Output log file");

    if (!parser.parse()) assert(false);

    assert_msg(pathFileArg.found(), "Input file is not provided");

    const char *pathFile = pathFileArg.value().c_str();
    double dist = distArg.found() ? distArg.value() : 2.0;
    std::string outputFile = outArg.found() ? outArg.value() : "/dev/null";

    auto path = readInputFile(pathFile);
    Point robotPosition = {1.0, 0.5};

    PurePursuit *tracker = new PurePursuit(path, dist);

    // ROI begins
    zsim_roi_begin();

    Point lookAhead = tracker->getLookAheadPoint(robotPosition);

    zsim_roi_end();
    // ROI ends

    // Write the output trajectory
    std::ofstream outLog;
    outLog.open(outputFile);
    outLog << lookAhead.x << " " << lookAhead.y << std::endl;
    outLog.close();

    return 0;
}
