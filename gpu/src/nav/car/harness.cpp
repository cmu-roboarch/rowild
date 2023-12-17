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
#include "sdcar.h"
#include <fstream>
#include <string>

int main(int argc, const char **argv) {
    using args::FlagArg;
    using args::KVArg;
    using args::Parser;

    Parser parser(argv[0], argc, argv, false);
    KVArg<std::string> inputMapArg(parser, "map", "", "Input map file");
    KVArg<double> heuristicWeightArg(parser, "weight", "", "Heuristic weight");
    KVArg<std::string> outputPathArg(parser, "output", "", "Output path file");

    if (!parser.parse()) assert(false);

    assert_msg(inputMapArg.found(), "Input map file is not provided");

    std::string inputFile = inputMapArg.value();
    double hWeight =
        heuristicWeightArg.found() ? heuristicWeightArg.value() : 1.0;
    std::string outputFile =
        outputPathArg.found() ? outputPathArg.value() : "/dev/null";

    int sx = 5, sy = 5, stheta = 0;
    int ex = 10, ey = 10, etheta = 0;
    bool randomPoints = true;

    CarRobot car(inputFile);
    std::vector<int> path =
        car.plan(sx, sy, stheta, ex, ey, etheta, randomPoints, hWeight);

    // Write the output path
    std::ofstream pathFile;
    pathFile.open(outputFile);
    int cnt = 0;
    for (auto v : path) {
        if (cnt) pathFile << " -> ";
        if (++cnt % 6 == 0) pathFile << "\n\t";
        int x, y, thetaIdx;
        car.toXYTheta(v, &x, &y, &thetaIdx);
        pathFile << "(" << x << " " << y << " " << thetaIdx << ")";
    }
    pathFile << "\n";
    pathFile.close();

    return 0;
}
