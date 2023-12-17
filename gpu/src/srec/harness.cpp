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
#include "frame.h"
#include "log.h"
#include "map.h"
#include "nlohmann/json.hpp"
#include "reconstruct.h"
#include "rowild_utils.h"
#include <fstream>
#include <string>
#include <vector>

using json = nlohmann::json;

double **readCameraIntrinsics(std::string inputFile, int downsample) {
    double **intrinsicMatrix = new double *[3];
    for (int i = 0; i < 3; i++) {
        intrinsicMatrix[i] = new double[3];
    }

    std::ifstream cameraFile;
    cameraFile.open(inputFile);
    assert(cameraFile.good());

    json j;
    cameraFile >> j;
    cameraFile.close();

    json jArray = json::array();
    assert(j.contains("intrinsic_matrix"));
    jArray = j["intrinsic_matrix"];

    int r = 0, c = 0;
    for (auto &element : jArray) {
        intrinsicMatrix[r][c] = static_cast<double>(element) / downsample;
        if (++r == 3) {
            r = 0;
            c++;
        }
    }

    intrinsicMatrix[2][2] = 1.0;
    return intrinsicMatrix;
}

double **readInitTransFile(std::string inputFile) {
    double **tCameraToWorld = new double *[4];
    for (int i = 0; i < 4; i++) {
        tCameraToWorld[i] = new double[4];
    }

    std::ifstream transFile;
    transFile.open(inputFile);
    assert(transFile.good());

    double token;
    int r = 0, c = 0;
    while (transFile >> token) {
        tCameraToWorld[r][c] = token;
        if (++c == 4) {
            c = 0;
            r++;
        }
    }
    transFile.close();

    return tCameraToWorld;
}

int main(int argc, const char **argv) {
    using args::KVArg;
    using args::Parser;

    Parser parser(argv[0], argc, argv, false);
    KVArg<std::string> inputPathArg(parser, "path", "",
                                    "Path to the dataset directory");
    KVArg<std::string> intrinsicsFileArg(parser, "camera", "",
                                         "Camera intrinsics file");
    KVArg<int> startIdxArg(parser, "start_idx", "",
                           "Index to the first frame in the dataset");
    KVArg<int> endIdxArg(parser, "end_idx", "",
                         "Index to the last frame in the dataset");
    KVArg<int> downsampleArg(parser, "downsample", "", "Downsample factor");
    KVArg<std::string> outputArg(parser, "output", "", "Output log file");

    if (!parser.parse()) assert(false);

    assert_msg(inputPathArg.found(), "The path to input scene is not provided");
    assert_msg(intrinsicsFileArg.found(),
               "The camera intrinsics file is not provided");

    std::string inputPath = inputPathArg.value();
    std::string intrinsicsFile = intrinsicsFileArg.value();
    int startIdx = startIdxArg.found() ? startIdxArg.value() : 1;
    int endIdx = endIdxArg.found() ? endIdxArg.value() : 10;
    int downsample = downsampleArg.found() ? downsampleArg.value() : 1;
    std::string outputFile =
        outputArg.found() ? outputArg.value() : "/dev/null";

    double **intrinsicMatrix = readCameraIntrinsics(intrinsicsFile, downsample);
    double **tCameraToWorld =
        readInitTransFile(inputPath + "/init_transform.txt");

    double **tWorldToCamera = new double *[4];
    for (int i = 0; i < 4; i++)
        tWorldToCamera[i] = new double[4];
    invert4x4Matrix<double>(tCameraToWorld, tWorldToCamera);

    std::vector<std::string> outputTrajectory;

    double depthScale = 256.0 / 5000.0;
    double colorScale = 1.0;

    Map *m = new Map();

    for (int i = startIdx; i <= endIdx; i++) {
        Frame *f = new Frame(inputPath, i, depthScale, colorScale, downsample);
        f->calcVertexMap(intrinsicMatrix);

        // ROI begins

        if (i > 1) {
            tWorldToCamera =
                reconstruct::icp(m, f, intrinsicMatrix, tWorldToCamera);
            invert4x4Matrix<double>(tWorldToCamera, tCameraToWorld);
        }

        m->fuse(f, intrinsicMatrix, tCameraToWorld);

        // ROI ends

        outputTrajectory.push_back(matrixToString(tCameraToWorld, 4, 4));
        delete f;

        info("Frame %d is processed", i);
    }

    std::ofstream outLogFile;
    outLogFile.open(outputFile);
    for (auto t : outputTrajectory) {
        outLogFile << t << std::endl;
    }
    outLogFile.close();

    for (int i = 0; i < 3; i++)
        delete[] intrinsicMatrix[i];
    delete[] intrinsicMatrix;
    for (int i = 0; i < 4; i++)
        delete[] tCameraToWorld[i];
    delete[] tCameraToWorld;
    delete m;

    return 0;
}
