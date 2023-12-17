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
#include "log.h"
#include "objdet.h"
#include "pp.h"
#include "slam.h"
#include <filesystem>
#include <iostream>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

std::vector<Point> readPath(const char *fileName) {
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

std::vector<std::vector<double>> readSensorLog(std::string inputLogFile) {
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
    KVArg<std::string> inputLabelsArg(parser, "labels", "",
                                      "Input file containing class names");
    KVArg<std::string> inputModelArg(parser, "model", "",
                                     "Pre-trained model containing weights");
    KVArg<std::string> inputCfgArg(parser, "cfg", "",
                                   "Model configuration file");
    KVArg<std::string> inputImgDirArg(parser, "imgdir", "", "Input image file");
    KVArg<float> imgScaleArg(parser, "scale", "", "Image scale factor");
    KVArg<float> confThreshArg(
        parser, "confidence", "",
        "The minimum confidence threshold for drawing bounding boxes");
    KVArg<std::string> pathFileArg(parser, "path", "", "Input path file");
    KVArg<std::string> sensorLogArg(parser, "log", "", "Input sensor log file");
    KVArg<std::string> outputArg(parser, "output", "", "Output log file");

    if (!parser.parse()) assert(false);

    assert_msg(inputLabelsArg.found(), "Input class file is not provided");
    assert_msg(inputModelArg.found(), "Input model file is not provided");
    assert_msg(inputCfgArg.found(), "Input configuration file is not provided");
    assert_msg(inputImgDirArg.found(), "Input image directory is not provided");
    assert_msg(pathFileArg.found(), "Input path file is not provided");
    assert_msg(sensorLogArg.found(), "Input sensor log file is not provided");

    // Mobile-Net SSD
    std::string inputLabelsFile = inputLabelsArg.value();
    std::string inputModelFile = inputModelArg.value();
    std::string inputCfgFile = inputCfgArg.value();
    std::string inputImgDir = inputImgDirArg.value();
    float imgScale = imgScaleArg.found() ? imgScaleArg.value() : 1.0;
    float confThresh = confThreshArg.found() ? confThreshArg.value() : 0.4;
    ObjDetect *objDet = new ObjDetect(inputLabelsFile, inputModelFile,
                                      inputCfgFile, confThresh);

    // SLAM
    std::string inputSensorLogFile = sensorLogArg.value();
    double sigX2 = 0.25 * 0.25;
    double sigY2 = 0.1 * 0.1;
    double sigAlpha2 = 0.1 * 0.1;  // Rotation
    double sigBeta2 = 0.01 * 0.01; // Bearing angle
    double sigR2 = 0.08 * 0.08;    // Range

    std::vector<std::vector<double>> inputSensorLog =
        readSensorLog(inputSensorLogFile);
    std::vector<double> initialMeasurement = inputSensorLog.front();
    int numLandmarks = static_cast<int>(initialMeasurement.size() / 2);
    SLAM *ekfslam =
        new EKFSLAM(numLandmarks, sigX2, sigY2, sigAlpha2, sigBeta2, sigR2);
    size_t sensorLogIdx = 1;

    // Pure Pursuit
    const char *pathFile = pathFileArg.value().c_str();
    auto path = readPath(pathFile);
    PurePursuit *tracker = new PurePursuit(path, 2.0);

    // Logging
    std::string outputFile =
        outputArg.found() ? outputArg.value() : "/dev/null";
    std::vector<std::vector<double>> outputLog;

    // ROI begins
    try {
        for (const auto &entry : fs::directory_iterator(inputImgDir)) {
            if (entry.is_regular_file()) {
                auto filePath = entry.path();
                if (filePath.extension() == ".jpg" ||
                    filePath.extension() == ".png") {

                    std::thread evaluationThread(&ObjDetect ::evaluate, objDet,
                                                 filePath.string(), imgScale);

                    if (sensorLogIdx < inputSensorLog.size()) {
                        std::vector l = inputSensorLog[sensorLogIdx];
                        if (l.size() == 2) {
                            // Control input
                            ekfslam->motionUpdate(l[0], l[1], l[2]);
                        } else if (l.size() ==
                                   2 * static_cast<size_t>(numLandmarks)) {
                            // Measurement input
                            ekfslam->measurementUpdate(l);
                        } else {
                            panic("Illegal input");
                        }

                        std::vector state = ekfslam->getStatus();
                        Point lookahead = tracker->getLookAheadPoint(
                            Point(state[0], state[1]));
                        state.push_back(lookahead.x);
                        state.push_back(lookahead.x);

                        outputLog.push_back(state);
                        sensorLogIdx++;
                    }

                    if (evaluationThread.joinable()) {
                        evaluationThread.join();
                    }
                }
            }
        }
    } catch (const fs::filesystem_error &e) {
        std::cerr << "Filesystem error: " << e.what() << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "General error: " << e.what() << std::endl;
    }
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

    delete objDet;
    delete ekfslam;

    return 0;
}
