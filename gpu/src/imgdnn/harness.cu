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

#include <fstream>
#include <iostream>
#include <opencv2/core/cuda.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "args.h"
#include "log.h"
#include "rowild_utils.h"
#include "timer.h"

std::vector<std::string> readInputLabels(std::string file) {
    std::vector<std::string> cn;
    std::ifstream inFile(file);
    std::string line;

    while (std::getline(inFile, line)) {
        cn.push_back(line);
    }

    return cn;
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
    KVArg<std::string> inputImgArg(parser, "img", "", "Input image file");
    KVArg<float> imgScaleArg(parser, "scale", "", "Image scale factor");
    KVArg<float> confThreshArg(
        parser, "confidence", "",
        "The minimum confidence threshold for drawing bounding boxes");
    KVArg<std::string> outputImgArg(parser, "output", "", "Output image file");

    if (!parser.parse()) assert(false);

    assert_msg(inputLabelsArg.found(), "Input class file is not provided");
    assert_msg(inputModelArg.found(), "Input model file is not provided");
    assert_msg(inputCfgArg.found(), "Input configuration file is not provided");
    assert_msg(inputImgArg.found(), "Input image file");

    std::string inputLabelsFile = inputLabelsArg.value();
    std::string inputModelFile = inputModelArg.value();
    std::string inputCfgFile = inputCfgArg.value();
    std::string inputImgFile = inputImgArg.value();
    float imgScale = imgScaleArg.found() ? imgScaleArg.value() : 1.0;
    float confThresh = confThreshArg.found() ? confThreshArg.value() : 0.4;
    std::string outputImg = outputImgArg.found() ? outputImgArg.value() : "";

    assert(imgScale > 0 && imgScale <= 1.0);
    assert(confThresh >= 0 && confThresh <= 1.0);

    auto classLabels = readInputLabels(inputLabelsFile);
    auto model = cv::dnn::readNet(inputModelFile, inputCfgFile, "TensorFlow");
    cv::Mat image = cv::imread(inputImgFile);
    cv::Mat blob =
        cv::dnn::blobFromImage(image, imgScale, cv::Size(300, 300),
                               cv::Scalar(127.5, 127.5, 127.5), true, false);

    model.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    model.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

    model.setInput(blob);

    timer t;

    // ROI begins
    cv::Mat output = model.forward();
    cv::Mat detectionMat(output.size[2], output.size[3], CV_32F,
                         output.ptr<float>());
    // ROI ends

    std::cout << "Inference time: " << t.elapsed() << std::endl;

    if (!outputImg.empty()) {
        for (int i = 0; i < detectionMat.rows; i++) {
            int classId = detectionMat.at<float>(i, 1);

            if (detectionMat.at<float>(i, 2) < confThresh) continue;
            int box_x =
                static_cast<int>(detectionMat.at<float>(i, 3) * image.cols);
            int box_y =
                static_cast<int>(detectionMat.at<float>(i, 4) * image.rows);
            int box_width = static_cast<int>(
                detectionMat.at<float>(i, 5) * image.cols - box_x);
            int box_height = static_cast<int>(
                detectionMat.at<float>(i, 6) * image.rows - box_y);
            rectangle(image, cv::Point(box_x, box_y),
                      cv::Point(box_x + box_width, box_y + box_height),
                      cv::Scalar(255, 255, 255), 2);
            putText(image, classLabels[classId - 1].c_str(),
                    cv::Point(box_x, box_y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 255, 255), 1);
        }

        imwrite(outputImg, image);
    }

    return 0;
}
