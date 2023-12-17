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

#include "objdet.h"

ObjDetect::ObjDetect(std::string labelsFile, std::string modelFile,
                     std::string cfgFile, float conf) {
    std::ifstream inFile(labelsFile);
    std::string line;
    while (std::getline(inFile, line)) {
        this->classLabels.push_back(line);
    }

    this->model = cv::dnn::readNet(modelFile, cfgFile, "TensorFlow");

    assert(conf >= 0 && conf <= 1);
    this->confThresh = conf;
}

void ObjDetect::evaluate(std::string imageFile, float scale) {
    assert(scale > 0 && scale <= 1);

    cv::Mat image = cv::imread(imageFile);
    cv::Mat blob =
        cv::dnn::blobFromImage(image, scale, cv::Size(300, 300),
                               cv::Scalar(127.5, 127.5, 127.5), true, false);

    this->model.setInput(blob);
    cv::Mat output = this->model.forward();
    cv::Mat detectionMat(output.size[2], output.size[3], CV_32F,
                         output.ptr<float>());

    for (int i = 0; i < detectionMat.rows; i++) {
        if (detectionMat.at<float>(i, 2) >= this->confThresh) {
            int classId = detectionMat.at<float>(i, 1);
            if (classId == 49 /*Knife*/) { // Extend this based on possible
                                           // suspicious objects on your campus!
                std::cout << "Alert! Detected: "
                          << this->classLabels[classId - 1] << " on campus"
                          << std::endl;
            }
        }
    }
}
