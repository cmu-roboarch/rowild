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

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

#include "args.h"
#include "log.h"
#include "orb.h"
#include "zsim_hooks.h"

using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::nanoseconds;

int main(int argc, const char **argv) {
    using args::KVArg;
    using args::Parser;

    Parser parser(argv[0], argc, argv, false);
    KVArg<std::string> inputDirArg(
        parser, "dir", "", "Input directory containing image sequences");

    if (!parser.parse()) assert(false);

    assert_msg(inputDirArg.found(), "Input image directory");

    std::string inputDir = inputDirArg.value();

    std::vector<std::string> imagePaths;

    for (const auto &entry : std::filesystem::directory_iterator(inputDir)) {
        imagePaths.push_back(entry.path().string());
    }

    std::sort(imagePaths.begin(), imagePaths.end());

    // Assuming a known camera matrix K
    cv::Mat K = (cv::Mat_<double>(3, 3) << 525.0, 0.0, 319.5, 0.0, 525.0, 239.5,
                 0.0, 0.0, 1.0);

    SimpleORBSLAM slam(K);

    double totalTime = 0;
    for (const std::string &path : imagePaths) {
        cv::Mat image = cv::imread(path, cv::IMREAD_UNCHANGED);

        if (image.empty()) {
            std::cerr << "Failed to read image at: " << path << std::endl;
            continue;
        }

        auto t0 = high_resolution_clock::now();

        // ROI begins
        zsim_roi_begin();

        slam.processImage(image);

        zsim_roi_end();
        // ROI ends

        auto t1 = high_resolution_clock::now();

        totalTime += duration_cast<nanoseconds>(t1 - t0).count() * 1e-9;

        // cv::imshow("Image Sequence", image); if (cv::waitKey(1) == 27) {
        // break; }
    }

    cv::destroyAllWindows();

    std::cout << "Inference time: " << totalTime << std::endl;

    return 0;
}
