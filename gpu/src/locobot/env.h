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

#pragma once

#include "log.h"
#include "rowild_utils.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

struct PointsObs {
    float point[9][3];

    explicit PointsObs(float (&p)[9][3]) {
        std::copy(&p[0][0], &p[0][0] + 9 * 3, &point[0][0]);
    }
};

struct AxesObs {
    float axes[3][3];

    explicit AxesObs(float (&a)[3][3]) {
        std::copy(&a[0][0], &a[0][0] + 3 * 3, &axes[0][0]);
    }
};

class Environment {
  public:
    explicit Environment(const char *fileName) {
        std::ifstream file(fileName);
        assert(file.good());
        std::string line;

        while (std::getline(file, line)) {
            while (line.size() && line.front() == ' ')
                line.erase(0, 1);
            while (line.size() && line.back() == ' ')
                line.pop_back();
            if (line.size() == 0) continue;
            if (line[0] == '#') continue;

            line.erase(std::remove(line.begin(), line.end(), ','), line.end());

            std::stringstream ss(line);
            float token;
            std::vector<float> nums;
            while (ss >> token) {
                nums.push_back(token);
            }
            assert(nums.size() == 9);

            float H[4][4];
            rpyxyzToH<float>(nums[0], nums[1], nums[2], nums[3], nums[4],
                             nums[5], H);

            float corners[9][3];
            float dim[3] = {nums[6], nums[7], nums[8]};
            blockDescToBoundingBox<float>(H, dim, corners);

            float axes[3][3];
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    axes[i][j] = H[j][i];
                }
            }

            this->pointsObs.push_back(PointsObs(corners));
            this->axesObs.push_back(AxesObs(axes));
        }

        file.close();
    }

    std::vector<PointsObs> getPointsObs() const { return this->pointsObs; }

    std::vector<AxesObs> getAxesObs() const { return this->axesObs; }

  private:
    std::vector<PointsObs> pointsObs;
    std::vector<AxesObs> axesObs;
};
