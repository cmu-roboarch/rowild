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
#include "utils.h"
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

struct PointsObs {
    double point[9][3];

    explicit PointsObs(double (&p)[9][3]) {
        std::copy(&p[0][0], &p[0][0] + 9 * 3, &point[0][0]);
    }
};

struct AxesObs {
    double axes[3][3];

    explicit AxesObs(double (&a)[3][3]) {
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
            double token;
            std::vector<double> nums;
            while (ss >> token) {
                nums.push_back(token);
            }
            if (nums.size() != 9) {
                std::cout << line << std::endl;
                printContainer(nums);
            }
            assert(nums.size() == 9);

            double H[4][4];
            rpyxyzToH(nums[0], nums[1], nums[2], nums[3], nums[4], nums[5], H);

            double corners[9][3];
            double dim[3] = {nums[6], nums[7], nums[8]};
            blockDescToBoundingBox(H, dim, corners);

            double axes[3][3];
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

    void showEnv() const {
        assert(this->pointsObs.size() == this->axesObs.size());
        std::cout << "Num obstacles: " << this->pointsObs.size() << std::endl;

        std::cout << "Obstacle points:" << std::endl;
        for (auto p : this->pointsObs) {
            printMatrix<double, 9, 3>(p.point);
            std::cout << std::string(10, '-') << std::endl;
        }

        std::cout << "Obstacle axes:" << std::endl;
        for (auto a : this->axesObs) {
            printMatrix<double, 3, 3>(a.axes);
            std::cout << std::string(10, '-') << std::endl;
        }

        std::cout << std::string(20, '+') << std::endl;
    }

  private:
    std::vector<PointsObs> pointsObs;
    std::vector<AxesObs> axesObs;
};
