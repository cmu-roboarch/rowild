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
#include <fstream>
#include <string>
#include <vector>

class Arm {
  public:
    explicit Arm(const char *fileName) {
        std::ifstream file(fileName);
        assert(file.good());
        std::string token;

        file >> token;
        assert(token == "DOF");
        file >> token;
        assert(token == "=");
        file >> token;
        this->dof = stoi(token);

        file >> token;
        assert(token == "LINK_LENGTH");
        file >> token;
        assert(token == "=");
        file >> token;
        this->linkLength = stoi(token);

        file >> token;
        assert(token == "NUM_TESTS");
        file >> token;
        assert(token == "=");
        file >> token;
        int numTests = stoi(token);

        while (true) {
            getline(file, token);
            if (token == "START:") break;
        }

        for (int i = 0; i < numTests; i++) {
            getline(file, token);
            std::stringstream row(token);
            assert(row.good());
            double value;
            double *cfg = new double[this->dof];
            int j = 0;
            while (row >> value) {
                cfg[j] = value;
                j++;
            }
            this->startCfgs.push_back(cfg);
            assert(j == this->dof);
            assert(!row.good());
        }

        while (true) {
            getline(file, token);
            if (token == "GOAL:") break;
        }

        for (int i = 0; i < numTests; i++) {
            getline(file, token);
            std::stringstream row(token);
            assert(row.good());
            double value;
            double *cfg = new double[this->dof];
            int j = 0;
            while (row >> value) {
                cfg[j] = value;
                j++;
            }
            this->goalCfgs.push_back(cfg);
            assert(j == this->dof);
            assert(!row.good());
        }
        file.close();
    }

    int getDof() const { return dof; }
    int getLinkLength() const { return linkLength; }
    std::vector<double *> getStartCfgs() const { return startCfgs; }
    std::vector<double *> getGoalCfgs() const { return goalCfgs; }

  private:
    int dof;        // Arm's degree of freedom
    int linkLength; // The length of every of the links
    std::vector<double *> startCfgs, goalCfgs;
};
