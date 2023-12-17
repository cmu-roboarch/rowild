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
#include <string>
#include <utility>
#include <vector>

class Environment {
  public:
    explicit Environment(const char *fileName) {
        /*
         * The map files posit 1-based indexing, but the code posits
         * standard 0-based indexing. That's why the data structures are
         * filled out with values that are not necessarily the same as those
         * in the map files.
         */

        auto getPair = [](std::string str) {
            std::size_t pos = str.find(',');
            assert(pos != std::string::npos);
            int first = stoi(str.substr(0, pos));
            int second = stoi(str.substr(pos + 1));
            return std::make_pair(first, second);
        };

        std::ifstream file(fileName);
        assert(file.good());

        std::string line;
        std::pair<int, int> pair;

        getline(file, line);
        assert(line == "N");

        getline(file, line);
        pair = getPair(line);
        mapX = pair.first;
        mapY = pair.second;

        getline(file, line);
        assert(line == "C");

        getline(file, line);
        collisionThreshold = stoi(line);

        getline(file, line);
        assert(line == "R");

        getline(file, line);
        pair = getPair(line);
        robotX = pair.first - 1; // 0-based indexing
        assert(robotX >= 0 && robotX < mapX);
        robotY = pair.second - 1; // 0-based indexing
        assert(robotY >= 0 && robotY < mapY);

        getline(file, line);
        assert(line == "T");

        while (true) {
            getline(file, line);
            if (line == "M") break;
            pair = getPair(line);
            targetTrajectory.push_back(std::make_pair(
                pair.first - 1, pair.second - 1)); // 0-based indexing
        }

        map = new int *[mapX];
        for (int i = 0; i < mapX; i++) {
            map[i] = new int[mapY];
        }

        for (int i = 0; i < mapX; i++) {
            getline(file, line);
            std::stringstream row(line);
            for (int j = 0; j < mapY; j++) {
                assert(row.good());
                std::string substr;
                getline(row, substr, ',');
                map[i][j] = stoi(substr);
                assert(map[i][j] > 0);
            }
            assert(!row.good());
        }
        file.close();
    }

    ~Environment() {
        for (int i = 0; i < mapX; i++) {
            delete[] map[i];
        }
        delete[] map;
    }

    int getMapX() const { return mapX; }

    int getMapY() const { return mapY; }

    int getRobotX() const { return robotX; }

    int getRobotY() const { return robotY; }

    int getCellCost(int x, int y) const { return map[x][y]; }

    int getTargetSteps() const { return targetTrajectory.size(); }

    int getTargetX(int time) const {
        assert(time < static_cast<int>(targetTrajectory.size()));
        return targetTrajectory[time].first;
    }

    int getTargetY(int time) const {
        assert(time < static_cast<int>(targetTrajectory.size()));
        return targetTrajectory[time].second;
    }

    bool isFree(int x, int y) const { return (map[x][y] < collisionThreshold); }

    bool isValid(int x, int y) const {
        return ((x >= 0) && (x < mapX) && (y >= 0) && (y < mapY));
    }

  private:
    int mapX, mapY;
    int **map;
    int collisionThreshold;
    int robotX, robotY;
    std::vector<std::pair<int, int>> targetTrajectory;
};
