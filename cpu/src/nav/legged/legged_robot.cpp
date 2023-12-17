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

#include "legged_robot.h"
#include "log.h"
#include <algorithm>
#include <fstream>
#include <string>
#include <vector>

LeggedRobot::LeggedRobot(std::string mapFile, int scaleMap, int gridDegree,
                         STATE initial, STATE goal, int robotL, int robotW,
                         std::string heuristic, double hWeight, bool deepening)
    : AStar(initial, goal, hWeight, deepening) {
    this->theGoal = goal;
    this->robotLength = robotL;
    this->robotWidth = robotW;

    this->numNeighbors = gridDegree;

    if (gridDegree == 4) {
        dX = {-1, 0, 0, 1};
        dY = {0, -1, 1, 0};
    } else if (gridDegree == 8) {
        dX = {-1, -1, -1, 0, 0, 1, 1, 1};
        dY = {-1, 0, 1, -1, 1, -1, 0, 1};
    } else {
        assert_msg(false, "Invalid gridDegree: %d", gridDegree);
    }

    for (int i = 0; i < static_cast<int>(dX.size()); i++) {
        int xMove = dX[i];
        int yMove = dY[i];
        this->movementCost.push_back(
            std::sqrt(std::abs(xMove) + std::abs(yMove)));
    }

    this->readMap(mapFile, scaleMap);

    assert_msg(this->isValid(initial),
               "The initial state cannot be invalid/collision");
    assert_msg(this->isValid(goal),
               "The goal state cannot be invalid/collision");

    std::transform(heuristic.begin(), heuristic.end(), heuristic.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    if (heuristic.compare("none") == 0) {
        this->heuristicFunc = [](STATE, STATE) { return 0; };
    } else if (heuristic.compare("euclidean") == 0) {
        this->heuristicFunc = getEuclideanDistance<STATE, 2>;
    } else if (heuristic.compare("manhattan") == 0) {
        this->heuristicFunc = getManhattanDistance<STATE, 2>;
    } else {
        assert_msg(false, "Invalid heuristic: %s", heuristic.c_str());
    }
}

LeggedRobot::~LeggedRobot() {
    for (int i = 0; i < this->mapY; i++) {
        delete[] this->env[i];
    }
    delete[] this->env;
}

std::vector<STATE> LeggedRobot::getNeighbors(const STATE &s) const {
    std::vector<STATE> neighbors;
    for (int i = 0; i < this->numNeighbors; i++) {
        neighbors.push_back({s[0] + this->dX[i], s[1] + this->dY[i]});
    }

    return neighbors;
}

double LeggedRobot::getG(const STATE &prevS, const STATE &currS, double prevG,
                         int dir) const {
    assert(this->isValid(prevS));
    assert(this->isValid(currS));

    assert(dir >= 0 && dir < this->numNeighbors);
    return prevG + this->movementCost[dir];
}

double LeggedRobot::getH(const STATE &s) const {
    assert(this->isValid(s));
    return this->heuristicFunc(s, this->theGoal);
}

bool LeggedRobot::isObstacle(const STATE &s) const {
    for (int i = 0; i <= this->robotWidth; i++) {
        for (int j = 0; j <= this->robotLength; j++) {
            int x = s[0] + j;
            int y = s[1] + i;

            if (this->env[y][x]) {
                return true;
            }
        }
    }

    return false;
}

bool LeggedRobot::isValid(const STATE &s) const {
    for (int i = 0; i <= this->robotWidth; i++) {
        for (int j = 0; j <= this->robotLength; j++) {
            int x = s[0] + j;
            int y = s[1] + i;

            if ((x < 0) || (x >= this->mapX) || (y < 0) || (y >= this->mapY)) {
                return false; // Sticks out of the map
            }
        }
    }

    return !this->isObstacle(s);
}

void LeggedRobot::readMap(std::string fileName, int scale) {
    // NOTE: This function assumes Moving AI format for the input map. If
    // you are using another benchmark, change it accordingly.
    std::ifstream file(fileName);
    assert(file.good());

    std::string line;
    std::size_t pos;

    auto removeCR = [](std::string str) {
        // NOTE: The benchmarks files released by Moving AI have not been
        // produced observing Unix<->Windows file exchange norms. As such, a
        // new line in the files is represented by CR+LF instead of just LF.
        if (static_cast<int>(str.back()) == 13 /*CR ASCII*/) {
            str.pop_back();
        }
        return str;
    };

    std::getline(file, line);
    line = removeCR(line);
    assert(line == "type octile");

    std::getline(file, line);
    line = removeCR(line);
    pos = line.find(' ');
    assert(pos != std::string::npos);
    assert(line.substr(0, pos) == "height");
    this->mapY = stoi(line.substr(pos + 1));

    std::getline(file, line);
    line = removeCR(line);
    pos = line.find(' ');
    assert(pos != std::string::npos);
    assert(line.substr(0, pos) == "width");
    this->mapX = stoi(line.substr(pos + 1));

    std::getline(file, line);
    line = removeCR(line);
    assert(line == "map");

    /*
     * Moving AI map format:
     *  @ or T: Obstacle
     *  .: Free
     *
     *              Width(X)
     *            ◄─────────►
     *           ▲ ....@@@@@@
     *           │ .....@@@@@
     * Height(Y) │ ......@@@@
     *           │ .......@@@
     *           ▼ ........@@
     *
     * >> Width is X and Height is Y in data structures
     * >> Top-left corner is (X=0, Y=0)
     *
     * The data structure of the environment (env) resembles its actual shape
     *
     *
     * The modeled robot:
     *
     *           Length(X)
     *          ┌─────────┐
     * Width(Y) │         │
     *          └─────────┘
     * >> Top-left corner is (X=0, Y=0)
     *
     * The state of the robot is represented by STATE=std::vector<int> whose
     * size is always 2. state[0] is x and state[1] is y of the robot.
     */

    env = new bool *[scale * this->mapY];
    for (int i = 0; i < scale * this->mapY; i++) {
        env[i] = new bool[scale * this->mapX];
    }

    for (int y = 0; y < this->mapY; y++) {
        std::getline(file, line);
        line = removeCR(line);
        assert(static_cast<int>(line.length()) == this->mapX);

        for (int x = 0; x < this->mapX; x++) {
            char point = line[x];
            assert(point == '.' /*free*/ ||
                   (point == '@' || point == 'T') /*obstacle*/);
            bool obstacle = (point == '@' || point == 'T');

            int baseY = scale * y;
            int baseX = scale * x;
            for (int sy = 0; sy < scale; sy++) {
                for (int sx = 0; sx < scale; sx++) {
                    this->env[baseY + sy][baseX + sx] = obstacle;
                }
            }
        }
    }
    file.close();

    this->mapX *= scale;
    this->mapY *= scale;
}

void LeggedRobot::showMap() const {
    // For debug purposes
    for (int y = 0; y < this->mapY; y++) {
        for (int x = 0; x < this->mapX; x++) {
            bool obs = this->env[y][x];
            char c = obs ? '@' : '.';
            printf("%c", c);
        }
        printf("\n");
    }
}
