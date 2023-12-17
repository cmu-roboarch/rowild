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

#include "drone.h"
#include "log.h"
#include <algorithm>
#include <fstream>
#include <string>
#include <vector>

Drone::Drone(std::string mapFile, std::string heuristic, double hWeight,
             bool deepening)
    : AStar(hWeight, deepening) {

    this->dX = {-1, 1, 0, 0, 0, 0};
    this->dY = {0, 0, -1, 1, 0, 0};
    this->dZ = {0, 0, 0, 0, -1, 1};

    for (int i = 0; i < static_cast<int>(this->dX.size()); i++) {
        int xMove = this->dX[i];
        int yMove = this->dY[i];
        int zMove = this->dZ[i];
        double cost =
            std::sqrt(std::abs(xMove) + std::abs(yMove) + std::abs(zMove));
        assert(cost == 1.0);
        this->movementCost.push_back(cost);
    }

    this->readMap(mapFile);

    std::transform(heuristic.begin(), heuristic.end(), heuristic.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    if (heuristic.compare("none") == 0) {
        this->heuristicFunc = [](STATE, STATE) { return 0; };
    } else if (heuristic.compare("euclidean") == 0) {
        this->heuristicFunc = getEuclideanDistance<STATE, 3>;
    } else if (heuristic.compare("manhattan") == 0) {
        this->heuristicFunc = getManhattanDistance<STATE, 3>;
    } else {
        assert_msg(false, "Invalid heuristic: %s", heuristic.c_str());
    }
}

Drone::~Drone() {
    int xS = this->maxX - this->minX + 1;
    int yS = this->maxY - this->minY + 1;

    for (int i = 0; i < xS; i++) {
        for (int j = 0; j < yS; j++) {
            delete[] this->occGrid[i][j];
        }
        delete[] this->occGrid[i];
    }
    delete[] this->occGrid;
}

STATE Drone::applyMovement(const STATE &s, int dir) const {
    assert(dir < static_cast<int>(this->dX.size()));
    return {s[0] + this->dX[dir], s[1] + this->dY[dir], s[2] + this->dZ[dir]};
}

std::vector<STATE> Drone::getNeighbors(const STATE &s) const {
    int numNeighbors = static_cast<int>(this->dX.size());
    assert(static_cast<int>(this->dY.size()) == numNeighbors &&
           static_cast<int>(this->dZ.size()) == numNeighbors);

    std::vector<STATE> neighbors;
    for (int i = 0; i < numNeighbors; i++) {
        neighbors.push_back(
            {s[0] + this->dX[i], s[1] + this->dY[i], s[2] + this->dZ[i]});
    }

    return neighbors;
}

double Drone::getG(const STATE &prevS, const STATE &currS, double prevG,
                   int dir) const {
    assert(this->isValid(prevS));
    assert(this->isValid(currS));

    return prevG + this->movementCost[dir];
}

double Drone::getH(const STATE &s, const STATE &g) const {
    assert(this->isValid(s));
    return this->heuristicFunc(s, g);
}

bool Drone::isObstacle(const STATE &s) const {
    int x = s[0];
    int y = s[1];
    int z = s[2];

    assert(x - this->minX >= 0 && x - this->minX <= this->maxX - this->minX);
    assert(y - this->minY >= 0 && y - this->minY <= this->maxY - this->minY);
    assert(z - this->minZ >= 0 && z - this->minZ <= this->maxZ - this->minZ);

    return this->occGrid[x - this->minX][y - this->minY][z - this->minZ];
}

bool Drone::isValid(const STATE &s) const {
    int x = s[0], y = s[1], z = s[2];
    if (x <= this->minX || x >= this->maxX || y <= this->minY ||
        y >= this->maxY || z <= this->minZ || z >= this->maxZ) {
        return false;
    }

    return !this->isObstacle(s);
}

void Drone::readMap(std::string fileName) {
    std::ifstream file(fileName);
    assert(file.good());

    std::string token;
    int min, max;

    file >> token;
    assert(token == "X");
    file >> min >> max;
    this->minX = min;
    this->maxX = max;

    file >> token;
    assert(token == "Y");
    file >> min >> max;
    this->minY = min;
    this->maxY = max;

    file >> token;
    assert(token == "Z");
    file >> min >> max;
    this->minZ = min;
    this->maxZ = max;

    int xS = this->maxX - this->minX + 1;
    int yS = this->maxY - this->minY + 1;
    int zS = this->maxZ - this->minZ + 1;
    this->occGrid = new bool **[xS];
    for (int i = 0; i < xS; i++) {
        this->occGrid[i] = new bool *[yS];
        for (int j = 0; j < yS; j++) {
            this->occGrid[i][j] = new bool[zS]();
        }
    }

    int x, y, z;
    while (file >> x >> y >> z) {
        this->occGrid[x - this->minX][y - this->minY][z - this->minZ] = true;
    }
    file.close();
}
