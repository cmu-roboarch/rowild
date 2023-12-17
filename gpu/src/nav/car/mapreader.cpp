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

#include "mapreader.h"
#include "log.h"
#include <fstream>
#include <iostream>

MapReader::MapReader(int _thirdDimensionSize) {
    mapGraph.clear();
    mapGraph.resize(8 * 1024 * 8 * 1024);
    this->thirdDimensionSize = _thirdDimensionSize;
}

void MapReader::readMap(std::string fileName) {
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
    this->mapX = stoi(line.substr(pos + 1));

    std::getline(file, line);
    line = removeCR(line);
    pos = line.find(' ');
    assert(pos != std::string::npos);
    assert(line.substr(0, pos) == "width");
    this->mapY = stoi(line.substr(pos + 1));

    std::getline(file, line);
    line = removeCR(line);
    assert(line == "map");

    /*
     *  @ or T: Obstacle
     *  .: Free
     *              Width(Y)
     *            ◄─────────►
     *           ▲ ....@@@@@@
     *           │ .....@@@@@
     * Height(X) │ ......@@@@
     *           │ .......@@@
     *           ▼ ........@@
     *
     */

    MAP_CELL *buf = this->mapGraph.data();
    for (int x = 0; x < this->mapX; x++) {
        std::getline(file, line);
        line = removeCR(line);
        assert(static_cast<int>(line.length()) == this->mapY);

        for (int y = 0; y < this->mapY; y++) {
            char point = line[y];
            assert(point == '.' || point == '@' || point == 'T');
            bool obstacle = (point == '@' || point == 'T');
            MAP_CELL cellVal =
                (obstacle ? this->MAP_OBSTACLE_VAL : this->MAP_FREE_VAL);
            for (int d3 = 0; d3 < this->thirdDimensionSize; d3++) {
                *buf++ = cellVal;
            }
        }
    }

    file.close();
}

void MapReader::printMap() {
    MAP_CELL *buf = this->mapGraph.data();
    for (int i = 0; i < this->mapX; i++) {
        for (int j = 0; j < this->mapY; j++) {
            int val = static_cast<int>(*buf);
            std::cout << val << " ";
            buf++;
        }
        std::cout << std::endl;
    }
}

int MapReader::getMapX() const { return this->mapX; }

int MapReader::getMapY() const { return this->mapY; }

const MAP_CELL *MapReader::getGraph() const { return this->mapGraph.data(); }

int MapReader::genRandFreeIdx() const {
    while (true) {
        int idx = rand() % (this->mapX * this->mapY);
        MAP_CELL c = this->mapGraph[idx];
        assert(c == this->MAP_FREE_VAL || c == this->MAP_OBSTACLE_VAL);
        if (c == this->MAP_FREE_VAL) return idx;
    }
}

bool MapReader::isIdxFree(int idx) const {
    MAP_CELL c = this->mapGraph[idx];
    assert(c == this->MAP_FREE_VAL || c == this->MAP_OBSTACLE_VAL);
    return (c == this->MAP_FREE_VAL);
}

bool MapReader::isPointFree(int x, int y) const {
    assert(this->isInRange(x, y));
    int idx = this->xythetaToIdx(x, y, 0);
    return this->isIdxFree(idx);
}

int MapReader::xythetaToIdx(int x, int y, int thetaIdx) const {
    assert(this->isInRange(x, y));
    assert(0 <= thetaIdx < this->thirdDimensionSize);
    return (x * this->mapY + y) * this->thirdDimensionSize + thetaIdx;
}

void MapReader::idxToXYTheta(int idx, int *x, int *y, int *thetaIdx) const {
    assert(this->isIdxValid(idx));
    *thetaIdx = idx % this->thirdDimensionSize;
    *x = (idx / thirdDimensionSize) / this->mapY;
    *y = (idx / thirdDimensionSize) % this->mapY;
    assert(idx == this->xythetaToIdx(*x, *y, *thetaIdx));
}

bool MapReader::isInRange(int x, int y) const {
    return 0 <= x < this->mapY && 0 <= y < this->mapY;
}

bool MapReader::isIdxValid(int idx) const {
    return idx < this->mapX * this->mapY;
}
