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

MapReader::MapReader() {
    mapGraph.clear();
    mapGraph.resize(8 * 1024 * 8 * 1024);
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
    this->mapHeight = stoi(line.substr(pos + 1));

    std::getline(file, line);
    line = removeCR(line);
    pos = line.find(' ');
    assert(pos != std::string::npos);
    assert(line.substr(0, pos) == "width");
    this->mapWidth = stoi(line.substr(pos + 1));

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
     * >> Width is Y and Height is X in data structures
     * >> Top-left corner is (X=0, Y=0)
     */

    MAP_CELL *buf = this->mapGraph.data();
    for (int x = 0; x < this->mapHeight; x++) {
        std::getline(file, line);
        line = removeCR(line);
        assert(static_cast<int>(line.length()) == this->mapWidth);

        for (int y = 0; y < this->mapWidth; y++) {
            char point = line[y];
            assert(point == '.' || point == '@' || point == 'T');
            bool obstacle = (point == '@' || point == 'T');
            *buf++ = (obstacle ? this->MAP_OBSTACLE_VAL : this->MAP_FREE_VAL);
        }
    }

    file.close();
}

void MapReader::generateZigzagMap(int height, int width) {
    this->mapHeight = height;
    this->mapWidth = width;
    this->startX = 0;
    this->startY = this->mapWidth / 2;
    this->endX = this->mapHeight - 1;
    this->endY = this->mapWidth / 2;

    int heightGap = 2;

    bool left = false;
    MAP_CELL *buf = this->mapGraph.data();
    for (int i = 0; i < this->mapHeight; ++i) {
        if (i && i % heightGap == 0) {
            if (left) *buf++ = this->MAP_FREE_VAL;
            for (int j = 0; j < this->mapWidth - 1; ++j) {
                *buf++ = this->MAP_OBSTACLE_VAL;
            }
            if (!left) *buf++ = this->MAP_FREE_VAL;
            left = !left;
        } else {
            for (int j = 0; j < this->mapWidth; ++j) {
                *buf++ = this->MAP_FREE_VAL;
            }
        }
    }
}

void MapReader::printMap() {
    MAP_CELL *buf = this->mapGraph.data();
    for (int i = 0; i < this->mapHeight; i++) {
        for (int j = 0; j < this->mapWidth; j++) {
            int val = static_cast<int>(*buf);
            std::cout << val << " ";
            buf++;
        }
        std::cout << std::endl;
    }
}

int MapReader::getMapHeight() const { return this->mapHeight; }

int MapReader::getMapWidth() const { return this->mapWidth; }

int MapReader::getStartX() const { return this->startX; }

int MapReader::getStartY() const { return this->startY; }

int MapReader::getEndX() const { return this->endX; }

int MapReader::getEndY() const { return this->endY; }

void MapReader::setStartEndPoints(int sx, int sy, int ex, int ey) {
    assert(this->isValidPoint(sx, sy));
    assert(this->isValidPoint(ex, ey));

    this->startX = sx;
    this->startY = sy;
    this->endX = ex;
    this->endY = ey;
}

void MapReader::setRandomStartEndPoints() {
    int startIdx = this->genRandFreeCell();
    this->startX = startIdx / this->mapWidth;
    this->startY = startIdx % this->mapWidth;

    int endIdx;
    do {
        endIdx = this->genRandFreeCell();
    } while (startIdx == endIdx);
    this->endX = endIdx / this->mapWidth;
    this->endY = endIdx % this->mapWidth;
}

const MAP_CELL *MapReader::getGraph() const { return this->mapGraph.data(); }

int MapReader::genRandFreeCell() const {
    while (true) {
        int idx = rand() % (this->mapHeight * this->mapWidth);
        MAP_CELL c = this->mapGraph[idx];
        assert(c == this->MAP_FREE_VAL || c == this->MAP_OBSTACLE_VAL);
        if (c == this->MAP_FREE_VAL) return idx;
    }
}

bool MapReader::isValidIdx(int idx) const {
    MAP_CELL c = this->mapGraph[idx];
    assert(c == this->MAP_FREE_VAL || c == this->MAP_OBSTACLE_VAL);
    return (c == this->MAP_FREE_VAL);
}

bool MapReader::isValidPoint(int x, int y) const {
    int idx = x * this->mapWidth + y;
    return this->isValidIdx(idx);
}
