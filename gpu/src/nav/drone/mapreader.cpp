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
    constexpr int MAX_MAP_ELEMENTS = 8 * 1024 * 8 * 1024;
    this->mapGraph.clear();
    this->mapGraph.resize(MAX_MAP_ELEMENTS);
    std::fill_n(this->mapGraph.begin(), MAX_MAP_ELEMENTS, this->MAP_FREE_VAL);
}

void MapReader::readMap(std::string fileName) {
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

    this->mapX = this->maxX - this->minX + 1;
    this->mapY = this->maxY - this->minY + 1;
    this->mapZ = this->maxZ - this->minZ + 1;

    int x, y, z;
    while (file >> x >> y >> z) {
        int idx =
            this->xyzToIdx(x - this->minX, y - this->minY, z - this->minZ);
        this->mapGraph[idx] = this->MAP_OBSTACLE_VAL;
    }

    file.close();
}

int MapReader::getMapX() const { return this->mapX; }

int MapReader::getMapY() const { return this->mapY; }

int MapReader::getMapZ() const { return this->mapZ; }

int MapReader::getStartX() const { return this->startX; }

int MapReader::getStartY() const { return this->startY; }

int MapReader::getStartZ() const { return this->startZ; }

int MapReader::getEndX() const { return this->endX; }

int MapReader::getEndY() const { return this->endY; }

int MapReader::getEndZ() const { return this->endZ; }

void MapReader::setStartEndPoints(int sx, int sy, int sz, int ex, int ey,
                                  int ez) {
    assert(this->isPointFree(sx, sy, sz));
    assert(this->isPointFree(ex, ey, ez));

    this->startX = sx;
    this->startY = sy;
    this->startZ = sz;
    this->endX = ex;
    this->endY = ey;
    this->endZ = ez;
}

void MapReader::setRandomStartEndPoints() {
    int startIdx = this->genRandFreeCell();
    this->idxToXYZ(startIdx, &(this->startX), &(this->startY), &(this->startZ));
    assert(this->isIdxValid(startIdx));
    assert(this->isPointFree(this->startX, this->startY, this->startZ));

    int endIdx;
    do {
        endIdx = this->genRandFreeCell();
    } while (startIdx == endIdx);
    this->idxToXYZ(endIdx, &(this->endX), &(this->endY), &(this->endZ));
    assert(this->isIdxValid(endIdx));
    assert(this->isPointFree(this->endX, this->endY, this->endZ));
}

const MAP_CELL *MapReader::getGraph() const { return this->mapGraph.data(); }

int MapReader::genRandFreeCell() const {
    while (true) {
        int idx = rand() % (this->mapX * this->mapY * this->mapZ);
        MAP_CELL c = this->mapGraph[idx];
        assert(c == this->MAP_FREE_VAL || c == this->MAP_OBSTACLE_VAL);
        if (c == this->MAP_FREE_VAL) return idx;
    }
}

bool MapReader::isIdxFree(int idx) const {
    assert(this->isIdxValid(idx));
    MAP_CELL c = this->mapGraph[idx];
    assert(c == this->MAP_FREE_VAL || c == this->MAP_OBSTACLE_VAL);
    return (c == this->MAP_FREE_VAL);
}

bool MapReader::isPointFree(int x, int y, int z) const {
    assert(this->isInRange(x, y, z));
    int idx = this->xyzToIdx(x, y, z);
    return this->isIdxFree(idx);
}

int MapReader::xyzToIdx(int x, int y, int z) const {
    assert(this->isInRange(x, y, z));
    return (x * this->mapY + y) * this->mapZ + z;
}

void MapReader::idxToXYZ(int idx, int *x, int *y, int *z) const {
    assert(this->isIdxValid(idx));
    *x = idx / (this->mapY * this->mapZ);
    *y = (idx / this->mapZ) % this->mapY;
    *z = idx % this->mapZ;
    assert(idx == this->xyzToIdx(*x, *y, *z));
}

bool MapReader::isInRange(int x, int y, int z) const {
    return 0 <= x && x < this->mapX && 0 <= y && y < this->mapY && 0 <= z &&
           z < this->mapZ;
}

bool MapReader::isIdxValid(int idx) const {
    return idx < this->mapX * this->mapY * this->mapZ;
}
