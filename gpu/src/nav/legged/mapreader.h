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
#include <stdint.h>
#include <string>
#include <vector>

typedef uint8_t MAP_CELL;

class MapReader {
  public:
    MapReader();
    void readMap(std::string fileName);
    void generateZigzagMap(int mapHeight, int mapWidth);
    void printMap();
    int getMapHeight() const;
    int getMapWidth() const;
    int getStartX() const;
    int getStartY() const;
    int getEndX() const;
    int getEndY() const;
    void setStartEndPoints(int sx, int sy, int ex, int ey);
    void setRandomStartEndPoints();
    const MAP_CELL *getGraph() const;

  private:
    int genRandFreeCell() const;
    bool isValidIdx(int idx) const;
    bool isValidPoint(int x, int y) const;

    const MAP_CELL MAP_FREE_VAL = 0xFF;
    const MAP_CELL MAP_OBSTACLE_VAL = 0;
    std::vector<uint8_t> mapGraph;

    int mapHeight, mapWidth;
    int startX, startY;
    int endX, endY;
};
