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

class Environment {
  public:
    explicit Environment(const char *fileName) {
        std::ifstream file(fileName);
        assert(file.good());
        std::string line;

        getline(file, line);
        mapX = stoi(line);

        getline(file, line);
        mapY = stoi(line);

        map = new int *[mapX];
        for (int i = 0; i < mapX; i++) {
            map[i] = new int[mapY];
        }

        for (int i = 0; i < mapX; i++) {
            getline(file, line);
            std::stringstream row(line);
            assert(row.good());
            std::string value;
            int j = 0;
            while (row >> value) {
                map[i][j] = stoi(value);
                assert(map[i][j] == 0 || map[i][j] == 1);
                j++;
            }
            assert(j == mapY);
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

    int getCellCost(int x, int y) const { return map[x][y]; }

    bool isFree(int x, int y) const { return (map[x][y] == 0); }

    bool isValid(int x, int y) const {
        return ((x >= 0) && (x < mapX) && (y >= 0) && (y < mapY));
    }

  private:
    int mapX, mapY;
    int **map;
};
