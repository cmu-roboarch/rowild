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

#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>

class OccupancyGrid {
private:
    double grid[100][100];
    int gridSize;
    double probOccupied;
    double probFree;

public:
    OccupancyGrid(int size, double initProb, double _probOccupied, double _probFree)
        : gridSize(size),
          probOccupied(_probOccupied),
          probFree(_probFree) {
              for (int i = 0; i < size; i++) {
                  for (int j = 0; j < size; j++) {
                    grid[i][j] = initProb;
                  }
              }
          }

    void update(double robotX, double robotY, double angle, double measurement) {
        double dx = cos(angle);
        double dy = sin(angle);
        
        for (int i = 0; i < measurement; i++) {
            int x = static_cast<int>(robotX + i * dx);
            int y = static_cast<int>(robotY + i * dy);
            
            if (x >= 0 && x < gridSize && y >= 0 && y < gridSize) {
                grid[x][y] = grid[x][y] * (1 - probOccupied) / (1 - grid[x][y]);
            }
        }

        int obstacleX = static_cast<int>(robotX + measurement * dx);
        int obstacleY = static_cast<int>(robotY + measurement * dy);

        if (obstacleX >= 0 && obstacleX < gridSize && obstacleY >= 0 && obstacleY < gridSize) {
            grid[obstacleX][obstacleY] = grid[obstacleX][obstacleY] * probOccupied / (1 - grid[obstacleX][obstacleY]);
        }
    }
};
