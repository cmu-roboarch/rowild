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

#include <fstream>
#include <string>
#include <vector>
#include <sstream>

class EnvMap {
 public:
     explicit EnvMap(std::string fileName) {
         std::ifstream mapFile(fileName);

         std::string strToken;
         int intToken;

         mapFile >> strToken >> intToken;
         this->sizeX = intToken;

         mapFile >> strToken >> intToken;
         this->sizeY = intToken;

         mapFile >> strToken >> intToken;
         this->resolution = intToken;

         this->sizeX /= resolution;
         this->sizeY /= resolution;

         std::getline(mapFile, strToken);

         this->map = new double*[this->sizeX];
         for (int i = 0; i < this->sizeX; i++) {
             this->map[i] = new double[this->sizeY];
         }

         freeXs = new std::vector<int>();
         freeYs = new std::vector<int>();

         for (int i = 0; i < this->sizeX; i++) {
             std::getline(mapFile, strToken);
             std::stringstream row(strToken);

             int j = 0;
             double e;
             while (row >> e) {
                 this->map[i][j] = e;
                 if (e == 0) {
                     this->freeXs->push_back(i);
                     this->freeYs->push_back(j);
                 }
                 j++;
             }
         }
         mapFile.close();
     }

     ~EnvMap() {
         for (int i = 0; i < this->sizeX; i++) {
             delete[] this->map[i];
         }
         delete[] map;

         delete freeXs;
         delete freeYs;
     }

     int getX() const {
         return this->sizeX;
     }

     int getY() const {
         return this->sizeY;
     }

     double getResolution() const {
         return this->resolution;
     }

     bool isValid(int x, int y) const {
         return ((x >= 0) && (x < this->sizeX) \
                 && (y >= 0) && (y < this->sizeY));
     }

     bool isFree(int x, int y) const {
         return this->map[x][y] == 0;
     }

     double getProb(int x, int y) const {
         return this->map[x][y];
     }

     void printMap() const {
         for (int i = 0; i < this->sizeX; i++) {
             for (int j = 0; j < this->sizeY; j++) {
                 printf("%.3f ", this->map[i][j]);
             }
             printf("\n");
         }
     }

     const double **getMap() const {
         return (const double**)this->map;
     }

     const std::vector<int> *getFreeXs() const {
         return (const std::vector<int> *)this->freeXs;
     }

     const std::vector<int> *getFreeYs() const {
         return (const std::vector<int> *)this->freeYs;
     }

 private:
     int sizeX, sizeY;
     int resolution;
     double **map;
     std::vector<int> *freeXs, *freeYs;
};
