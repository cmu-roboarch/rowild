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

#include <string>
#include "transforms.h"

class Frame {
 public:
     Frame(int _number, double _depthScale, double _colorScale, int _downsample) :
         number(_number), depthScale(_depthScale), colorScale(_colorScale), downsample(_downsample) {};

     int getHeight() const {
         return FRAME_HEIGHT;
     }

     int getWidth() const {
         return FRAME_WIDTH;
     }

     double (&getDepthMap())[FRAME_WIDTH][FRAME_HEIGHT] {
         return this->depthMap;
     }

     double (&getColorMap())[FRAME_WIDTH][FRAME_HEIGHT][3] {
         return this->colorMap;
     }

     double (&getNormalMap())[FRAME_WIDTH][FRAME_HEIGHT][3] {
         return this->normalMap;
     }

     void calcVertexMap(double (&intrinsicMatrix)[3][3]) {
         unproject(this->depthMap, FRAME_HEIGHT, FRAME_WIDTH, intrinsicMatrix, this->vertexMap);
     }

     double (&getVertexMap())[FRAME_WIDTH][FRAME_HEIGHT][3] {
         return this->vertexMap;
     }

 private:
     int number;
     double depthScale, colorScale;
     int downsample;
     double depthMap[FRAME_WIDTH][FRAME_HEIGHT];
     double colorMap[FRAME_WIDTH][FRAME_HEIGHT][3];
     double normalMap[FRAME_WIDTH][FRAME_HEIGHT][3];
     double vertexMap[FRAME_WIDTH][FRAME_HEIGHT][3];
};
