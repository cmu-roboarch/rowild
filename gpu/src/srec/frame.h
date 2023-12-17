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
#include "transforms.h"
#include <string>

class Frame {
  public:
    Frame(std::string path, int number, double depthScale, double colorScale,
          int downsample);
    ~Frame();

    void printBuffers(std::string header) const;

    int getHeight() const { return this->frameHeight; }

    int getWidth() const { return this->frameWidth; }

    double **getDepthMap() const { return this->depthMap; }

    double ***getColorMap() const { return this->colorMap; }

    double ***getNormalMap() const { return this->normalMap; }

    void calcVertexMap(double **intrinsicMatrix) {
        this->vertexMap =
            transforms::unproject(this->depthMap, this->frameHeight,
                                  this->frameWidth, intrinsicMatrix);
    }

    double ***getVertexMap() const {
        assert(this->vertexMap);
        return this->vertexMap;
    }

  private:
    int frameHeight, frameWidth;

    double **depthMap;
    double ***colorMap;
    double ***normalMap;
    double ***vertexMap;
};
