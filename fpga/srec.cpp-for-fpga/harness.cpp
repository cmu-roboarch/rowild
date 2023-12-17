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

#include <fstream>
#include <string>
#include <vector>
#include "frame.h"
#include "map.h"
#include "reconstruct.h"
#include "utils.h"


int main() {
    double intrinsicMatrix[3][3] = {{418.2, 0, 0}, {0, 480.0, 0}, {319.5, 239.5, 1}};
    double tCameraToWorld[4][4] = {{1.0, 2.0, 3.0, 4.0}, {5.0, 6.0, 7.0, 8.0}, {2.0, 3.0, 4.0, 5.0}, {3.0, 3.0, 2.0, 1.5}};
    double tWorldToCamera[4][4];

    invert4x4Matrix(tCameraToWorld, tWorldToCamera);

    double depthScale = 256.0 / 5000.0;
    double colorScale = 1.0;

    Map m;

    Frame f(1, depthScale, colorScale, 1);
        // f.calcVertexMap(intrinsicMatrix);
    icp(m, f, intrinsicMatrix, tWorldToCamera);
        // invert4x4Matrix(tCameraToWorld, tWorldToCamera);
        //	m.fuse(f, intrinsicMatrix, tCameraToWorld);


    return 0;
}
