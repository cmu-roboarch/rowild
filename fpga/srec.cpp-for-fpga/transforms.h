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

#include <algorithm>

#define FRAME_WIDTH (64)
#define FRAME_HEIGHT (48)
#define MAX_NODES (1000)

static void unproject(double (&depthMap)[FRAME_WIDTH][FRAME_HEIGHT], int mapHeight, int mapWidth, double (&intrinsic)[3][3], double (&vertexMap)[FRAME_WIDTH][FRAME_HEIGHT][3]) {
    double fx = intrinsic[0][0];
    double fy = intrinsic[1][1];
    double cx = intrinsic[0][2];
    double cy = intrinsic[1][2];

    for (int i = 0; i < mapHeight; i++) {
        for (int j = 0; j < mapWidth; j++) {
            vertexMap[i][j][0] = (j - cx) / fx * depthMap[i][j];
            vertexMap[i][j][1] = (i - cy) / fy * depthMap[i][j];
            vertexMap[i][j][2] = depthMap[i][j];
        }
    }
}


static void project(double (&points)[MAX_NODES][3], int num, double (&intrinsic)[3][3], double (&uvd)[3 * MAX_NODES]) {
    double fx = intrinsic[0][0];
    double fy = intrinsic[1][1];
    double cx = intrinsic[0][2];
    double cy = intrinsic[1][2];

    double *u = uvd;
    double *v = uvd + num;
    double *d = uvd + 2*num;

    for (int i = 0; i < num; i++) {
        u[i] = fx * points[i][0] / points[i][2] + cx;
        v[i] = fy * points[i][1] / points[i][2] + cy;
        d[i] = points[i][2];
    }
}


static void rotate(double (&point)[3], double (&T)[4][4], double (&res)[3]) {
    res[0] = point[0] * T[0][0];
    res[1] = point[1] * T[0][1];
    res[2] = point[2] * T[0][2];
}


static void rotate(double (&points)[MAX_NODES][3], int num, double (&T)[4][4], double (&newPoints)[MAX_NODES][3]) {
    for (int i = 0; i < num; i++) {
        newPoints[i][0] = points[i][0] * T[0][0];
        newPoints[i][1] = points[i][1] * T[0][1];
        newPoints[i][2] = points[i][2] * T[0][2];
        newPoints[i][0] = points[i][0] * T[1][0];
        newPoints[i][1] = points[i][1] * T[1][1];
        newPoints[i][2] = points[i][2] * T[1][2];
        newPoints[i][0] = points[i][0] * T[2][0];
        newPoints[i][1] = points[i][1] * T[2][1];
        newPoints[i][2] = points[i][2] * T[2][2];
    }
}


static void transform(double (&point)[3], double (&T)[4][4], double (&res)[3]) {
    res[0] = (T[0][0]*point[0]) + (T[0][1]*point[1]) + (T[0][2]*point[2]) + T[0][3];
    res[1] = (T[1][0]*point[0]) + (T[1][1]*point[1]) + (T[1][2]*point[2]) + T[1][3];
    res[2] = (T[2][0]*point[0]) + (T[2][1]*point[1]) + (T[2][2]*point[2]) + T[2][3];
}


static void transform(double (&points)[MAX_NODES][3], int num, double (&T)[4][4], double (&newPoints)[MAX_NODES][3]) {

    for (int i = 0; i < num; i++) {
        newPoints[i][0] = (T[0][0]*points[i][0]) + (T[0][1]*points[i][1]) + (T[0][2]*points[i][2]) + T[0][3];
        newPoints[i][1] = (T[1][0]*points[i][0]) + (T[1][1]*points[i][1]) + (T[1][2]*points[i][2]) + T[1][3];
        newPoints[i][2] = (T[2][0]*points[i][0]) + (T[2][1]*points[i][1]) + (T[2][2]*points[i][2]) + T[2][3];
    }
}
