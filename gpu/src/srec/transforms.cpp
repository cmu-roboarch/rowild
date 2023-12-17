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

#include "transforms.h"
#include "rowild_utils.h"
#include <algorithm>

namespace transforms {

double ***unproject(double **depthMap, int mapHeight, int mapWidth,
                    double **intrinsic) {
    double fx = intrinsic[0][0];
    double fy = intrinsic[1][1];
    double cx = intrinsic[0][2];
    double cy = intrinsic[1][2];

    double ***vertexMap = new double **[mapHeight];
    for (int i = 0; i < mapHeight; i++) {
        vertexMap[i] = new double *[mapWidth];
        for (int j = 0; j < mapWidth; j++) {
            vertexMap[i][j] = new double[3];
        }
    }

    for (int i = 0; i < mapHeight; i++) {
        for (int j = 0; j < mapWidth; j++) {
            vertexMap[i][j][0] = (j - cx) / fx * depthMap[i][j];
            vertexMap[i][j][1] = (i - cy) / fy * depthMap[i][j];
            vertexMap[i][j][2] = depthMap[i][j];
        }
    }

    return vertexMap;
}

double *project(double **points, int num, double **intrinsic) {
    double fx = intrinsic[0][0];
    double fy = intrinsic[1][1];
    double cx = intrinsic[0][2];
    double cy = intrinsic[1][2];

    double *uvd = new double[3 * num];
    double *u = uvd;
    double *v = uvd + num;
    double *d = uvd + 2 * num;

    for (int i = 0; i < num; i++) {
        u[i] = fx * points[i][0] / points[i][2] + cx;
        v[i] = fy * points[i][1] / points[i][2] + cy;
        d[i] = points[i][2];
    }

    return uvd;
}

double *rotate(double *point, double **T) {
    double *res = new double[3];
    res[0] = (T[0][0] * point[0]) + (T[0][1] * point[1]) + (T[0][2] * point[2]);
    res[1] = (T[1][0] * point[0]) + (T[1][1] * point[1]) + (T[1][2] * point[2]);
    res[2] = (T[2][0] * point[0]) + (T[2][1] * point[1]) + (T[2][2] * point[2]);
    return res;
}

double **rotate(double **points, int num, double **T) {
    double **newPoints = new double *[num];
    for (int i = 0; i < num; i++)
        newPoints[i] = new double[3];

    for (int i = 0; i < num; i++) {
        newPoints[i] = rotate(points[i], T);
    }

    return newPoints;
}

double *transform(double *point, double **T) {
    double *res = new double[3];
    res[0] = (T[0][0] * point[0]) + (T[0][1] * point[1]) +
             (T[0][2] * point[2]) + T[0][3];
    res[1] = (T[1][0] * point[0]) + (T[1][1] * point[1]) +
             (T[1][2] * point[2]) + T[1][3];
    res[2] = (T[2][0] * point[0]) + (T[2][1] * point[1]) +
             (T[2][2] * point[2]) + T[2][3];
    return res;
}

double **transform(double **points, int num, double **T) {
    double **newPoints = new double *[num];
    for (int i = 0; i < num; i++)
        newPoints[i] = new double[3];

    for (int i = 0; i < num; i++) {
        newPoints[i] = transform(points[i], T);
    }

    return newPoints;
}

} // namespace transforms
