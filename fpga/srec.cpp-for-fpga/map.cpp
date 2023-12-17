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

#include "map.h"
#include "utils.h"
#include <algorithm>
#include <string>
#include "transforms.h"

#define PI (3.141592654)
#define TWO_PI (6.283185308)

template<typename T>
T degToRad(T angle) {
    return angle * (PI / 180);
}


template<typename T>
T wrapToPi(T angle) {
    while (angle > PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}

template<typename T, int x, int y, int z>
void reshape3dTo2d(T (&A)[FRAME_WIDTH][FRAME_HEIGHT][3], T (&newMatrix)[MAX_NODES][3]) {
    for (int i = 0; i < x; i++) {
        for (int j = 0; j < y; j++) {
            for (int k = 0; k < z; k++) {
                int newRow = i*y + j;
                int newCol = k;
                newMatrix[newRow][newCol] = A[i][j][k];
            }
        }
    }
}


Map::Map() {
    this->initialized = false;
    this->numPoints = 0;

    this->distDiff = 0.03;
    this->angleDiff = degToRad(5.0);
}


void Map::fuse(Frame f, double (&intrinsicMatrix)[3][3], double (&T)[4][4]) {
    int h = f.getHeight(), w = f.getWidth();
    double nPoints[MAX_NODES][3];
    double nNormals[MAX_NODES][3];
    double nColors[MAX_NODES][3];
    bool mergeBoard[FRAME_HEIGHT][FRAME_WIDTH] = {};

    reshape3dTo2d<double, FRAME_HEIGHT, FRAME_WIDTH, 3>(f.getVertexMap(), nPoints);
    reshape3dTo2d<double, FRAME_HEIGHT, FRAME_WIDTH, 3>(f.getNormalMap(), nNormals);
    reshape3dTo2d<double, FRAME_HEIGHT, FRAME_WIDTH, 3>(f.getColorMap(), nColors);

    if (!this->initialized) {
        this->add(h*w, nPoints, nNormals, nColors, T, mergeBoard, h, w);
    } else {
        int initialNodeCount = this->numPoints;

        double invT[4][4];
        invert4x4Matrix(T, invT);

        double transformedPoints[MAX_NODES][3];
        double rotatedNormals[MAX_NODES][3];
        double uvd[3 * MAX_NODES];
        transform(this->points, initialNodeCount, invT, transformedPoints);
        rotate(this->normals, initialNodeCount, invT, rotatedNormals);
        project(transformedPoints, initialNodeCount, intrinsicMatrix, uvd);

        double *u = uvd;
        double *v = uvd + initialNodeCount;
        double *d = uvd + 2 * initialNodeCount;

        auto fastNorm = [](double *s, double *t) {
            double dist = (s[0]-t[0]) * (s[0]-t[0]);
            dist += (s[1]-t[1]) * (s[1]-t[1]);
            dist += (s[2]-t[2]) * (s[2]-t[2]);
            return sqrt(dist);
        };

        // Merge
        int mergedNodes = 0;
        for (int i = 0; i < initialNodeCount; i++) {
            int tU = round(u[i]);
            int tV = round(v[i]);
            double tD = d[i];
            if ((tV < 0) || (tV >= h) || (tU < 0) || (tU >= w) || (tD < 0)) {
                continue;
            }

            double *src = transformedPoints[i];
            double *tar = f.getVertexMap()[tV][tU];
            double dist = fastNorm(src, tar);
            if (dist > this->distDiff) continue;

            double *rNorm = rotatedNormals[i];
            double *inputNorm = f.getNormalMap()[tV][tU];
            double dotProd = (rNorm[0] * inputNorm[0]) + (rNorm[1] * inputNorm[1]) + (rNorm[2] * inputNorm[2]);

            double angle = acos(dotProd);
            if (angle > this->angleDiff) continue;

            double w = this->weights[i];
            double *mergedColor = f.getColorMap()[tV][tU];

            // Manually unrolled all the loops to improve ILP
            double rMergedPoint[3];
            double __tar[3] = {tar[0], tar[1], tar[2]};
            transform(__tar, T, rMergedPoint);
            this->points[i][0] = (w * this->points[i][0] + rMergedPoint[0]) / (w + 1);
            this->points[i][1] = (w * this->points[i][1] + rMergedPoint[1]) / (w + 1);
            this->points[i][2] = (w * this->points[i][2] + rMergedPoint[2]) / (w + 1);

            double rMergedNorm[3];
            double __inputNorm[3] = {inputNorm[0], inputNorm[1], inputNorm[2]};
            rotate(__inputNorm, T, rMergedNorm);
            this->normals[i][0] = (w * this->normals[i][0] + rMergedNorm[0]) / (w + 1);
            this->normals[i][1] = (w * this->normals[i][1] + rMergedNorm[1]) / (w + 1);
            this->normals[i][2] = (w * this->normals[i][2] + rMergedNorm[2]) / (w + 1);
            double absNorm = (this->normals[i][0] * this->normals[i][0]) + (this->normals[i][1] * this->normals[i][1]) + (this->normals[i][2] * this->normals[i][2]);
            absNorm = sqrt(absNorm);
            this->normals[i][0] /= absNorm;
            this->normals[i][1] /= absNorm;
            this->normals[i][2] /= absNorm;

            this->colors[i][0] = (w * this->colors[i][0] + mergedColor[0]) / (w + 1);
            this->colors[i][1] = (w * this->colors[i][1] + mergedColor[1]) / (w + 1);
            this->colors[i][2] = (w * this->colors[i][2] + mergedColor[2]) / (w + 1);

            this->weights[i]++;

            mergeBoard[tV][tU] = true;
            mergedNodes++;

        }

        this->add(h*w - mergedNodes, nPoints, nNormals, nColors, T, mergeBoard, h, w);
    }
}


void Map::add(int num, double (&nPoints)[MAX_NODES][3], double (&nNormals)[MAX_NODES][3], double (&nColors)[MAX_NODES][3], double (&T)[4][4], bool (&mergeBoard)[FRAME_HEIGHT][FRAME_WIDTH], int h, int w) {
    if (num == 0) return;

    if (!this->initialized) {
        double tInitPoints[MAX_NODES][3];
        double rInitNorms[MAX_NODES][3];

        transform(nPoints, num, T, tInitPoints);
        rotate(nNormals, num, T, rInitNorms);

        for (int i = 0; i < num; i++) {
            this->points[i][0] = tInitPoints[i][0];
            this->points[i][1] = tInitPoints[i][1];
            this->points[i][2] = tInitPoints[i][2];

            this->normals[i][0] = rInitNorms[i][0];
            this->normals[i][1] = rInitNorms[i][1];
            this->normals[i][2] = rInitNorms[i][2];

            this->colors[i][0] = nColors[i][0];
            this->colors[i][1] = nColors[i][1];
            this->colors[i][2] = nColors[i][2];

            this->weights[i] = 1.0;
        }

        this->numPoints = num;
        this->initialized = true;
        return;
    }

    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            if (!mergeBoard[i][j]) {
                int idx = i*w + j;
                for (int k = 0; k < 3; k++) {
                    points[numPoints][k] = (T[k][0] * nPoints[idx][0]) + (T[k][1] * nPoints[idx][1]) + (T[k][2] * nPoints[idx][2]) + T[k][3];
                    normals[numPoints][k] = (T[k][0] * nNormals[idx][0]) + (T[k][1] * nNormals[idx][1]) + (T[k][2] * nNormals[idx][2]);
                    colors[numPoints][k] = nColors[idx][k];
                }
                this->numPoints++;
            }
        }
    }
}
