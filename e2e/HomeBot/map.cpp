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
#include "transforms.h"
#include "utils.h"
#include <algorithm>
#include <string>
#include <xmmintrin.h>

Map::Map() {
    this->initialized = false;
    this->numPoints = 0;

    this->distDiff = 0.03;
    this->angleDiff = degToRad(5.0);

    this->points = new double *[MAX_NODES];
    this->normals = new double *[MAX_NODES];
    this->colors = new double *[MAX_NODES];
    for (int i = 0; i < MAX_NODES; i++) {
        this->points[i] = new double[3];
        this->normals[i] = new double[3];
        this->colors[i] = new double[3];
    }
    this->weights = new double[MAX_NODES];
}

Map::~Map() {
    for (int i = 0; i < MAX_NODES; i++) {
        delete[] this->points[i];
        delete[] this->normals[i];
        delete[] this->colors[i];
    }
    delete[] this->points;
    delete[] this->normals;
    delete[] this->colors;
    delete[] this->weights;
}

void Map::fuse(Frame *f, double **intrinsicMatrix, double **T) {
    int h = f->getHeight(), w = f->getWidth();
    double **nPoints = reshape3dTo2d(f->getVertexMap(), h, w, 3);
    double **nNormals = reshape3dTo2d(f->getNormalMap(), h, w, 3);
    double **nColors = reshape3dTo2d(f->getColorMap(), h, w, 3);

    if (!this->initialized) {
        this->add(h * w, nPoints, nNormals, nColors, T, NULL, h, w);
    } else {
        int initialNodeCount = this->numPoints;

        double **invT = new double *[4];
        for (int i = 0; i < 4; i++)
            invT[i] = new double[4];
        matrixInverse<double, 4>(T, invT);

        double **transformedPoints =
            transforms::transform(this->points, initialNodeCount, invT);

        double **rotatedNormals =
            transforms::rotate(this->normals, initialNodeCount, invT);

        double *uvd = transforms::project(transformedPoints, initialNodeCount,
                                          intrinsicMatrix);

        double *u = uvd;
        double *v = uvd + initialNodeCount;
        double *d = uvd + 2 * initialNodeCount;

        auto fastNorm = [](double *s, double *t) {
            double dist = (s[0] - t[0]) * (s[0] - t[0]);
            dist += (s[1] - t[1]) * (s[1] - t[1]);
            dist += (s[2] - t[2]) * (s[2] - t[2]);
            return sqrt(dist);
        };

        // A h * w board to keep track of merged/new points
        bool **mergeBoard = new bool *[h];
        for (int i = 0; i < h; i++) {
            mergeBoard[i] = new bool[w]();
        }

        // Merge
        int mergedNodes = 0;
#pragma omp parallel for num_threads(4)
        for (int i = 0; i < initialNodeCount; i++) {
            int tU = round(u[i]);
            int tV = round(v[i]);
            double tD = d[i];
            if ((tV < 0) || (tV >= h) || (tU < 0) || (tU >= w) || (tD < 0)) {
                continue;
            }

            double *src = transformedPoints[i];
            double *tar = f->getVertexMap()[tV][tU];
            double dist = fastNorm(src, tar);
            if (dist > this->distDiff) continue;

            double *rNorm = rotatedNormals[i];
            double *inputNorm = f->getNormalMap()[tV][tU];
            double dotProd = (rNorm[0] * inputNorm[0]) +
                             (rNorm[1] * inputNorm[1]) +
                             (rNorm[2] * inputNorm[2]);

            double angle = acos(dotProd);
            if (angle > this->angleDiff) continue;

            double w = this->weights[i];
            double *mergedColor = f->getColorMap()[tV][tU];

            // Manually unrolled all the loops to improve ILP
            double *rMergedPoint = transforms::transform(tar, T);
            this->points[i][0] =
                (w * this->points[i][0] + rMergedPoint[0]) / (w + 1);
            this->points[i][1] =
                (w * this->points[i][1] + rMergedPoint[1]) / (w + 1);
            this->points[i][2] =
                (w * this->points[i][2] + rMergedPoint[2]) / (w + 1);

            double *rMergedNorm = transforms::rotate(inputNorm, T);
            this->normals[i][0] =
                (w * this->normals[i][0] + rMergedNorm[0]) / (w + 1);
            this->normals[i][1] =
                (w * this->normals[i][1] + rMergedNorm[1]) / (w + 1);
            this->normals[i][2] =
                (w * this->normals[i][2] + rMergedNorm[2]) / (w + 1);

            double absNorm = (this->normals[i][0] * this->normals[i][0]) +
                             (this->normals[i][1] * this->normals[i][1]) +
                             (this->normals[i][2] * this->normals[i][2]);

            absNorm = sqrt(absNorm);
            this->normals[i][0] /= absNorm;
            this->normals[i][1] /= absNorm;
            this->normals[i][2] /= absNorm;

            this->colors[i][0] =
                (w * this->colors[i][0] + mergedColor[0]) / (w + 1);
            this->colors[i][1] =
                (w * this->colors[i][1] + mergedColor[1]) / (w + 1);
            this->colors[i][2] =
                (w * this->colors[i][2] + mergedColor[2]) / (w + 1);

            this->weights[i]++;

            mergeBoard[tV][tU] = true;
            mergedNodes++;

            delete[] rMergedPoint;
            delete[] rMergedNorm;
        }

        this->add(h * w - mergedNodes, nPoints, nNormals, nColors, T,
                  mergeBoard, h, w);

        delete[] invT[0];
        delete[] invT[1];
        delete[] invT[2];
        delete[] invT[3];

        for (int i = 0; i < initialNodeCount; i++) {
            delete[] transformedPoints[i];
            delete[] rotatedNormals[i];
        }

        delete[] transformedPoints;
        delete[] rotatedNormals;
        delete[] uvd;

        for (int i = 0; i < h; i++) {
            delete[] mergeBoard[i];
        }
        delete[] mergeBoard;
    }

    for (int i = 0; i < h * w; i++) {
        delete[] nPoints[i];
        delete[] nNormals[i];
        delete[] nColors[i];
    }
    delete[] nPoints;
    delete[] nNormals;
    delete[] nColors;
}

void Map::add(int num, double **nPoints, double **nNormals, double **nColors,
              double **T, bool **mergeBoard, int h, int w) {
    if (num == 0) return;

    if (!this->initialized) {
        assert(num <= MAX_NODES);

        double **tInitPoints = transforms::transform(nPoints, num, T);
        double **rInitNorms = transforms::rotate(nNormals, num, T);

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

        for (int i = 0; i < num; i++) {
            delete[] tInitPoints[i];
            delete[] rInitNorms[i];
        }
        delete[] tInitPoints;
        delete[] rInitNorms;

        return;
    }

    assert(this->numPoints > 0 && this->numPoints + num <= MAX_NODES);

    for (int i = 0; i < h; i++) {
        // Cache pollution is likely; disable for sparse images?
        _mm_prefetch(&nPoints[i + 1], _MM_HINT_ET1);
        _mm_prefetch(&nNormals[i + 1], _MM_HINT_ET1);
        _mm_prefetch(&nColors[i + 1], _MM_HINT_ET1);
        for (int j = 0; j < w; j++) {
            if (!mergeBoard[i][j]) {
                int idx = i * w + j;
                this->points[this->numPoints] =
                    transforms::transform(nPoints[idx], T);
                this->normals[this->numPoints] =
                    transforms::rotate(nNormals[idx], T);
                this->colors[this->numPoints] = new double[3]{
                    nColors[idx][0], nColors[idx][1], nColors[idx][2]};
                this->numPoints++;
            }
        }
    }
}

void Map::printMap(std::string header) const {
    printf("%s:\n", header.c_str());
    printf("points:\n");
    for (int i = 0; i < this->numPoints; i++) {
        for (int j = 0; j < 3; j++) {
            printf("%.2f ", this->points[i][j]);
        }
        printf("\n");
    }

    printf("\nnormals:\n");
    for (int i = 0; i < this->numPoints; i++) {
        for (int j = 0; j < 3; j++) {
            printf("%.2f ", this->normals[i][j]);
        }
        printf("\n");
    }

    printf("\ncolors:\n");
    for (int i = 0; i < this->numPoints; i++) {
        for (int j = 0; j < 3; j++) {
            printf("%.2f ", this->colors[i][j]);
        }
        printf("\n");
    }

    printf("\nweights:\n");
    for (int i = 0; i < this->numPoints; i++) {
        printf("%.2f\n", this->weights[i]);
    }

    printf("\n");
}
