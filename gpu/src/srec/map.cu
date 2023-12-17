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
#include "rowild_utils.h"
#include "transforms.h"
#include <algorithm>
#include <cuda_runtime.h>
#include <math.h>
#include <string>

Map::Map() {
    this->initialized = false;
    this->numPoints = 0;

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

__global__ void mergeKernel(double *points, double *normals, double *colors,
                            double *weights, double *vertexMap,
                            double *normalMap, double *colorMap,
                            bool *mergeBoard, double *invT, double *T,
                            int initialNodeCount, double fx, double fy,
                            double cx, double cy, int h, int w, double distDiff,
                            double angleDiff, int *d_mergedNodes) {

    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= initialNodeCount) return;

    int idx3 = i * 3; // Common index pattern for accessing 3D vectors

    double tp0 = (invT[0] * points[idx3]) + (invT[1] * points[idx3 + 1]) +
                 (invT[2] * points[idx3 + 2]) + invT[3];
    double tp1 = (invT[4] * points[idx3]) + (invT[5] * points[idx3 + 1]) +
                 (invT[6] * points[idx3 + 2]) + invT[7];
    double tp2 = (invT[8] * points[idx3]) + (invT[9] * points[idx3 + 1]) +
                 (invT[10] * points[idx3 + 2]) + invT[11];

    double rn0 = (invT[0] * normals[idx3]) + (invT[1] * normals[idx3 + 1]) +
                 (invT[2] * normals[idx3 + 2]);
    double rn1 = (invT[4] * normals[idx3]) + (invT[5] * normals[idx3 + 1]) +
                 (invT[6] * normals[idx3 + 2]);
    double rn2 = (invT[8] * normals[idx3]) + (invT[9] * normals[idx3 + 1]) +
                 (invT[10] * normals[idx3 + 2]);

    int tU = round(fx * tp0 / tp2 + cx);
    int tV = round(fy * tp1 / tp2 + cy);
    double tD = tp2;
    if ((tV < 0) || (tV >= h) || (tU < 0) || (tU >= w) || (tD < 0)) {
        return;
    }

    int mapIdx = (tV * w + tU) * 3;
    double tar0 = vertexMap[mapIdx];
    double tar1 = vertexMap[mapIdx + 1];
    double tar2 = vertexMap[mapIdx + 2];
    double dist =
        sqrt((tp0 - tar0) * (tp0 - tar0) + (tp1 - tar1) * (tp1 - tar1) +
             (tp2 - tar2) * (tp2 - tar2));
    if (dist > distDiff) {
        return;
    }

    double in0 = normalMap[mapIdx];
    double in1 = normalMap[mapIdx + 1];
    double in2 = normalMap[mapIdx + 2];
    double dotProd = (rn0 * in0) + (rn1 * in1) + (rn2 * in2);

    double angle = acos(dotProd);
    if (angle > angleDiff) {
        return;
    }

    double weight = weights[i];
    double mc0 = colorMap[mapIdx];
    double mc1 = colorMap[mapIdx + 1];
    double mc2 = colorMap[mapIdx + 2];

    double rmp0 = (T[0] * tar0) + (T[1] * tar1) + (T[2] * tar2) + T[3];
    double rmp1 = (T[4] * tar0) + (T[5] * tar1) + (T[6] * tar2) + T[7];
    double rmp2 = (T[8] * tar0) + (T[9] * tar1) + (T[10] * tar2) + T[11];

    points[idx3] = (weight * points[idx3] + rmp0) / (weight + 1);
    points[idx3 + 1] = (weight * points[idx3 + 1] + rmp1) / (weight + 1);
    points[idx3 + 2] = (weight * points[idx3 + 2] + rmp2) / (weight + 1);

    double rmn0 = (T[0] * in0) + (T[1] * in1) + (T[2] * in2);
    double rmn1 = (T[4] * in0) + (T[5] * in1) + (T[6] * in2);
    double rmn2 = (T[8] * in0) + (T[9] * in1) + (T[10] * in2);

    normals[idx3] = (weight * normals[idx3] + rmn0) / (weight + 1);
    normals[idx3 + 1] = (weight * normals[idx3 + 1] + rmn1) / (weight + 1);
    normals[idx3 + 2] = (weight * normals[idx3 + 2] + rmn2) / (weight + 1);
    double absNorm = sqrt(normals[idx3] * normals[idx3]) +
                     (normals[idx3 + 1] * normals[idx3 + 1]) +
                     (normals[idx3 + 2] * normals[idx3 + 2]);
    normals[idx3] /= absNorm;
    normals[idx3 + 1] /= absNorm;
    normals[idx3 + 2] /= absNorm;

    colors[idx3] = (weight * colors[idx3] + mc0) / (weight + 1);
    colors[idx3 + 1] = (weight * colors[idx3 + 1] + mc1) / (weight + 1);
    colors[idx3 + 2] = (weight * colors[idx3 + 2] + mc2) / (weight + 1);

    weights[i] += 1.0;

    mergeBoard[mapIdx] = true;
    atomicAdd(d_mergedNodes, 1);
}

void Map::fuse(Frame *f, double **intrinsicMatrix, double **T) {
    int h = f->getHeight(), w = f->getWidth();
    auto vertexMap = f->getVertexMap();
    auto normalMap = f->getNormalMap();
    auto colorMap = f->getColorMap();

    const int newXSize = h * w;
    double **nPoints = new double *[newXSize];
    double **nNormals = new double *[newXSize];
    double **nColors = new double *[newXSize];
    for (int i = 0; i < newXSize; i++) {
        nPoints[i] = new double[3];
        nNormals[i] = new double[3];
        nColors[i] = new double[3];
    }

    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            for (int k = 0; k < 3; k++) {
                int newRow = i * w + j;
                nPoints[newRow][k] = vertexMap[i][j][k];
                nNormals[newRow][k] = normalMap[i][j][k];
                nColors[newRow][k] = colorMap[i][j][k];
            }
        }
    }

    if (!this->initialized) {
        this->add(h * w, nPoints, nNormals, nColors, T, NULL, h, w);
    } else {
        int initialNodeCount = this->numPoints;

        double **invT = new double *[4];
        for (int i = 0; i < 4; i++)
            invT[i] = new double[4];
        invert4x4Matrix<double>(T, invT);

        double fx = intrinsicMatrix[0][0];
        double fy = intrinsicMatrix[1][1];
        double cx = intrinsicMatrix[0][2];
        double cy = intrinsicMatrix[1][2];

        // A h * w board to keep track of merged/new points
        bool **mergeBoard = new bool *[h];
        for (int i = 0; i < h; i++) {
            mergeBoard[i] = new bool[w]();
        }

        int mergedNodes = 0;

        double *d_points, *d_normals, *d_colors, *d_weights, *d_vertexMap,
            *d_normalMap, *d_colorMap, *d_invT, *d_T;
        bool *d_mergeBoard;
        int *d_mergedNodes;

        cudaMalloc(&d_points, initialNodeCount * 3 * sizeof(double));
        cudaMalloc(&d_normals, initialNodeCount * 3 * sizeof(double));
        cudaMalloc(&d_colors, initialNodeCount * 3 * sizeof(double));
        cudaMalloc(&d_weights, initialNodeCount * sizeof(double));
        cudaMalloc(&d_vertexMap, h * w * 3 * sizeof(double));
        cudaMalloc(&d_normalMap, h * w * 3 * sizeof(double));
        cudaMalloc(&d_colorMap, h * w * 3 * sizeof(double));
        cudaMalloc(&d_mergeBoard, h * w * sizeof(bool));
        cudaMalloc(&d_invT, 4 * 4 * sizeof(double));
        cudaMalloc(&d_T, 4 * 4 * sizeof(double));
        cudaMalloc(&d_mergedNodes, sizeof(int));

        cudaMemcpy(d_points, points, initialNodeCount * 3 * sizeof(double),
                   cudaMemcpyHostToDevice);
        cudaMemcpy(d_normals, normals, initialNodeCount * 3 * sizeof(double),
                   cudaMemcpyHostToDevice);
        cudaMemcpy(d_colors, colors, initialNodeCount * 3 * sizeof(double),
                   cudaMemcpyHostToDevice);
        cudaMemcpy(d_weights, weights, initialNodeCount * sizeof(double),
                   cudaMemcpyHostToDevice);
        cudaMemcpy(d_vertexMap, vertexMap, h * w * 3 * sizeof(double),
                   cudaMemcpyHostToDevice);
        cudaMemcpy(d_normalMap, normalMap, h * w * 3 * sizeof(double),
                   cudaMemcpyHostToDevice);
        cudaMemcpy(d_colorMap, colorMap, h * w * 3 * sizeof(double),
                   cudaMemcpyHostToDevice);
        cudaMemcpy(d_mergeBoard, mergeBoard, h * w * sizeof(bool),
                   cudaMemcpyHostToDevice);
        cudaMemcpy(d_invT, invT, 4 * 4 * sizeof(double),
                   cudaMemcpyHostToDevice);
        cudaMemcpy(d_T, T, 4 * 4 * sizeof(double), cudaMemcpyHostToDevice);
        cudaMemcpy(d_mergedNodes, &mergedNodes, sizeof(int),
                   cudaMemcpyHostToDevice);

        int numBlocks = (initialNodeCount + 256 - 1) / 256;

        mergeKernel<<<numBlocks, 256>>>(
            d_points, d_normals, d_colors, d_weights, d_vertexMap, d_normalMap,
            d_colorMap, d_mergeBoard, d_invT, d_T, initialNodeCount, fx, fy, cx,
            cy, h, w, this->distDiff, this->angleDiff, d_mergedNodes);

        cudaMemcpy(this->points, d_points,
                   initialNodeCount * 3 * sizeof(double),
                   cudaMemcpyDeviceToHost);
        cudaMemcpy(this->normals, d_normals,
                   initialNodeCount * 3 * sizeof(double),
                   cudaMemcpyDeviceToHost);
        cudaMemcpy(this->colors, d_colors,
                   initialNodeCount * 3 * sizeof(double),
                   cudaMemcpyDeviceToHost);
        cudaMemcpy(this->weights, d_weights, initialNodeCount * sizeof(double),
                   cudaMemcpyDeviceToHost);
        cudaMemcpy(&mergedNodes, d_mergedNodes, sizeof(int),
                   cudaMemcpyDeviceToHost);

        cudaFree(d_points);
        cudaFree(d_normals);
        cudaFree(d_colors);
        cudaFree(d_weights);
        cudaFree(d_vertexMap);
        cudaFree(d_normalMap);
        cudaFree(d_colorMap);
        cudaFree(d_mergeBoard);
        cudaFree(d_invT);
        cudaFree(d_T);
        cudaFree(d_mergedNodes);

        this->add(h * w - mergedNodes, nPoints, nNormals, nColors, T,
                  mergeBoard, h, w);

        delete[] invT[0];
        delete[] invT[1];
        delete[] invT[2];
        delete[] invT[3];

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

// Note: GPU acceleration for this function may not yield significant benefits
// in most scenarios. In my evaluations, `num` tends to be small after the
// initial call unless there are substantial differences between consecutive
// images. However, if you observe different behavior, consider
// GPU-acceleration the way I do for `fuse()`
void Map::add(int num, double **nPoints, double **nNormals, double **nColors,
              double **T, bool **mergeBoard, int h, int w) {
    if (num == 0) return;

    if (!this->initialized) {
        assert(num <= MAX_NODES);

        double **tInitPoints = new double *[num];
        double **rInitNorms = new double *[num];

        for (int i = 0; i < num; i++) {
            this->points[i][0] = (T[0][0] * nPoints[i][0]) +
                                 (T[0][1] * nPoints[i][1]) +
                                 (T[0][2] * nPoints[i][2]) + T[0][3];
            this->points[i][1] = (T[1][0] * nPoints[i][0]) +
                                 (T[1][1] * nPoints[i][1]) +
                                 (T[1][2] * nPoints[i][2]) + T[1][3];
            this->points[i][2] = (T[2][0] * nPoints[i][0]) +
                                 (T[2][1] * nPoints[i][1]) +
                                 (T[2][2] * nPoints[i][2]) + T[2][3];

            this->normals[i][0] = (T[0][0] * nNormals[i][0]) +
                                  (T[0][1] * nNormals[i][1]) +
                                  (T[0][2] * nNormals[i][2]);
            this->normals[i][1] = (T[1][0] * nNormals[i][0]) +
                                  (T[1][1] * nNormals[i][1]) +
                                  (T[1][2] * nNormals[i][2]);
            this->normals[i][2] = (T[2][0] * nNormals[i][0]) +
                                  (T[2][1] * nNormals[i][1]) +
                                  (T[2][2] * nNormals[i][2]);

            std::copy(nColors[i], nColors[i] + 3, this->colors[i]);
        }

        std::fill(this->weights, this->weights + num, 1.0);

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
        for (int j = 0; j < w; j++) {
            if (!mergeBoard[i][j]) {
                int idx = i * w + j;

                this->points[this->numPoints] = new double[3];
                this->points[this->numPoints][0] =
                    (T[0][0] * nPoints[idx][0]) + (T[0][1] * nPoints[idx][1]) +
                    (T[0][2] * nPoints[idx][2]) + T[0][3];
                this->points[this->numPoints][1] =
                    (T[1][0] * nPoints[idx][0]) + (T[1][1] * nPoints[idx][1]) +
                    (T[1][2] * nPoints[idx][2]) + T[1][3];
                this->points[this->numPoints][2] =
                    (T[2][0] * nPoints[idx][0]) + (T[2][1] * nPoints[idx][1]) +
                    (T[2][2] * nPoints[idx][2]) + T[2][3];

                this->normals[this->numPoints] = new double[3];
                this->normals[this->numPoints][0] =
                    (T[0][0] * nNormals[idx][0]) +
                    (T[0][1] * nNormals[idx][1]) + (T[0][2] * nNormals[idx][2]);
                this->normals[this->numPoints][1] =
                    (T[1][0] * nNormals[idx][0]) +
                    (T[1][1] * nNormals[idx][1]) + (T[1][2] * nNormals[idx][2]);
                this->normals[this->numPoints][2] =
                    (T[2][0] * nNormals[idx][0]) +
                    (T[2][1] * nNormals[idx][1]) + (T[2][2] * nNormals[idx][2]);

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
