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
#include <vector>
#include "frame.h"
#include "map.h"
#include "utils.h"
#include "transforms.h"

static void buildLinearSystem(double (&srcPoints)[MAX_NODES][3], double (&tarPoints)[MAX_NODES][3], double (&tarNormals)[MAX_NODES][3], int num, double (&T)[4][4], double (&A)[MAX_NODES][6], double (&b)[MAX_NODES]) {
    double sPointsPrime[MAX_NODES][3];
    transform(srcPoints, num, T, sPointsPrime);
    for (int i = 0; i < num; i++) {
        double *theSrcPoint = sPointsPrime[i];
        double *theTarPoint = tarPoints[i];
        double *theTarNormal = tarNormals[i];

        A[i][0] = theSrcPoint[1] * theTarNormal[2] - theSrcPoint[2] * theTarNormal[1];
        A[i][1] = theSrcPoint[2] * theTarNormal[0] - theSrcPoint[0] * theTarNormal[2];
        A[i][2] = theSrcPoint[0] * theTarNormal[1] - theSrcPoint[1] * theTarNormal[0];
        A[i][3] = theTarNormal[0];
        A[i][4] = theTarNormal[1];
        A[i][5] = theTarNormal[2];

        b[i] = (theTarNormal[0] * (theTarPoint[0] - theSrcPoint[0])) + (theTarNormal[1] * (theTarPoint[1] - theSrcPoint[1])) + (theTarNormal[2] * (theTarPoint[2] - theSrcPoint[2]));
    }
}


static void solveLinearSystem(double (&A)[MAX_NODES][6], double (&b)[MAX_NODES], int num, double (&X)[MAX_NODES]) {
    constexpr const int vars = 3 + 3;

    double transposedA[6][MAX_NODES];
    matrixTranspose<double, MAX_NODES, 6>(A, transposedA);

    double APrime[6][6];
    matrixMultiplication<double, 6, MAX_NODES, MAX_NODES, 6>(transposedA, A, APrime);

    double bPrime[6];
    for (int i = 0; i < vars; i++) {
        for (int j = 0; j < num; j++) {
            bPrime[i] += transposedA[i][j] * b[j];
        }
    }

    double inverseAPrime[6][6];
    // invertMatrix<6>(APrime, inverseAPrime);

    for (int i = 0; i < vars; i++) {
        X[i] = 0;
        for (int j = 0; j < vars; j++) {
            X[i] += inverseAPrime[i][j] * bPrime[j];
        }
    }
}


static void poseToTransformation(double (&delta)[MAX_NODES], double (&T)[4][4]) {
    double *w = delta;
    double *u = delta + 3;

    T[0][0] = cos(w[2]) * cos(w[1]);
    T[0][1] = -sin(w[2]) * cos(w[0]) + cos(w[2]) * sin(w[1]) * sin(w[0]);
    T[0][2] = sin(w[2]) * sin(w[0]) + cos(w[2]) * sin(w[1]) * cos(w[1]);
    T[1][0] = sin(w[2]) * cos(w[1]);
    T[1][1] = cos(w[2]) * cos(w[0]) + sin(w[2]) * sin(w[1]) * sin(w[0]);
    T[1][2] = -cos(w[2]) * sin(w[0]) + sin(w[2]) * sin(w[1]) * cos(w[0]);
    T[2][0] = -sin(w[1]);
    T[2][1] = cos(w[1]) * sin(w[0]);
    T[2][2] = cos(w[1]) * cos(w[0]);

    T[0][3] = u[0];
    T[1][3] = u[1];
    T[2][3] = u[2];

    T[3][0] = T[3][1] = T[3][2] =  0.0;
    T[3][3] = 1.0;
}


static int findProjCorr(Map m, Frame f, double (&intrinsic)[3][3], double (&T)[4][4], double (&iuv)[MAX_NODES * 3]) {
    int h = f.getHeight();
    int w = f.getWidth();

    int numPoints = m.getNumPoints();
    double tPoints[MAX_NODES][3];
    // transform(m.getPoints(), numPoints, T, tPoints);

    double uvd[3 * MAX_NODES];
    // project(tPoints, numPoints, intrinsic, uvd);
    double *u = uvd;
    double *v = uvd + numPoints;
    double *d = uvd + 2*numPoints;

    int iuvIdx = 0;

    for (int i = 0; i < numPoints; i++) {
        int tU = round(u[i]);
        int tV = round(v[i]);
        double tD = d[i];

        if ((tV >= 0) && (tV < h) && (tU >= 0) && (tU < w) && (tD >= 0)) {
            // double *s = tPoints[i];
            // double *t = f.getVertexMap()[tV][tU];
            double dist = 0; // sqrt(((s[0]-t[0]) * (s[0]-t[0])) + ((s[1]-t[1]) * (s[1]-t[1])) + ((s[2]-t[2]) * (s[2]-t[2])));

            if (dist < 0.07 /*Distance threshold*/) {
                iuv[iuvIdx++] = i;
                iuv[iuvIdx++] = tU;
                iuv[iuvIdx++] = tV;
            }
        }
    }

    return iuvIdx/3;
}


static void icp(Map m, Frame f, double (&intrinsic)[3][3], double (&T)[4][4]) {
    for (int i = 0; i < 10; i++) {
        int newNumPoints;
        double iuv[3*MAX_NODES];
        //newNumPoints = findProjCorr(m, f, intrinsic, T, iuv);
        if (newNumPoints == 0) break;

        double corrSrcPoints[MAX_NODES][3];
        double corrTarPoints[MAX_NODES][3];
        double corrTarNormals[MAX_NODES][3];

        double A[MAX_NODES][6];
        double b[MAX_NODES];

        buildLinearSystem(corrSrcPoints, corrTarPoints, corrTarNormals, newNumPoints, T, A, b);

        double delta[MAX_NODES];
        solveLinearSystem(A, b, newNumPoints, delta);

        double update[4][4];
        poseToTransformation(delta, update);

        double oldT[4][4];
        matrixMultiplication<double, 4, 4, 4>(update, oldT, T);
    }
}
