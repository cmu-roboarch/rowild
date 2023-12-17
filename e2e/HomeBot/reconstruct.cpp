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

#include "reconstruct.h"
#include "transforms.h"
#include "utils.h"
#include <algorithm>
#include <vector>

namespace reconstruct {

void buildLinearSystem(double **srcPoints, double **tarPoints,
                       double **tarNormals, int num, double **T, double **A,
                       double *b) {
    double **sPointsPrime = transforms::transform(srcPoints, num, T);
    for (int i = 0; i < num; i++) {
        double *theSrcPoint = sPointsPrime[i];
        double *theTarPoint = tarPoints[i];
        double *theTarNormal = tarNormals[i];

        A[i][0] =
            theSrcPoint[1] * theTarNormal[2] - theSrcPoint[2] * theTarNormal[1];
        A[i][1] =
            theSrcPoint[2] * theTarNormal[0] - theSrcPoint[0] * theTarNormal[2];
        A[i][2] =
            theSrcPoint[0] * theTarNormal[1] - theSrcPoint[1] * theTarNormal[0];
        A[i][3] = theTarNormal[0];
        A[i][4] = theTarNormal[1];
        A[i][5] = theTarNormal[2];

        b[i] = (theTarNormal[0] * (theTarPoint[0] - theSrcPoint[0])) +
               (theTarNormal[1] * (theTarPoint[1] - theSrcPoint[1])) +
               (theTarNormal[2] * (theTarPoint[2] - theSrcPoint[2]));
    }

    for (int i = 0; i < num; i++)
        delete[] sPointsPrime[i];
    delete[] sPointsPrime;
}

double *solveLinearSystem(double **A /* num x 6 */, double *b, int num) {
    constexpr const int vars = 3 + 3;

    double **transposedA = new double *[vars]; // 6 x num
    for (int i = 0; i < vars; i++) {
        transposedA[i] = new double[num];
    }
    matrixTranspose(A, transposedA, num, vars);

    double **APrime = new double *[vars]; // 6 x 6
    for (int i = 0; i < vars; i++) {
        APrime[i] = new double[vars];
    }

    matrixMultiplication(transposedA, A, APrime, 3 + 3, num, 3 + 3);

    double *bPrime = new double[vars](); // 1 x 6

    for (int i = 0; i < vars; i++) {
        for (int j = 0; j < num; j++) {
            bPrime[i] += transposedA[i][j] * b[j];
        }
    }

    double **inverseAPrime = new double *[vars];
    for (int i = 0; i < vars; i++) {
        inverseAPrime[i] = new double[vars];
    }
    matrixInverse<double, vars>(APrime, inverseAPrime);

    double *x = new double[vars]();
    for (int i = 0; i < vars; i++) {
        for (int j = 0; j < vars; j++) {
            x[i] += inverseAPrime[i][j] * bPrime[j];
        }
    }

    for (int i = 0; i < vars; i++) {
        delete[] transposedA[i];
        delete[] APrime[i];
        delete[] inverseAPrime[i];
    }

    delete[] APrime;
    delete[] inverseAPrime;
    delete[] bPrime;
    delete[] transposedA;

    return x;
}

double **poseToTransformation(double *delta) {
    double *w = delta;
    double *u = delta + 3;

    double **T = new double *[4];
    for (int i = 0; i < 4; i++) {
        T[i] = new double[4];
    }

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

    T[3][0] = T[3][1] = T[3][2] = 0.0;
    T[3][3] = 1.0;

    return T;
}

std::vector<int> findProjCorr(Map *m, Frame *f, double **intrinsic, double **T,
                              int *newNum) {
    int h = f->getHeight();
    int w = f->getWidth();

    int numPoints = m->getNumPoints();
    double **tPoints = transforms::transform(m->getPoints(), numPoints, T);

    double *uvd = transforms::project(tPoints, numPoints, intrinsic);
    double *u = uvd;
    double *v = uvd + numPoints;
    double *d = uvd + 2 * numPoints;

    auto fastNorm = [](double *s, double *t) {
        double dist = (s[0] - t[0]) * (s[0] - t[0]);
        dist += (s[1] - t[1]) * (s[1] - t[1]);
        dist += (s[2] - t[2]) * (s[2] - t[2]);
        return sqrt(dist);
    };

    // For the sake performance I pack <index, targetU, targetV> tuples
    // in an "interleaved" manner. This effectively eliminates multiple
    // further loops that would otherwise be needed
    std::vector<int> iuv;

    for (int i = 0; i < numPoints; i++) {
        int tU = round(u[i]);
        int tV = round(v[i]);
        double tD = d[i];

        if ((tV >= 0) && (tV < h) && (tU >= 0) && (tU < w) && (tD >= 0)) {
            double *s = tPoints[i];
            double *t = f->getVertexMap()[tV][tU];
            double dist = fastNorm(s, t);

            if (dist < 0.07 /*Distance threshold*/) {
                iuv.push_back(i);
                iuv.push_back(tU);
                iuv.push_back(tV);
            }
        }
    }

    delete[] uvd;
    for (int i = 0; i < numPoints; i++)
        delete[] tPoints[i];
    delete[] tPoints;

    *newNum = static_cast<int>(iuv.size()) / 3;

    return iuv;
}

double **icp(Map *m, Frame *f, double **intrinsic, double **T) {
    for (int i = 0; i < 10; i++) {
        int newNumPoints;
        auto iuv = findProjCorr(m, f, intrinsic, T, &newNumPoints);
        if (newNumPoints == 0) break;

        double **corrSrcPoints = new double *[newNumPoints];
        double **corrTarPoints = new double *[newNumPoints];
        double **corrTarNormals = new double *[newNumPoints];
        for (int i = 0; i < newNumPoints; i++) {
            corrSrcPoints[i] = new double[3];
            corrTarPoints[i] = new double[3];
            corrTarNormals[i] = new double[3];
        }

#pragma omp parallel for num_threads(8)
        for (int i = 0; i < newNumPoints; i++) {
            int idx = iuv[3 * i];
            int tU = iuv[3 * i + 1];
            int tV = iuv[3 * i + 2];

            double *theSrcPoint = m->getPoints()[idx];
            double *theTarPoint = f->getVertexMap()[tV][tU];
            double *theTarNormal = f->getNormalMap()[tV][tU];

            for (int j = 0; j < 3; j++) {
                corrSrcPoints[i][j] = theSrcPoint[j];
                corrTarPoints[i][j] = theTarPoint[j];
                corrTarNormals[i][j] = theTarNormal[j];
            }
        }

        double **A = new double *[newNumPoints];
        double *b = new double[newNumPoints];
        for (int i = 0; i < newNumPoints; i++) {
            A[i] = new double[3 + 3];
        }

        buildLinearSystem(corrSrcPoints, corrTarPoints, corrTarNormals,
                          newNumPoints, T, A, b);

        double *delta = solveLinearSystem(A, b, newNumPoints);

        double **update = poseToTransformation(delta);

        double **oldT = new double *[4];
        for (int i = 0; i < 4; i++) {
            oldT[i] = new double[4]{T[i][0], T[i][1], T[i][2], T[i][3]};
        }
        matrixMultiplication<double, 4, 4, 4>(update, oldT, T);

        for (int i = 0; i < newNumPoints; i++) {
            delete[] corrSrcPoints[i];
            delete[] corrTarPoints[i];
            delete[] corrTarNormals[i];
            delete[] A[i];
        }
        delete[] oldT[0];
        delete[] oldT[1];
        delete[] oldT[2];
        delete[] oldT[3];

        delete[] corrSrcPoints;
        delete[] corrTarPoints;
        delete[] corrTarNormals;
        delete[] A;
        delete[] b;
        delete[] oldT;
    }

    return T;
}

} // namespace reconstruct
