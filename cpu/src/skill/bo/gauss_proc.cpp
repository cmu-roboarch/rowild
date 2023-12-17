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

#include "gauss_proc.h"
#include "rowild_utils.h"
#include <utility>
#include <vector>

GaussianProcess::~GaussianProcess() {
    // Clear data structures
    assert(this->xTrain.size() == this->yTrain.size());
    if (!this->xTrain.empty()) {
        for (int i = 0; i < static_cast<int>(xTrain.size()); i++) {
            delete[] this->kMatrix[i];
            delete[] this->invKMatrix[i];
        }

        delete[] this->kMatrix;
        delete[] this->invKMatrix;

        this->xTrain.clear();
        this->yTrain.clear();
    }
}

void GaussianProcess::train(const std::vector<PARAM> &x,
                            const std::vector<double> &y) {
    // Clear data structures
    assert(this->xTrain.size() == this->yTrain.size());
    if (!this->xTrain.empty()) {
        for (int i = 0; i < static_cast<int>(xTrain.size()); i++) {
            delete[] this->kMatrix[i];
            delete[] this->invKMatrix[i];
        }

        delete[] this->kMatrix;
        delete[] this->invKMatrix;

        this->xTrain.clear();
        this->yTrain.clear();
    }

    // Store training data
    assert(x.size() == y.size());
    this->xTrain = x;
    this->yTrain = y;

    int n = static_cast<int>(x.size());
    this->kMatrix = new double *[n];
    this->invKMatrix = new double *[n];
    for (int i = 0; i < n; i++) {
        this->kMatrix[i] = new double[n];
        this->invKMatrix[i] = new double[n];
    }

    // Compute K matrix
    for (int i = 0; i < static_cast<int>(x.size()); i++) {
        for (int j = 0; j < static_cast<int>(x.size()); j++) {
            PARAM *xI = &this->xTrain[i];
            PARAM *xJ = &this->xTrain[j];
            assert(xI->size() == xJ->size());
            assert(xI->size() == this->sigmaL.size());

            double k = 0;
            for (int p = 0; p < static_cast<int>(xI->size()); p++) {
                auto d = xI->at(p) - xJ->at(p);
                d *= this->sigmaL[p] * this->sigmaL[p];
                d *= xI->at(p) - xJ->at(p);
                k += d;
            }

            this->kMatrix[i][j] = this->sigmaA * this->sigmaA * exp(-0.5 * k);

            if (i == j) {
                this->kMatrix[i][j] += this->sigmaN * this->sigmaN;
            }
        }
    }

    matrixInverse<double>(this->kMatrix, this->invKMatrix, n);
}

std::pair<double, double> GaussianProcess::test(const PARAM &x) const {
    int l = static_cast<int>(this->xTrain.size());

    double **kVec = new double *[1];
    kVec[0] = new double[l]();

    for (int j = 0; j < l; j++) {
        const PARAM *xT = &this->xTrain[j];
        assert(xT->size() == x.size());
        assert(xT->size() == this->sigmaL.size());

        double k = 0;
        for (int p = 0; p < static_cast<int>(xT->size()); p++) {
            auto d = x[p] - xT->at(p);
            d *= this->sigmaL[p] * this->sigmaL[p];
            d *= x[p] - xT->at(p);
            k += d;
        }

        kVec[0][j] = this->sigmaA * this->sigmaA * exp(-0.5 * k);
    }

    double **m = new double *[1];
    m[0] = new double[l];
    matrixMultiplication(kVec, this->invKMatrix, m, 1, l, l);

    assert(l == static_cast<int>(this->yTrain.size()));
    double yMu = 0;
    double yStd = 0;
    for (int i = 0; i < l; i++) {
        yMu += m[0][i] * this->yTrain[i];
        yStd += m[0][i] * kVec[0][i];
    }

    delete[] kVec[0];
    delete[] kVec;
    delete[] m[0];
    delete[] m;

    yStd = sqrt(this->sigmaN + this->sigmaA - yStd);

    return std::make_pair(yMu, yStd);
}
