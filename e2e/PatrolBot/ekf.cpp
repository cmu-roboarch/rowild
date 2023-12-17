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

#include "ekf.h"

EKFSLAM::EKFSLAM(int numLandmarks, double sX2, double sY2, double sA2,
                 double sB2, double sR2)
    : SLAM(numLandmarks), sigX2(sX2), sigY2(sY2), sigAlpha2(sA2), sigBeta2(sB2),
      sigR2(sR2) {

    int dim = 3 + 2 * numLandmarks;

    mu.resize(dim, 0.0);
    Sigma = createMatrix(dim, dim);
    for (int i = 0; i < 3; i++) {
        Sigma[i][i] = 0.001; // Initial uncertainty for robot is low
    }
    for (int i = 3; i < dim; i++) {
        Sigma[i][i] = 1000.0; // Initial uncertainty for landmarks is high
    }
}

void EKFSLAM::motionUpdate(double dx, double dy, double dtheta) {
    mu[0] += dx;
    mu[1] += dy;
    mu[2] += dtheta;

    std::vector<std::vector<double>> G =
        createMatrix(mu.size(), mu.size(), 1.0);

    std::vector<std::vector<double>> R = createMatrix(3, 3);
    R[0][0] = sigX2;
    R[1][1] = sigY2;
    R[2][2] = sigAlpha2;

    auto tempSigma = matMul(matMul(G, Sigma), transpose(G));
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Sigma[i][j] += R[i][j];
        }
    }
}

void EKFSLAM::measurementUpdate(const std::vector<double> &measurements) {
    for (size_t i = 0; i < measurements.size() / 2 && i < 6; i++) {
        std::vector<std::vector<double>> Q = createMatrix(2, 2);
        Q[0][0] = sigR2;
        Q[1][1] = sigBeta2;

        int landmarkIndex = 3 + 2 * i;

        double dx = measurements[2 * i] - mu[landmarkIndex];
        double dy = measurements[2 * i + 1] - mu[landmarkIndex + 1];
        double q = dx * dx + dy * dy;

        std::vector<std::vector<double>> H = createMatrix(2, mu.size());
        H[0][0] = -dx / sqrt(q);
        H[0][1] = -dy / sqrt(q);
        H[0][landmarkIndex] = dx / sqrt(q);
        H[0][landmarkIndex + 1] = dy / sqrt(q);
        H[1][0] = dy / q;
        H[1][1] = -dx / q;
        H[1][landmarkIndex] = -dy / q;
        H[1][landmarkIndex + 1] = dx / q;

        auto S = matAdd(matMul(matMul(H, Sigma), transpose(H)), Q);
        auto K = matMul(matMul(Sigma, transpose(H)), inverse(S));

        std::vector<double> z_diff = {measurements[2 * i],
                                      measurements[2 * i + 1]};
        auto mu_diff = matMul(K, z_diff);
        for (size_t j = 0; j < mu_diff.size(); j++) {
            mu[j] += mu_diff[j];
        }

        Sigma = matSub(Sigma, matMul(matMul(K, H), Sigma));
    }
}

std::vector<double> EKFSLAM::getStatus() const { return mu; }

std::vector<std::vector<double>> EKFSLAM::createMatrix(int rows, int cols,
                                                       double initVal) {
    return std::vector<std::vector<double>>(rows,
                                            std::vector<double>(cols, initVal));
}

std::vector<std::vector<double>>
EKFSLAM::transpose(const std::vector<std::vector<double>> &mat) {
    int rows = mat.size();
    int cols = mat[0].size();
    std::vector<std::vector<double>> trans(cols, std::vector<double>(rows));
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            trans[j][i] = mat[i][j];
    return trans;
}

std::vector<std::vector<double>>
EKFSLAM::matMul(const std::vector<std::vector<double>> &mat1,
                const std::vector<std::vector<double>> &mat2) {
    int rows = mat1.size();
    int cols = mat2[0].size();
    int inner = mat1[0].size();
    if (inner != static_cast<int>(mat2.size())) {
        throw std::runtime_error(
            "Matrix dimensions do not match for multiplication.");
    }

    std::vector<std::vector<double>> result(rows, std::vector<double>(cols, 0));
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            for (int k = 0; k < inner; k++) {
                result[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
    return result;
}

std::vector<double> EKFSLAM::matMul(const std::vector<std::vector<double>> &mat,
                                    const std::vector<double> &vec) {
    int rows = mat.size();
    int inner = mat[0].size();
    if (inner != static_cast<int>(vec.size())) {
        throw std::runtime_error(
            "Matrix and vector dimensions do not match for multiplication.");
    }

    std::vector<double> result(rows, 0);
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < inner; j++) {
            result[i] += mat[i][j] * vec[j];
        }
    }
    return result;
}

std::vector<std::vector<double>>
EKFSLAM::matAdd(const std::vector<std::vector<double>> &A,
                const std::vector<std::vector<double>> &B) {
    int rows = A.size(), cols = A[0].size();
    std::vector<std::vector<double>> result(rows, std::vector<double>(cols));
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            result[i][j] = A[i][j] + B[i][j];
    return result;
}

std::vector<std::vector<double>>
EKFSLAM::matSub(const std::vector<std::vector<double>> &A,
                const std::vector<std::vector<double>> &B) {
    int rows = A.size(), cols = A[0].size();
    std::vector<std::vector<double>> result(rows, std::vector<double>(cols));
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            result[i][j] = A[i][j] - B[i][j];
    return result;
}

std::vector<std::vector<double>>
EKFSLAM::inverse(const std::vector<std::vector<double>> &mat) {
    int n = mat.size();
    std::vector<std::vector<double>> augmentedMatrix(
        n, std::vector<double>(2 * n));

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            augmentedMatrix[i][j] = mat[i][j];
            if (i == j) {
                augmentedMatrix[i][j + n] = 1;
            }
        }
    }

    // Gauss-Jordan elimination
    for (int i = 0; i < n; i++) {
        if (augmentedMatrix[i][i] == 0) {
            int swapRow = -1;
            for (int k = i + 1; k < n; k++) {
                if (augmentedMatrix[k][i] != 0) {
                    swapRow = k;
                    break;
                }
            }
            if (swapRow == -1) {
                throw std::runtime_error(
                    "Matrix is singular and cannot be inverted.");
            }
            std::swap(augmentedMatrix[i], augmentedMatrix[swapRow]);
        }

        double diagonalValue = augmentedMatrix[i][i];
        for (int j = 0; j < 2 * n; j++) {
            augmentedMatrix[i][j] /= diagonalValue;
        }

        for (int k = 0; k < n; k++) {
            if (k != i) {
                double factor = augmentedMatrix[k][i];
                for (int j = 0; j < 2 * n; j++) {
                    augmentedMatrix[k][j] -= factor * augmentedMatrix[i][j];
                }
            }
        }
    }

    std::vector<std::vector<double>> inverseMatrix(n, std::vector<double>(n));
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            inverseMatrix[i][j] = augmentedMatrix[i][j + n];
        }
    }

    return inverseMatrix;
}
