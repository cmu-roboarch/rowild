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

#include <cmath>
#include <iomanip>

template <typename T> T degToRad(T angle) {
    return angle * (3.141592654 / 180);
}

template <typename T>
static std::string matrixToString(T **A, int rows, int cols) {
    std::string str = "<";

    for (int i = 0; i < rows; i++) {
        str.append("(");
        for (int j = 0; j < cols; j++) {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(8) << A[i][j];
            str += ss.str() + ", ";
        }
        str.pop_back();
        str.pop_back();
        str.append("), ");
    }
    str.pop_back();
    str.pop_back();
    str.append(">");

    return str;
}

template <typename T> T **reshape3dTo2d(T ***A, int x, int y, int z) {
    int newX = x * y;
    int newY = z;

    T **newMatrix = new T *[newX];
    for (int i = 0; i < newX; i++) {
        newMatrix[i] = new T[newY];
    }

    for (int i = 0; i < x; i++) {
        for (int j = 0; j < y; j++) {
            for (int k = 0; k < z; k++) {
                int newRow = i * y + j;
                int newCol = k;
                newMatrix[newRow][newCol] = A[i][j][k];
            }
        }
    }

    return newMatrix;
}

template <typename T> void matrixInverse(T **A, double **inv, int n) {
    auto getMinor = [](double **src, double **dest, int row, int col, int n) {
        int rowCount = 0;
        for (int i = 0; i < n; i++) {
            if (i != row) {
                int colCount = 0;
                for (int j = 0; j < n; j++) {
                    if (j != col) {
                        dest[rowCount][colCount] = src[i][j];
                        colCount++;
                    }
                }
                rowCount++;
            }
        }
    };

    const auto calcDeterminant = [&getMinor](double **mat, int n) -> double {
        auto detImpl = [&getMinor](double **mat, int n, auto &detRef) mutable {
            if (n == 1) return mat[0][0];
            double det = 0;

            double **minor = new double *[n - 1];
            for (int i = 0; i < n - 1; i++) {
                minor[i] = new double[n - 1];
            }

            for (int i = 0; i < n; i++) {
                getMinor(mat, minor, 0, i, n);
                det += (i % 2 == 1 ? -1.0 : 1.0) * mat[0][i] *
                       detRef(minor, n - 1, detRef);
            }

            for (int i = 0; i < n - 1; i++) {
                delete[] minor[i];
            }

            delete[] minor;
            return det;
        };

        return detImpl(mat, n, detImpl);
    };

    double det = 1.0 / calcDeterminant(A, n);

    double *temp = new double[(n - 1) * (n - 1)];
    double **minor = new double *[n - 1];
    for (int i = 0; i < n - 1; i++) {
        minor[i] = temp + (i * (n - 1));
    }

    for (int j = 0; j < n; j++) {
        for (int i = 0; i < n; i++) {
            getMinor(A, minor, j, i, n);
            inv[i][j] = det * calcDeterminant(minor, n - 1);
            if ((i + j) % 2 == 1) {
                inv[i][j] = -inv[i][j];
            }
        }
    }

    delete[] temp;
    delete[] minor;
}

template <typename T, size_t n> void matrixInverse(T **A, double **inv) {
    auto getMinor = [](double **src, double **dest, int row, int col, int nn) {
        int rowCount = 0;
        for (int i = 0; i < nn; i++) {
            if (i != row) {
                int colCount = 0;
                for (int j = 0; j < nn; j++) {
                    if (j != col) {
                        dest[rowCount][colCount] = src[i][j];
                        colCount++;
                    }
                }
                rowCount++;
            }
        }
    };

    const auto calcDeterminant = [&getMinor](double **mat, int nn) -> double {
        auto detImpl = [&getMinor](double **mat, int nn, auto &detRef) mutable {
            if (nn == 1) return mat[0][0];
            double det = 0;

            double **minor = new double *[nn - 1];
            for (int i = 0; i < nn - 1; i++) {
                minor[i] = new double[nn - 1];
            }

            for (int i = 0; i < nn; i++) {
                getMinor(mat, minor, 0, i, nn);
                det += (i % 2 == 1 ? -1.0 : 1.0) * mat[0][i] *
                       detRef(minor, nn - 1, detRef);
            }

            for (int i = 0; i < nn - 1; i++) {
                delete[] minor[i];
            }

            delete[] minor;
            return det;
        };

        return detImpl(mat, nn, detImpl);
    };

    double det = 1.0 / calcDeterminant(A, n);

    double *temp = new double[(n - 1) * (n - 1)];
    double **minor = new double *[n - 1];
    for (size_t i = 0; i < n - 1; i++) {
        minor[i] = temp + (i * (n - 1));
    }

    for (size_t j = 0; j < n; j++) {
        for (size_t i = 0; i < n; i++) {
            getMinor(A, minor, j, i, n);
            inv[i][j] = det * calcDeterminant(minor, n - 1);
            if ((i + j) % 2 == 1) {
                inv[i][j] = -inv[i][j];
            }
        }
    }

    delete[] temp;
    delete[] minor;
}

template <typename T> void matrixTranspose(T **A, T **B, int row, int col) {
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            B[j][i] = A[i][j];
        }
    }
}

template <typename T, size_t row, size_t col>
void matrixTranspose(T (&A)[row][col], T (&B)[col][row]) {
    for (size_t i = 0; i < row; i++) {
        for (size_t j = 0; j < col; j++) {
            B[j][i] = A[i][j];
        }
    }
}

template <typename T>
void matrixMultiplication(T **A, T **B, T **res, int rowA, int rowB, int colB) {
    assert(A != res && B != res);
    for (int i = 0; i < rowA; i++) {
        for (int j = 0; j < colB; j++) {
            res[i][j] = 0;
            for (int k = 0; k < rowB; k++) {
                res[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

template <typename T, size_t rowA, size_t rowB, size_t colB>
void matrixMultiplication(T **A, T **B, T **res) {
    assert(A != res && B != res);
    for (size_t i = 0; i < rowA; i++) {
        for (size_t j = 0; j < colB; j++) {
            res[i][j] = 0;
            for (size_t k = 0; k < rowB; k++) {
                res[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

template <typename T, size_t rowA, size_t colA, size_t rowB, size_t colB>
void matrixMultiplication(T (&A)[rowA][colA], T (&B)[rowB][colB],
                          T (&res)[rowA][colB]) {
    for (size_t i = 0; i < rowA; i++) {
        for (size_t j = 0; j < colB; j++) {
            res[i][j] = 0;
            for (size_t k = 0; k < rowB; k++) {
                res[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

template <typename T, size_t rowA, size_t colA, size_t rowB, size_t colB>
void matrixMultiplication(T (&A)[rowA][colA], T **B, T (&res)[rowA][colB]) {
    for (size_t i = 0; i < rowA; i++) {
        for (size_t j = 0; j < colB; j++) {
            res[i][j] = 0;
            for (size_t k = 0; k < rowB; k++) {
                res[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

template <typename T, size_t rowA, size_t colA, size_t rowB, size_t colB>
void matrixMultiplication(T **A, T (&B)[rowB][colB], T (&res)[rowA][colB]) {
    for (size_t i = 0; i < rowA; i++) {
        for (size_t j = 0; j < colB; j++) {
            res[i][j] = 0;
            for (size_t k = 0; k < rowB; k++) {
                res[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

template <typename T, size_t rowA, size_t colA, size_t rowB, size_t colB>
void matrixMultiplication(T (&A)[rowA][colA], T (&B)[rowB][colB], T **res) {
    for (size_t i = 0; i < rowA; i++) {
        for (size_t j = 0; j < colB; j++) {
            res[i][j] = 0;
            for (size_t k = 0; k < rowB; k++) {
                res[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}
