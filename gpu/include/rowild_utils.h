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
#include <thrust/functional.h>

#define PI (3.141592654)
#define TWO_PI (6.283185308)

template <typename T>
struct linearIdxToRowIdx : public thrust::unary_function<T, T> {
    T C; // number of columns

    __host__ __device__ explicit linearIdxToRowIdx(T C) : C(C) {}

    __host__ __device__ T operator()(T i) { return i / C; }
};

template <typename T> struct stdDevFunctor {
    const T m; // Mean
    __host__ __device__ explicit stdDevFunctor(T _m) : m(_m) {}

    __host__ __device__ T operator()(const T &d) const {
        return (d - m) * (d - m);
    }
};

template <typename T>
__host__ static std::string matrixToString(T **A, int rows, int cols) {
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

// The following two functions aren't intented to be called on large matrices.
// They are `inlined` in the CUDA kernel codes to operate on small matrices;
// otherwise, I would have parallelized them :)
template <typename T, int SIZE>
__device__ inline void copyMatrix(const T source[SIZE][SIZE],
                                  T destination[SIZE][SIZE]) {
    for (int row = 0; row < SIZE; row++) {
        for (int col = 0; col < SIZE; col++) {
            destination[row][col] = source[row][col];
        }
    }
}

template <typename T, size_t SIZE>
__device__ inline void sqMatMult(const T (&A)[SIZE][SIZE],
                                 const T (&B)[SIZE][SIZE], T (&C)[SIZE][SIZE]) {
    for (size_t i = 0; i < SIZE; i++) {
        for (size_t j = 0; j < SIZE; j++) {
            C[i][j] = 0;
            for (size_t k = 0; k < SIZE; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

template <typename T, size_t rowA, size_t colA, size_t rowB, size_t colB>
__host__ void cpuMatMul(T (&A)[rowA][colA], T (&B)[rowB][colB],
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

template <typename T>
__host__ __device__ inline void
blockDescToBoundingBox(T (&H)[4][4], T (&dim)[3], T (&corners)[9][3]) {
    T xL[3] = {static_cast<T>(H[0][0] * dim[0] / 2.0),
               static_cast<T>(H[1][0] * dim[0] / 2.0),
               static_cast<T>(H[2][0] * dim[0] / 2.0)};
    T yL[3] = {static_cast<T>(H[0][1] * dim[1] / 2.0),
               static_cast<T>(H[1][1] * dim[1] / 2.0),
               static_cast<T>(H[2][1] * dim[1] / 2.0)};
    T zL[3] = {static_cast<T>(H[0][2] * dim[2] / 2.0),
               static_cast<T>(H[1][2] * dim[2] / 2.0),
               static_cast<T>(H[2][2] * dim[2] / 2.0)};

    for (int i = 0; i < 3; i++)
        corners[0][i] = H[i][3];
    for (int i = 0; i < 3; i++)
        corners[1][i] = H[i][3] + xL[i] + yL[i] + zL[i];
    for (int i = 0; i < 3; i++)
        corners[2][i] = H[i][3] + xL[i] + yL[i] - zL[i];
    for (int i = 0; i < 3; i++)
        corners[3][i] = H[i][3] + xL[i] - yL[i] + zL[i];
    for (int i = 0; i < 3; i++)
        corners[4][i] = H[i][3] + xL[i] - yL[i] - zL[i];
    for (int i = 0; i < 3; i++)
        corners[5][i] = H[i][3] - xL[i] + yL[i] + zL[i];
    for (int i = 0; i < 3; i++)
        corners[6][i] = H[i][3] - xL[i] + yL[i] - zL[i];
    for (int i = 0; i < 3; i++)
        corners[7][i] = H[i][3] - xL[i] - yL[i] + zL[i];
    for (int i = 0; i < 3; i++)
        corners[8][i] = H[i][3] - xL[i] - yL[i] - zL[i];
}

template <typename T>
__host__ __device__ inline void rpyxyzToH(T roll, T pitch, T yaw, T x, T y, T z,
                                          T (&H)[4][4]) {
    T Ht[4][4] = {{1, 0, 0, x}, {0, 1, 0, y}, {0, 0, 1, z}, {0, 0, 0, 1}};

    T Hx[4][4] = {{1, 0, 0, 0},
                  {0, cosf(roll), -sinf(roll), 0},
                  {0, sinf(roll), cosf(roll), 0},
                  {0, 0, 0, 1}};

    T Hy[4][4] = {{cosf(pitch), 0, sinf(pitch), 0},
                  {0, 1, 0, 0},
                  {-sinf(pitch), 0, cosf(pitch), 0},
                  {0, 0, 0, 1}};

    T Hz[4][4] = {{cosf(yaw), -sinf(yaw), 0, 0},
                  {sinf(yaw), cosf(yaw), 0, 0},
                  {0, 0, 1, 0},
                  {0, 0, 0, 1}};

    T t1[4][4] = {};
    T t2[4][4] = {};

#ifdef __CUDA_ARCH__
    sqMatMult<float, 4>(Ht, Hz, t1);
    sqMatMult<float, 4>(t1, Hy, t2);
    sqMatMult<float, 4>(t2, Hx, H);
#else
    cpuMatMul<float, 4, 4, 4, 4>(Ht, Hz, t1);
    cpuMatMul<float, 4, 4, 4, 4>(t1, Hy, t2);
    cpuMatMul<float, 4, 4, 4, 4>(t2, Hx, H);
#endif
}

template <typename T>
__device__ bool pointsOverlap(T p1[9][3], T p2[9][3], T axis[3]) {

    T max1 = 0, max2 = 0, min1, min2;

    for (size_t j = 0; j < 3; j++) {
        max1 += p1[0][j] * axis[j];
        max2 += p2[0][j] * axis[j];
    }
    min1 = max1;
    min2 = max2;

    for (size_t i = 1; i < 9; i++) {
        T n1 = 0, n2 = 0;
        for (size_t j = 0; j < 3; j++) {
            n1 += p1[i][j] * axis[j];
            n2 += p2[i][j] * axis[j];
        }

        if (n1 > max1) max1 = n1;
        if (n1 < min1) min1 = n1;
        if (n2 > max2) max2 = n2;
        if (n2 < min2) min2 = n2;
    }

    if (max1 <= max2 && max1 >= min2) return true;
    if (min1 <= max2 && min1 >= min2) return true;
    if (max2 <= max1 && max2 >= min1) return true;
    if (min2 <= max1 && min2 >= min1) return true;

    return false;
}

template <typename T>
__device__ bool cuboidCuboidCollision(T p1[9][3], T axes1[3][3], T p2[9][3],
                                      T axes2[3][3]) {

    auto fastNorm = [] __device__(T x[3]) {
        return sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
    };

    auto diffNorm = [&fastNorm] __device__(T x[3], T y[3]) {
        T temp[3] = {x[0] - y[0], x[1] - y[1], x[2] - y[2]};
        return fastNorm(temp);
    };

    // Sphere check
    if (diffNorm(p1[0], p2[0]) >
        diffNorm(p1[0], p1[1]) + diffNorm(p2[0], p2[1])) {
        return false;
    }

    // Surface normal check
    for (int i = 0; i < 3; i++) {
        if (!pointsOverlap(p1, p2, axes1[i])) {
            return false;
        }
    }
    for (int i = 0; i < 3; i++) {
        if (!pointsOverlap(p1, p2, axes2[i])) {
            return false;
        }
    }

    // Edge-edge check
    auto crossMult = [] __device__(T A[3], T B[3], T C[3]) {
        C[0] = A[1] * B[2] - A[2] * B[1];
        C[1] = A[2] * B[0] - A[0] * B[2];
        C[2] = A[0] * B[1] - A[1] * B[0];
    };
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            T cm[3];
            crossMult(axes1[i], axes2[j], cm);
            if (!pointsOverlap(p1, p2, cm)) {
                return false;
            }
        }
    }

    return true;
}

template <typename T> __host__ T degToRad(T angle) {
    return angle * (PI / 180);
}

template <typename T> __host__ T **reshape3dTo2d(T ***A, int x, int y, int z) {
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

template <typename T> bool invert4x4Matrix(const T m[4][4], T invOut[4][4]) {
    T inv[4][4], det;

    inv[0][0] = m[1][1] * m[2][2] * m[3][3] - m[1][1] * m[2][3] * m[3][2] -
                m[2][1] * m[1][2] * m[3][3] + m[2][1] * m[1][3] * m[3][2] +
                m[3][1] * m[1][2] * m[2][3] - m[3][1] * m[1][3] * m[2][2];

    inv[1][0] = -m[1][0] * m[2][2] * m[3][3] + m[1][0] * m[2][3] * m[3][2] +
                m[2][0] * m[1][2] * m[3][3] - m[2][0] * m[1][3] * m[3][2] -
                m[3][0] * m[1][2] * m[2][3] + m[3][0] * m[1][3] * m[2][2];

    inv[2][0] = m[1][0] * m[2][1] * m[3][3] - m[1][0] * m[2][3] * m[3][1] -
                m[2][0] * m[1][1] * m[3][3] + m[2][0] * m[1][3] * m[3][1] +
                m[3][0] * m[1][1] * m[2][3] - m[3][0] * m[1][3] * m[2][1];

    inv[3][0] = -m[1][0] * m[2][1] * m[3][2] + m[1][0] * m[2][2] * m[3][1] +
                m[2][0] * m[1][1] * m[3][2] - m[2][0] * m[1][2] * m[3][1] -
                m[3][0] * m[1][1] * m[2][2] + m[3][0] * m[1][2] * m[2][1];

    inv[0][1] = -m[0][1] * m[2][2] * m[3][3] + m[0][1] * m[2][3] * m[3][2] +
                m[2][1] * m[0][2] * m[3][3] - m[2][1] * m[0][3] * m[3][2] -
                m[3][1] * m[0][2] * m[2][3] + m[3][1] * m[0][3] * m[2][2];

    inv[1][1] = m[0][0] * m[2][2] * m[3][3] - m[0][0] * m[2][3] * m[3][2] -
                m[2][0] * m[0][2] * m[3][3] + m[2][0] * m[0][3] * m[3][2] +
                m[3][0] * m[0][2] * m[2][3] - m[3][0] * m[0][3] * m[2][2];

    inv[2][1] = -m[0][0] * m[2][1] * m[3][3] + m[0][0] * m[2][3] * m[3][1] +
                m[2][0] * m[0][1] * m[3][3] - m[2][0] * m[0][3] * m[3][1] -
                m[3][0] * m[0][1] * m[2][3] + m[3][0] * m[0][3] * m[2][1];

    inv[3][1] = m[0][0] * m[2][1] * m[3][2] - m[0][0] * m[2][2] * m[3][1] -
                m[2][0] * m[0][1] * m[3][2] + m[2][0] * m[0][2] * m[3][1] +
                m[3][0] * m[0][1] * m[2][2] - m[3][0] * m[0][2] * m[2][1];

    inv[0][2] = m[0][1] * m[1][2] * m[3][3] - m[0][1] * m[1][3] * m[3][2] -
                m[1][1] * m[0][2] * m[3][3] + m[1][1] * m[0][3] * m[3][2] +
                m[3][1] * m[0][2] * m[1][3] - m[3][1] * m[0][3] * m[1][2];

    inv[1][2] = -m[0][0] * m[1][2] * m[3][3] + m[0][0] * m[1][3] * m[3][2] +
                m[1][0] * m[0][2] * m[3][3] - m[1][0] * m[0][3] * m[3][2] -
                m[3][0] * m[0][2] * m[1][3] + m[3][0] * m[0][3] * m[1][2];

    inv[2][2] = m[0][0] * m[1][1] * m[3][3] - m[0][0] * m[1][3] * m[3][1] -
                m[1][0] * m[0][1] * m[3][3] + m[1][0] * m[0][3] * m[3][1] +
                m[3][0] * m[0][1] * m[1][3] - m[3][0] * m[0][3] * m[1][1];

    inv[3][2] = -m[0][0] * m[1][1] * m[3][2] + m[0][0] * m[1][2] * m[3][1] +
                m[1][0] * m[0][1] * m[3][2] - m[1][0] * m[0][2] * m[3][1] -
                m[3][0] * m[0][1] * m[1][2] + m[3][0] * m[0][2] * m[1][1];

    inv[0][3] = -m[0][1] * m[1][2] * m[2][3] + m[0][1] * m[1][3] * m[2][2] +
                m[1][1] * m[0][2] * m[2][3] - m[1][1] * m[0][3] * m[2][2] -
                m[2][1] * m[0][2] * m[1][3] + m[2][1] * m[0][3] * m[1][2];

    inv[1][3] = m[0][0] * m[1][2] * m[2][3] - m[0][0] * m[1][3] * m[2][2] -
                m[1][0] * m[0][2] * m[2][3] + m[1][0] * m[0][3] * m[2][2] +
                m[2][0] * m[0][2] * m[1][3] - m[2][0] * m[0][3] * m[1][2];

    inv[2][3] = -m[0][0] * m[1][1] * m[2][3] + m[0][0] * m[1][3] * m[2][1] +
                m[1][0] * m[0][1] * m[2][3] - m[1][0] * m[0][3] * m[2][1] -
                m[2][0] * m[0][1] * m[1][3] + m[2][0] * m[0][3] * m[1][1];

    inv[3][3] = m[0][0] * m[1][1] * m[2][2] - m[0][0] * m[1][2] * m[2][1] -
                m[1][0] * m[0][1] * m[2][2] + m[1][0] * m[0][2] * m[2][1] +
                m[2][0] * m[0][1] * m[1][2] - m[2][0] * m[0][2] * m[1][1];

    det = m[0][0] * inv[0][0] + m[0][1] * inv[1][0] + m[0][2] * inv[2][0] +
          m[0][3] * inv[3][0];

    if (fabs(det) < 1e-6) return false;

    det = 1.0 / det;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            invOut[i][j] = inv[i][j] * det;
        }
    }

    return true;
}

template <typename T> bool invert4x4Matrix(T **m, T **invOut) {
    T inv[4][4], det;

    inv[0][0] = m[1][1] * m[2][2] * m[3][3] - m[1][1] * m[2][3] * m[3][2] -
                m[2][1] * m[1][2] * m[3][3] + m[2][1] * m[1][3] * m[3][2] +
                m[3][1] * m[1][2] * m[2][3] - m[3][1] * m[1][3] * m[2][2];

    inv[1][0] = -m[1][0] * m[2][2] * m[3][3] + m[1][0] * m[2][3] * m[3][2] +
                m[2][0] * m[1][2] * m[3][3] - m[2][0] * m[1][3] * m[3][2] -
                m[3][0] * m[1][2] * m[2][3] + m[3][0] * m[1][3] * m[2][2];

    inv[2][0] = m[1][0] * m[2][1] * m[3][3] - m[1][0] * m[2][3] * m[3][1] -
                m[2][0] * m[1][1] * m[3][3] + m[2][0] * m[1][3] * m[3][1] +
                m[3][0] * m[1][1] * m[2][3] - m[3][0] * m[1][3] * m[2][1];

    inv[3][0] = -m[1][0] * m[2][1] * m[3][2] + m[1][0] * m[2][2] * m[3][1] +
                m[2][0] * m[1][1] * m[3][2] - m[2][0] * m[1][2] * m[3][1] -
                m[3][0] * m[1][1] * m[2][2] + m[3][0] * m[1][2] * m[2][1];

    inv[0][1] = -m[0][1] * m[2][2] * m[3][3] + m[0][1] * m[2][3] * m[3][2] +
                m[2][1] * m[0][2] * m[3][3] - m[2][1] * m[0][3] * m[3][2] -
                m[3][1] * m[0][2] * m[2][3] + m[3][1] * m[0][3] * m[2][2];

    inv[1][1] = m[0][0] * m[2][2] * m[3][3] - m[0][0] * m[2][3] * m[3][2] -
                m[2][0] * m[0][2] * m[3][3] + m[2][0] * m[0][3] * m[3][2] +
                m[3][0] * m[0][2] * m[2][3] - m[3][0] * m[0][3] * m[2][2];

    inv[2][1] = -m[0][0] * m[2][1] * m[3][3] + m[0][0] * m[2][3] * m[3][1] +
                m[2][0] * m[0][1] * m[3][3] - m[2][0] * m[0][3] * m[3][1] -
                m[3][0] * m[0][1] * m[2][3] + m[3][0] * m[0][3] * m[2][1];

    inv[3][1] = m[0][0] * m[2][1] * m[3][2] - m[0][0] * m[2][2] * m[3][1] -
                m[2][0] * m[0][1] * m[3][2] + m[2][0] * m[0][2] * m[3][1] +
                m[3][0] * m[0][1] * m[2][2] - m[3][0] * m[0][2] * m[2][1];

    inv[0][2] = m[0][1] * m[1][2] * m[3][3] - m[0][1] * m[1][3] * m[3][2] -
                m[1][1] * m[0][2] * m[3][3] + m[1][1] * m[0][3] * m[3][2] +
                m[3][1] * m[0][2] * m[1][3] - m[3][1] * m[0][3] * m[1][2];

    inv[1][2] = -m[0][0] * m[1][2] * m[3][3] + m[0][0] * m[1][3] * m[3][2] +
                m[1][0] * m[0][2] * m[3][3] - m[1][0] * m[0][3] * m[3][2] -
                m[3][0] * m[0][2] * m[1][3] + m[3][0] * m[0][3] * m[1][2];

    inv[2][2] = m[0][0] * m[1][1] * m[3][3] - m[0][0] * m[1][3] * m[3][1] -
                m[1][0] * m[0][1] * m[3][3] + m[1][0] * m[0][3] * m[3][1] +
                m[3][0] * m[0][1] * m[1][3] - m[3][0] * m[0][3] * m[1][1];

    inv[3][2] = -m[0][0] * m[1][1] * m[3][2] + m[0][0] * m[1][2] * m[3][1] +
                m[1][0] * m[0][1] * m[3][2] - m[1][0] * m[0][2] * m[3][1] -
                m[3][0] * m[0][1] * m[1][2] + m[3][0] * m[0][2] * m[1][1];

    inv[0][3] = -m[0][1] * m[1][2] * m[2][3] + m[0][1] * m[1][3] * m[2][2] +
                m[1][1] * m[0][2] * m[2][3] - m[1][1] * m[0][3] * m[2][2] -
                m[2][1] * m[0][2] * m[1][3] + m[2][1] * m[0][3] * m[1][2];

    inv[1][3] = m[0][0] * m[1][2] * m[2][3] - m[0][0] * m[1][3] * m[2][2] -
                m[1][0] * m[0][2] * m[2][3] + m[1][0] * m[0][3] * m[2][2] +
                m[2][0] * m[0][2] * m[1][3] - m[2][0] * m[0][3] * m[1][2];

    inv[2][3] = -m[0][0] * m[1][1] * m[2][3] + m[0][0] * m[1][3] * m[2][1] +
                m[1][0] * m[0][1] * m[2][3] - m[1][0] * m[0][3] * m[2][1] -
                m[2][0] * m[0][1] * m[1][3] + m[2][0] * m[0][3] * m[1][1];

    inv[3][3] = m[0][0] * m[1][1] * m[2][2] - m[0][0] * m[1][2] * m[2][1] -
                m[1][0] * m[0][1] * m[2][2] + m[1][0] * m[0][2] * m[2][1] +
                m[2][0] * m[0][1] * m[1][2] - m[2][0] * m[0][2] * m[1][1];

    det = m[0][0] * inv[0][0] + m[0][1] * inv[1][0] + m[0][2] * inv[2][0] +
          m[0][3] * inv[3][0];

    if (fabs(det) < 1e-6) return false;

    det = 1.0 / det;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            invOut[i][j] = inv[i][j] * det;
        }
    }

    return true;
}

template <typename T>
__host__ void matrixTranspose(T **A, T **B, int row, int col) {
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
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
    for (size_t i = 0; i < rowA; i++) {
        for (size_t j = 0; j < colB; j++) {
            res[i][j] = 0;
            for (size_t k = 0; k < rowB; k++) {
                res[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}
