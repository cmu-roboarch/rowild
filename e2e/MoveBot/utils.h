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
#define PI (3.141592654)

template <typename T> static void printContainer(const T c) {
    for (auto e : c) {
        std::cout << e << " ";
    }
    std::cout << std::endl;
}

template <typename T> static void printArray(const T a, size_t size) {
    for (size_t i = 0; i < size; i++) {
        std::cout << a[i] << " ";
    }
    std::cout << std::endl;
}

template <typename T, size_t size> static void printArray(const T (&a)[size]) {
    for (size_t i = 0; i < size; i++) {
        std::cout << a[i] << " ";
    }
    std::cout << std::endl;
}

template <typename T> void printMatrix(const T **A, int rows, int cols) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            std::cout << A[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

template <typename T, size_t rows, size_t cols>
void printMatrix(const T (&A)[rows][cols]) {
    for (size_t i = 0; i < rows; i++) {
        for (size_t j = 0; j < cols; j++) {
            std::cout << A[i][j] << " ";
        }
        std::cout << std::endl;
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

template <typename T> void print3dArray(const T ***A, int d1, int d2, int d3) {
    // A[d1][d2][d3]
    std::cout << "{";
    for (int i = 0; i < d1; i++) {
        std::cout << "<";
        for (int j = 0; j < d2; j++) {
            std::cout << "(";
            for (int k = 0; k < d3; k++) {
                std::cout << A[i][j][k];
                if (k != d3 - 1) std::cout << ", ";
            }
            std::cout << ")";
            if (j != d2 - 1) std::cout << ", ";
        }
        std::cout << ">";
        if (i != d1 - 1) std::cout << ", ";
    }
    std::cout << "}";
}

template <typename T, size_t d1, size_t d2, size_t d3>
void print3dArray(const T (&A)[d1][d2][d3]) {
    std::cout << "{";
    for (size_t i = 0; i < d1; i++) {
        std::cout << "<";
        for (size_t j = 0; j < d2; j++) {
            std::cout << "(";
            for (size_t k = 0; k < d3; k++) {
                std::cout << A[i][j][k];
                if (k != d3 - 1) std::cout << ", ";
            }
            std::cout << ")";
            if (j != d2 - 1) std::cout << ", ";
        }
        std::cout << ">";
        if (i != d1 - 1) std::cout << ", ";
    }
    std::cout << "}";
}

template <typename T>
void blockDescToBoundingBox(T (&H)[4][4], T (&dim)[3], T (&corners)[9][3]) {
    /*
     * This function Takes in an object pose (specified by a homogeneous
     * transformation matrix, H) and object dimensions (length(x), width(y),
     * height(z)) and returns the bounding box of the object
     *
     * This function is performance-critical and must be highly optimized. I
     * unroll the loops manually to increase ILP as much as possible.
     */

    T xL[3] = {H[0][0] * dim[0] / 2.0, H[1][0] * dim[0] / 2.0,
               H[2][0] * dim[0] / 2.0};
    T yL[3] = {H[0][1] * dim[1] / 2.0, H[1][1] * dim[1] / 2.0,
               H[2][1] * dim[1] / 2.0};
    T zL[3] = {H[0][2] * dim[2] / 2.0, H[1][2] * dim[2] / 2.0,
               H[2][2] * dim[2] / 2.0};

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
void rpyxyzToH(T roll, T pitch, T yaw, T x, T y, T z, T (&H)[4][4]) {
    /*
     * This function takes a cuboid's origin (x, y, z) and orientation and
     * calculates the homogeneous transformation matrix, H. The orientation is
     * represented by the standard roll-pitch-yaw convention, where roll,
     * pitch, and yaw represents the angle about the body's x-axis, y-axis, and
     * z-axis, respectively.
     *
     * This function is performance-critical and must be highly optimized. I
     * allocate most strutures in stack and unroll the loops manually to
     * increase ILP as much as possible.
     *
     * Also, not that surprisingly, I find even passing 6 arguments better than
     * storing the array in memory and passing a pointer to it.
     */

    auto matMul4x4 = [](T(&A)[4][4], T(&B)[4][4], T(&C)[4][4]) {
        for (size_t i = 0; i < 4; i++) {
            for (size_t j = 0; j < 4; j++) {
                C[i][j] = 0;
                for (size_t k = 0; k < 4; k++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }

        return C;
    };

    T Ht[4][4] = {{1, 0, 0, x}, {0, 1, 0, y}, {0, 0, 1, z}, {0, 0, 0, 1}};

    T Hx[4][4] = {{1, 0, 0, 0},
                  {0, cos(roll), -sin(roll), 0},
                  {0, sin(roll), cos(roll), 0},
                  {0, 0, 0, 1}};

    T Hy[4][4] = {{cos(pitch), 0, sin(pitch), 0},
                  {0, 1, 0, 0},
                  {-sin(pitch), 0, cos(pitch), 0},
                  {0, 0, 0, 1}};

    T Hz[4][4] = {{cos(yaw), -sin(yaw), 0, 0},
                  {sin(yaw), cos(yaw), 0, 0},
                  {0, 0, 1, 0},
                  {0, 0, 0, 1}};

    T t1[4][4] = {};
    T t2[4][4] = {};

    matMul4x4(Ht, Hz, t1);
    matMul4x4(t1, Hy, t2);
    matMul4x4(t2, Hx, H);
}

template <typename T>
bool pointsOverlap(T (&p1)[9][3], T (&p2)[9][3], T (&axis)[3]) {
    /*
     * This function checks whether two points overlap over an axis. This
     * function is called as a part of collision detection, and hence, its
     * performance is CRUCIALLY important. That's why everything is so
     * hard-wired.
     */

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
bool cuboidCuboidCollision(T (&p1)[9][3], T (&axes1)[3][3], T (&p2)[9][3],
                           T (&axes2)[3][3]) {
    auto fastNorm = [](T x[3]) {
        return sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
    };

    auto diffNorm = [&fastNorm](T x[3], T y[3]) {
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
    auto crossMult = [](T(&A)[3], T(&B)[3], T(&C)[3]) {
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
