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

#include "log.h"
#include "parallel_hashmap/phmap.h"
#include "vec_utils.h"
#include <algorithm>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <list>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#define PI (3.141592654)
#define TWO_PI (6.283185308)

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

template <typename T> static void printNonIterableContrainer(T c) {
    while (!c.empty()) {
        std::cout << c.top() << " ";
        c.pop();
    }
    std::cout << std::endl;
}

template <typename T> static std::string containerToString(const T c) {
    std::string str = "[";
    for (auto e : c) {
        str += std::to_string(e);
        str += " ";
    }
    str.pop_back();
    str.append("]");
    return str;
}

// The base class for the "enhanced" parallel set (EPS). It supports containers
// like vector as the key. The vector, depending on its "size," is hashed using
// one of the above functions.
template <typename T> class BaseEPS {
  public:
    static_assert(std::is_same_v<T, std::vector<int>> ||
                  std::is_same_v<T, std::list<int>>);

    BaseEPS() = default;

    void insert(const T &c) {
        uint64_t h = this->getContainerHash(c);
        this->theSet.insert(h);
        assert(this->contains(c));
    }

    bool contains(const T &c) {
        uint64_t h = this->getContainerHash(c);
        return this->theSet.contains(h);
    }

    void clear() { this->theSet.clear(); }

  protected:
    virtual uint64_t getContainerHash(const T &c) = 0;

  private:
    phmap::parallel_flat_hash_set<uint64_t> theSet;
};

template <typename T> class ShortEPS : public BaseEPS<T> {
  public:
    explicit ShortEPS(int shamount) : BaseEPS<T>() {
        this->shamount = shamount;
    }

  protected:
    inline uint64_t getContainerHash(const T &c) {
        // Hash the container C of type integer. Different elements of C
        // are "mapped" to different bits of the output. The assumption is
        // the values are strictly smaller than 2^shamount
        uint64_t r = 0;
        for (const auto &e : c) {
            r <<= this->shamount;
            r += e;
        }

        return r;
    }

  private:
    int shamount;
};

template <typename T> class LongEPS : public BaseEPS<T> {
  public:
    LongEPS() : BaseEPS<T>() {}

  protected:
    inline uint64_t getContainerHash(const T &c) {
        std::stringstream r;
        std::copy(c.begin(), c.end(), std::ostream_iterator<int>(r, ","));
        std::hash<std::string> hasher;
        return hasher(r.str());
    }
};

// Similar to EPS, but the data structure is "map"
template <typename K, typename V> class BaseEPM {
  public:
    static_assert(std::is_same_v<K, std::vector<int>> ||
                  std::is_same_v<K, std::list<int>>);

    BaseEPM() = default;

    void insert(const K &c, V value) {
        uint64_t h = this->getContainerHash(c);
        this->theMap[h] = value;
        assert(this->contains(c));
    }

    bool contains(const K &c) {
        uint64_t h = this->getContainerHash(c);
        return this->theMap.contains(h);
    }

    V getValue(const K &c) {
        assert(this->contains(c));
        uint64_t h = this->getContainerHash(c);
        return this->theMap[h];
    }

    void clear() { this->theMap.clear(); }

  protected:
    virtual uint64_t getContainerHash(const K &c) = 0;

  private:
    phmap::parallel_flat_hash_map<uint64_t, V> theMap;
};

template <typename K, typename V> class ShortEPM : public BaseEPM<K, V> {
  public:
    explicit ShortEPM(int shamount) : BaseEPM<K, V>() {
        this->shamount = shamount;
    }

  protected:
    inline uint64_t getContainerHash(const K &c) {
        // Hash the container C of type integer. Different elements of C
        // are "mapped" to different bits of the output. The assumption is
        // the values are strictly smaller than 2^shamount
        uint64_t r = 0;
        for (const auto &e : c) {
            r <<= this->shamount;
            r += e;
        }
        return r;
    }

  private:
    int shamount;
};

template <typename K, typename V> class LongEPM : public BaseEPM<K, V> {
  public:
    LongEPM() : BaseEPM<K, V>() {}

  protected:
    inline uint64_t getContainerHash(const K &c) {
        std::stringstream r;
        std::copy(c.begin(), c.end(), std::ostream_iterator<int>(r, ","));
        std::hash<std::string> hasher;
        return hasher(r.str());
    }
};

template <typename T> double getEuclideanDistance(const T &v1, const T &v2) {
    assert(v1.size() == v2.size());
    double dist = 0;
    for (int i = 0; i < static_cast<int>(v1.size()); i++) {
        dist += (v1[i] - v2[i]) * (v1[i] - v2[i]);
    }

    return sqrt(dist);
}

template <typename T, size_t s>
double getEuclideanDistance(const T &v1, const T &v2) {
    // Could vectorization help? Yes, if s >= 8; otherwise, it doesn't help
    // any. In RoWild, s is either 2 (XY planning) or 3 (XYZ planning), so I
    // leave it here. Consider vectorization in other AI applications like
    // N-puzzle.
    assert(v1.size() == v2.size());
    assert(v1.size() >= s);
    double dist = 0;
    for (size_t i = 0; i < s; i++) {
        dist += (v1[i] - v2[i]) * (v1[i] - v2[i]);
    }

    return sqrt(dist);
}

template <typename T> int getManhattanDistance(const T &v1, const T &v2) {
    assert(v1.size() == v2.size());
    int dist = 0;
    for (int i = 0; i < static_cast<int>(v1.size()); i++) {
        dist += std::abs(v1[i] - v2[i]);
    }

    return dist;
}

template <typename T, size_t s>
int getManhattanDistance(const T &v1, const T &v2) {
    assert(v1.size() == v2.size());
    assert(v1.size() >= s);
    int dist = 0;
    for (size_t i = 0; i < s; i++) {
        dist += std::abs(v1[i] - v2[i]);
    }

    return dist;
}

template <typename T> bool matrix2x2Inverse(T **A, double **inverse) {
    double det = A[0][0] * A[1][1] - A[1][0] * A[0][1];
    if (unlikely(det == 0)) return false;

    double invDet = 1.0 / det;

    inverse[0][0] = invDet * A[1][1];
    inverse[0][1] = -invDet * A[0][1];
    inverse[1][0] = -invDet * A[1][0];
    inverse[1][1] = invDet * A[0][0];

    return true;
}

template <typename T>
bool matrix2x2Inverse(T (&A)[2][2], double (&inverse)[2][2]) {
    double det = A[0][0] * A[1][1] - A[1][0] * A[0][1];
    if (unlikely(det == 0)) return false;

    double invDet = 1.0 / det;

    inverse[0][0] = invDet * A[1][1];
    inverse[0][1] = -invDet * A[0][1];
    inverse[1][0] = -invDet * A[1][0];
    inverse[1][1] = invDet * A[0][0];

    return true;
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

template <typename T, size_t n>
void matrixInverse(T (&_A)[n][n], double (&_inv)[n][n]) {
    T **A = new T *[n];
    for (size_t i = 0; i < n; i++)
        A[i] = new T[n];
    double **inv = new double *[n];
    for (size_t i = 0; i < n; i++)
        inv[i] = new double[n];

    for (size_t i = 0; i < n; i++) {
        for (size_t j = 0; j < n; j++) {
            A[i][j] = _A[i][j];
        }
    }

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

    for (size_t i = 0; i < n; i++) {
        for (size_t j = 0; j < n; j++) {
            _inv[i][j] = inv[i][j];
        }
    }

    delete[] temp;
    delete[] minor;
    for (size_t i = 0; i < n; i++) {
        delete[] A[i];
        delete[] inv[i];
    }
    delete[] A;
    delete[] inv;
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
void matrixAddition(T **A, T **B, T **res, int rows, int cols) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            res[i][j] = A[i][j] + B[i][j];
        }
    }
}

template <typename T, size_t rows, size_t cols>
void matrixAddition(T (&A)[rows][cols], T (&B)[rows][cols],
                    T (&res)[rows][cols]) {
    for (size_t i = 0; i < rows; i++) {
        for (size_t j = 0; j < cols; j++) {
            res[i][j] = A[i][j] + B[i][j];
        }
    }
}

template <typename T, size_t rows, size_t cols>
void matrixAddition(T **A, T (&B)[rows][cols], T **res) {
    for (size_t i = 0; i < rows; i++) {
        for (size_t j = 0; j < cols; j++) {
            res[i][j] = A[i][j] + B[i][j];
        }
    }
}

template <typename T, size_t rows, size_t cols>
void matrixAddition(T (&A)[rows][cols], T **B, T (&res)[rows][cols]) {
    for (size_t i = 0; i < rows; i++) {
        for (size_t j = 0; j < cols; j++) {
            res[i][j] = A[i][j] + B[i][j];
        }
    }
}

template <typename T, size_t rows, size_t cols>
void matrixAddition(T (&A)[rows][cols], T (&B)[rows][cols], T **res) {
    for (size_t i = 0; i < rows; i++) {
        for (size_t j = 0; j < cols; j++) {
            res[i][j] = A[i][j] + B[i][j];
        }
    }
}

template <typename T, size_t rows, size_t cols>
void matrixSubtraction(T **A, T **B, T **res) {
    for (size_t i = 0; i < rows; i++) {
        for (size_t j = 0; j < cols; j++) {
            res[i][j] = A[i][j] - B[i][j];
        }
    }
}

template <typename T> void cholesky(T **A, T **L, size_t dim) {
    for (size_t i = 0; i < dim; i++) {
        for (size_t j = 0; j < dim; j++) {
            L[i][j] = 0;
        }
    }

    for (size_t i = 0; i < dim; i++) {
        for (size_t j = 0; j <= i; j++) {
            T sum = 0;
            for (size_t k = 0; k < j; k++) {
                sum += L[i][k] * L[j][k];
            }

            if (i == j) {
                L[i][j] = sqrt(A[i][i] - sum);
            } else {
                L[i][j] = (1.0 / L[j][j] * (A[i][j] - sum));
            }
        }
    }
}

template <typename T, size_t dim> void cholesky(T **A, T **L) {
    for (size_t i = 0; i < dim; i++) {
        for (size_t j = 0; j < dim; j++) {
            L[i][j] = 0;
        }
    }

    for (size_t i = 0; i < dim; i++) {
        for (size_t j = 0; j <= i; j++) {
            T sum = 0;
            for (size_t k = 0; k < j; k++) {
                sum += L[i][k] * L[j][k];
            }

            if (i == j) {
                L[i][j] = sqrt(A[i][i] - sum);
            } else {
                L[i][j] = (1.0 / L[j][j] * (A[i][j] - sum));
            }
        }
    }
}

template <typename T, size_t dim>
void cholesky(T (&A)[dim][dim], T (&L)[dim][dim]) {
    for (size_t i = 0; i < dim; i++) {
        for (size_t j = 0; j < dim; j++) {
            L[i][j] = 0;
        }
    }

    for (size_t i = 0; i < dim; i++) {
        for (size_t j = 0; j <= i; j++) {
            T sum = 0;
            for (size_t k = 0; k < j; k++) {
                sum += L[i][k] * L[j][k];
            }

            if (i == j) {
                L[i][j] = sqrt(A[i][i] - sum);
            } else {
                L[i][j] = (1.0 / L[j][j] * (A[i][j] - sum));
            }
        }
    }
}

template <typename T>
void matrixSubtraction(T **A, T **B, T **res, size_t rows, size_t cols) {
    for (size_t i = 0; i < rows; i++) {
        for (size_t j = 0; j < cols; j++) {
            res[i][j] = A[i][j] - B[i][j];
        }
    }
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

template <typename T> T degToRad(T angle) { return angle * (PI / 180); }

template <typename T> T wrapToPi(T angle) {
    while (angle > PI)
        angle -= TWO_PI;
    while (angle < -PI)
        angle += TWO_PI;
    return angle;
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
