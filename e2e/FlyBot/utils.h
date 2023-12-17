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

#include "parallel_hashmap/phmap.h"

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
