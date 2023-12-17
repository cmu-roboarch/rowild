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
#include "vcl/vectorclass.h"
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

template <typename T> constexpr auto getVectorInstance() {
    // This function takes the template parameter T, and returns the
    // corresponding vector type in VCL for AVX-512. If your machine doesn't
    // support AVX-512, the code will still run, but you wouldn't get the same
    // performance.

    if constexpr (std::is_same<T, uint8_t>::value) return Vec64uc();
    if constexpr (std::is_same<T, uint16_t>::value) return Vec32us();
    if constexpr (std::is_same<T, uint32_t>::value) return Vec16ui();
    if constexpr (std::is_same<T, uint64_t>::value) return Vec8uq();

    if constexpr (std::is_same<T, int8_t>::value) return Vec64c();
    if constexpr (std::is_same<T, int16_t>::value) return Vec32s();
    if constexpr (std::is_same<T, int32_t>::value) return Vec16i();
    if constexpr (std::is_same<T, int64_t>::value) return Vec8q();

    if constexpr (std::is_same<T, float>::value) return Vec16f();
    if constexpr (std::is_same<T, double>::value) return Vec8d();
}

template <typename T> constexpr bool isIntegerVec() {
    if constexpr (std::is_same<T, Vec64uc>::value) return true;
    if constexpr (std::is_same<T, Vec32us>::value) return true;
    if constexpr (std::is_same<T, Vec16ui>::value) return true;
    if constexpr (std::is_same<T, Vec8uq>::value) return true;

    if constexpr (std::is_same<T, Vec64c>::value) return true;
    if constexpr (std::is_same<T, Vec32s>::value) return true;
    if constexpr (std::is_same<T, Vec16i>::value) return true;
    if constexpr (std::is_same<T, Vec8q>::value) return true;

    return false;
}

template <typename T> constexpr bool isFloatingPointVec() {
    if constexpr (std::is_same<T, Vec16f>::value) return true;
    if constexpr (std::is_same<T, Vec8d>::value) return true;

    return false;
}
