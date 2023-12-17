/** $lic$
 * Copyright (C) 2012-2015 by Massachusetts Institute of Technology
 * Copyright (C) 2010-2013 by The Board of Trustees of Stanford University
 *
 * This file is part of zsim.
 *
 * zsim is free software; you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, version 2.
 *
 * If you use this software in your research, we request that you reference
 * the zsim paper ("ZSim: Fast and Accurate Microarchitectural Simulation of
 * Thousand-Core Systems", Sanchez and Kozyrakis, ISCA-40, June 2013) as the
 * source of the simulator in any publications that use this software, and that
 * you send us a citation of your work.
 *
 * zsim is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

/* General logging/info/warn/panic routines */

#pragma once

#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <cuda_runtime.h>

#define PANIC_EXIT_CODE (112)

// assertions are often frequently executed but never inlined. Might as well tell the compiler about it
#define likely(x)       __builtin_expect((x), 1)
#define unlikely(x)     __builtin_expect((x), 0)

class PrintExpr {
    private:
        std::stringstream& ss;

    public:
        PrintExpr(std::stringstream& _ss) : ss(_ss) {}

        // Start capturing values
        template<typename T> const PrintExpr operator->* (T t) const { ss << t; return *this; }

        // Overloads for all lower-precedence operators
        template<typename T> const PrintExpr operator == (T t) const { ss << " == " << t; return *this; }
        template<typename T> const PrintExpr operator != (T t) const { ss << " != " << t; return *this; }
        template<typename T> const PrintExpr operator <= (T t) const { ss << " <= " << t; return *this; }
        template<typename T> const PrintExpr operator >= (T t) const { ss << " >= " << t; return *this; }
        template<typename T> const PrintExpr operator <  (T t) const { ss << " < "  << t; return *this; }
        template<typename T> const PrintExpr operator >  (T t) const { ss << " > "  << t; return *this; }
        template<typename T> const PrintExpr operator &  (T t) const { ss << " & "  << t; return *this; }
        template<typename T> const PrintExpr operator |  (T t) const { ss << " | "  << t; return *this; }
        template<typename T> const PrintExpr operator ^  (T t) const { ss << " ^ "  << t; return *this; }
        template<typename T> const PrintExpr operator && (T t) const { ss << " && " << t; return *this; }
        template<typename T> const PrintExpr operator || (T t) const { ss << " || " << t; return *this; }
        template<typename T> const PrintExpr operator +  (T t) const { ss << " + "  << t; return *this; }
        template<typename T> const PrintExpr operator -  (T t) const { ss << " - "  << t; return *this; }
        template<typename T> const PrintExpr operator *  (T t) const { ss << " * "  << t; return *this; }
        template<typename T> const PrintExpr operator /  (T t) const { ss << " / "  << t; return *this; }
        template<typename T> const PrintExpr operator %  (T t) const { ss << " % "  << t; return *this; }
        template<typename T> const PrintExpr operator << (T t) const { ss << " << " << t; return *this; }
        template<typename T> const PrintExpr operator >> (T t) const { ss << " >> " << t; return *this; }

        // std::nullptr_t overloads (for nullptr's in assertions)
        // Only a few are needed, since most ops w/ nullptr are invalid
        const PrintExpr operator->* (std::nullptr_t t) const { ss << "nullptr"; return *this; }
        const PrintExpr operator == (std::nullptr_t t) const { ss << " == nullptr"; return *this; }
        const PrintExpr operator != (std::nullptr_t t) const { ss << " != nullptr"; return *this; }

    private:
        template<typename T> const PrintExpr operator =  (T t) const;  // will fail, can't assign in assertion
};

inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true) {
   if (code != cudaSuccess) {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      if (abort) exit(code);
   }
}

#define panic(args...) \
{ \
    fprintf(stderr, "Panic on %s:%d: ", __FILE__, __LINE__); \
    fprintf(stderr, args); \
    fprintf(stderr, "\n"); \
    fflush(stderr); \
    exit(PANIC_EXIT_CODE); \
}

#define warn(args...) \
{ \
    fprintf(stderr, "WARN: "); \
    fprintf(stderr, args); \
    fprintf(stderr, "\n"); \
    fflush(stderr); \
}

#define info(args...) \
{ \
    fprintf(stdout, args); \
    fprintf(stdout, "\n"); \
    fflush(stdout); \
}

#ifndef NASSERT
#undef assert
#define assert(expr) \
if (unlikely(!(expr))) { \
    std::stringstream __assert_ss__LINE__; (PrintExpr(__assert_ss__LINE__)->*expr); \
    fprintf(stderr, "Failed assertion on %s:%d '%s' (with '%s')\n", __FILE__, __LINE__, #expr, __assert_ss__LINE__.str().c_str()); \
    fflush(stderr); \
    exit(1); \
};

#define assert_msg(cond, args...) \
if (unlikely(!(cond))) { \
    fprintf(stderr, "Failed assertion on %s:%d: ", __FILE__, __LINE__); \
    fprintf(stderr, args); \
    fprintf(stderr, "\n"); \
    fflush(stderr); \
    exit(1); \
};

#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }

#else
// Avoid unused warnings, never emit any code
// see http://cnicholson.net/2009/02/stupid-c-tricks-adventures-in-assert/
#define assert(cond) do { (void)sizeof(cond); } while (0);
#define assert_msg(cond, args...) do { (void)sizeof(cond); } while (0);
#define gpuErrchk(ans) do { (void)sizeof(ans); } while (0);
#endif

#define checkpoint()                                            \
    do {                                                        \
        info("%s:%d %s", __FILE__, __LINE__, __FUNCTION__);     \
    } while (0)
