# MIT License
#
# Copyright (c) 2023 Carnegie Mellon University
#
# This file is part of RoWild.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import re


DONT_CARE_FILES = [".*cpu/include/nlohmann/.*",
                   ".*cpu/include/parallel_hashmap/.*",
                   ".*cpu/include/vcl/.*",
                   ".*cpu/include/args.h",
                   ".*cpu/include/glob.hpp",
                   ".*cpu/include/stb_image.h",
                   ".*cpu/include/log.h",
                   ".*cpu/include/zsim_hooks.h",
                   ".*cpu/include/pad.h",
                   ".*cpu/src/motc/ldl.cpp",
                   ".*cpu/src/motc/matrix_support.cpp",
                   ".*cpu/src/motc/solver.cpp",
                   ".*cpu/src/motc/solver.h",
                   ".*cpu/src/motc/util.cpp",
                   ".*gpu/include/args.h",
                   ".*gpu/include/log.h",
                   ".*gpu/include/timer.h",
                   ".*gpu/include/moderngpu/.*",
                   ".*gpu/src/nav/.*/GPU-kernel.cuh",
                   ".*gpu/src/nav/.*/GPU-solver.cu",
                   ".*gpu/src/nav/.*/GPU-solver.hpp",
                   ".*gpu/src/nav/.*/mgpucontext.cu",
                   ".*gpu/src/nav/.*/mgpuutil.cpp",
                   ".*gpu/include/nlohmann/json.hpp",
                   ".*gpu/include/stb_image.h",
                   ".*gpu/include/glob.hpp",
                   ".*gpu/include/vcl/.*",
                   ".*e2e/.*stb_image.h",
                   ".*e2e/.*parallel_hashmap.*",
                   ".*e2e/.*nlohmann/json.hpp",
                   ".*/vcl/.*",
                   ".*e2e/.*/args.h",
                   ".*e2e/.*/log.h",
                   ".*e2e/FlyBot/coin/.*",
                   ".*e2e/HomeBot/glob.hpp",
                   ".*cpp-for-fpga.*",
                   ]


def isDontCareFile(filePath):
    for dcf in DONT_CARE_FILES:
        m = re.search(dcf, str(filePath))
        if m: return True
    return False
