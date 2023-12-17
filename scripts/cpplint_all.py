#! /usr/bin/env python3

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

# Kasraa: This script find all C++ files under a parent path and `cpplint`s
# them

import os, sys
from pathlib import Path
import re
import shlex, shutil
import subprocess as sp
import cpplint  # Make sure it's installed
from scriptlib import isDontCareFile


DONT_CARE_MESSAGES = [".*Include the directory when naming header files.*",
                      ".*Consider using rand_r\(\.\.\.\) instead of rand\(\.\.\.\) for improved thread safety.*",
                      ".*<regex> is an unapproved C\+\+11 header.*",
                      ".*<chrono> is an unapproved C\+\+11 header.*",
                      ".*gpu/src/mloc/pfilter\.h:153:  Lines should be <= 80 characters long.*",
                      ".*gpu/src/mloc/pfilter\.h.*Is this a non-const reference\? If so, make const or use a pointer: double &probability.*",
                      ]



def filterLintOut(log):
    filteredLog = ''
    for l in log.splitlines():
        isDontCareMsg = False
        for dcm in DONT_CARE_MESSAGES:
            m = re.search(dcm, l)
            if m: isDontCareMsg = True
        if isDontCareMsg: continue
        if l.strip(): filteredLog += l + '\n'
    return filteredLog



def checkCppLint(filePath):
    proc = sp.run(shlex.split(f'cpplint {filePath}'), stdout=sp.PIPE, stderr=sp.PIPE)
    log = proc.stderr.decode().strip()
    fLog = filterLintOut(log)
    if fLog.strip():
        print(fLog, end='')



if __name__ == '__main__':
    if len(sys.argv[1:]) != 1:
        print('Missing parent path')
        exit(1)

    exts = ['.cpp', '.hpp', '.c', '.h', '.cu']
    for e in exts:
        allFiles = list(Path(sys.argv[1]).glob('**/*'+e))
        for filePath in allFiles:
            if not isDontCareFile(filePath):
                checkCppLint(filePath)
            else:
                # print(f'{filePath} is a don\'t core file')
                pass
