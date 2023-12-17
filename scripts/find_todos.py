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

# Kasraa: This script simply finds TODOs and FIXMEs in the files under a given
# root path. The point of using this script instead of sheer `rg` is to filter
# annoyingly-many TODOs in third-party files (listed in scriptlib.py).

import sys
from pathlib import Path
from scriptlib import isDontCareFile
import re
import shlex, shutil
import subprocess as sp
from columnar import columnar


TODO_PATTERNS = ['TODO', 'FIXME']    # Case-sensitive
outLog = []


def containsPattern(line):
    for pattern in TODO_PATTERNS:
        if pattern in line:
            return True
    return False



def findTODOs(filePath):
    global outLog
    f = open(filePath, 'r')
    content = f.readlines()
    todoFound = False
    for i, line in enumerate(content):
        if not containsPattern(line): continue
        todoFound = True
        outLog.append([filePath, i+1, line.strip()])
    assert todoFound



if __name__ == '__main__':
    if len(sys.argv[1:]) != 1:
        print('Missing parent path')
        exit(1)

    oredPattern = ''
    for p in TODO_PATTERNS:
        oredPattern += f'|{p}'
    oredPattern = oredPattern[1:]

    proc = sp.run(shlex.split(f'rg \'{oredPattern}\' {sys.argv[1]} --files-with-matches'), stdout=sp.PIPE, stderr=sp.PIPE)
    log = proc.stdout.decode().strip()
    for l in log.splitlines():
        if isDontCareFile(l): continue
        if Path(l) == Path(__file__): continue
        findTODOs(l)

    if len(outLog):
        print(columnar(outLog, ['File Path', 'Line Number', 'Code']))
