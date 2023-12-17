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

import sys, os

def checkFile(fileName):
    with open(fileName, 'r') as f:
        lines = f.readlines()

    numFuncs = 0
    theStr = ''
    for l, line in enumerate(lines):
        if line == "}\n":
            numFuncs += 1
            if l == len(lines) - 1: continue
            if l == len(lines) - 2:
                theStr += f'What is going on in line {l}\n'
                continue
            if lines[l+1].isspace() and lines[l+2].isspace(): pass
            else:
                theStr += f'A function ends in line {l} but the following two lines are not white space\n'

    theStr += f'processed {numFuncs} functions!'
    return theStr


if __name__ == '__main__':
    if len(sys.argv[1:]) != 1:
        print('Missing input file')
        exit(1)

    print(checkFile(sys.argv[1]))
