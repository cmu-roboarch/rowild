#! /usr/bin/env bash

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

if [ "$#" -ne 1 ]; then
    echo "Illegal number of arguments"
    exit 1
fi

if [[ ! -d $1 ]]; then
    echo "Couldn't find directory $1"
    exit 1
fi

cd "$1" || echo "Can't cd to $1" | exit 1

rm ./*.m
rm csolve.c
rm Makefile
sed -i 's/^int main/\/\/ int main/g' solver.h

for file in *.c; do
    NAME=$(basename "$file" .c)
    mv "$file" "${NAME}.cpp"
done

mv description.cvxgen ldl.cpp matrix_support.cpp solver.* util.cpp ../

cd .. || true

exit 0
