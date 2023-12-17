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

BINARY="mpc.out"
OUTPUT_DIRECTORY="./output-logs"

if [[ ! -f $BINARY ]]; then
    echo "Missing required files!"
    echo "Binary: $BINARY ?"
    exit 1
fi

execute() {
    local HORIZON=$1
    local DT=$2

    mkdir -p "$OUTPUT_DIRECTORY"
    local OUTPUT_FILE="$OUTPUT_DIRECTORY/"
    OUTPUT_FILE="${OUTPUT_FILE}h-${HORIZON}_"
    OUTPUT_FILE="${OUTPUT_FILE}dt-${DT}"
    OUTPUT_FILE="${OUTPUT_FILE}.txt"

    local ARGUMENTS=""
    ARGUMENTS="$ARGUMENTS--horizon=$HORIZON "
    ARGUMENTS="$ARGUMENTS--dt=$DT "
    ARGUMENTS="$ARGUMENTS--output=$OUTPUT_FILE "

    ./$BINARY $ARGUMENTS
    if [[ $? != 0 ]]; then
        echo "Error in running ./${BINARY} ${ARGUMENTS}"
    fi

    echo "--------------------------------------"
}

ALL_HORIZONS=(10 100)
ALL_DTS=(0.1 0.5)

for HORIZON in "${ALL_HORIZONS[@]}"; do
    for DT in "${ALL_DTS[@]}"; do
        execute "$HORIZON" "$DT"
    done
done

wait

echo "**************************************"

exit 0
