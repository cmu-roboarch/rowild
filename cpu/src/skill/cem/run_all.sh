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

BINARY="cem.out"
SIMULATOR_LOG="./simulator.txt"
OUTPUT_DIRECTORY="./output-logs"

if [[ ! -f $BINARY || ! -f $SIMULATOR_LOG ]]; then
    echo "Missing required files!"
    echo "Binary: $BINARY ?"
    echo "Input simulator: $SIMULATOR_LOG ?"
    exit 1
fi

execute() {
    local NUM_UPDATES=$1
    local NUM_SAMPLES=$2
    local TOPN=$3

    mkdir -p "$OUTPUT_DIRECTORY"
    local OUTPUT_FILE="$OUTPUT_DIRECTORY/"
    OUTPUT_FILE="${OUTPUT_FILE}u-${NUM_UPDATES}_"
    OUTPUT_FILE="${OUTPUT_FILE}s-${NUM_SAMPLES}_"
    OUTPUT_FILE="${OUTPUT_FILE}tn-${TOPN}"
    OUTPUT_FILE="${OUTPUT_FILE}.txt"

    local ARGUMENTS=""
    ARGUMENTS="$ARGUMENTS--simulator=$SIMULATOR_LOG "
    ARGUMENTS="$ARGUMENTS--updates=$NUM_UPDATES "
    ARGUMENTS="$ARGUMENTS--samples=$NUM_SAMPLES "
    ARGUMENTS="$ARGUMENTS--topn=$TOPN "
    ARGUMENTS="$ARGUMENTS--output=$OUTPUT_FILE "

    ./$BINARY $ARGUMENTS
    if [[ $? != 0 ]]; then
        echo "Error in running ./${BINARY} ${ARGUMENTS}"
    fi

    echo "--------------------------------------"
}

ALL_NUM_UPDATES=(10 15)
ALL_NUM_SAMPLES=(10 15)
ALL_TOPNS=(5 10)

for NUM_UPDATES in "${ALL_NUM_UPDATES[@]}"; do
    for NUM_SAMPLES in "${ALL_NUM_SAMPLES[@]}"; do
        for TOPN in "${ALL_TOPNS[@]}"; do
            execute "$NUM_UPDATES" "$NUM_SAMPLES" "$TOPN" &
        done
    done
done

wait

echo "**************************************"

exit 0
