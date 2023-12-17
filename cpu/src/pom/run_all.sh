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

BINARY="pom.out"
INPUT_DIRECTORY="./input-measurements"
OUTPUT_DIRECTORY="./output-grids"

if [[ ! -f $BINARY || ! -d $INPUT_DIRECTORY ]]; then
    echo "Missing required files!"
    echo "Binary: $BINARY ?"
    echo "Input directory: $INPUT_DIRECTORY ?"
    exit 1
fi

execute() {
    local INPUT_FILE=$1
    local GRID_SIZE=$2
    local INITP=$3
    local OCCP=$4
    local FREEP=$5

    mkdir -p "$OUTPUT_DIRECTORY"
    local MEAS_FILE_NAME=$(basename "$INPUT_FILE" .txt)

    local OUTPUT_FILE="$OUTPUT_DIRECTORY/"
    OUTPUT_FILE="${OUTPUT_FILE}meas-${MEAS_FILE_NAME}_"
    OUTPUT_FILE="${OUTPUT_FILE}size-${GRID_SIZE}_"
    OUTPUT_FILE="${OUTPUT_FILE}initp-${INITP}_"
    OUTPUT_FILE="${OUTPUT_FILE}occp-${OCCP}_"
    OUTPUT_FILE="${OUTPUT_FILE}freep-${FREEP}"
    OUTPUT_FILE="${OUTPUT_FILE}.txt"

    local ARGUMENTS=""
    ARGUMENTS="$ARGUMENTS--input=$INPUT_FILE "
    ARGUMENTS="$ARGUMENTS--size=$GRID_SIZE "
    ARGUMENTS="$ARGUMENTS--initprob=$INITP "
    ARGUMENTS="$ARGUMENTS--occprob=$OCCP "
    ARGUMENTS="$ARGUMENTS--freeprob=$FREEP "
    ARGUMENTS="$ARGUMENTS--output=$OUTPUT_FILE "

    ./$BINARY $ARGUMENTS
    if [[ $? != 0 ]]; then
        echo "Error in running ./${BINARY} ${ARGUMENTS}"
    fi

    echo "--------------------------------------"
}

ALL_GRID_SIZES=(100)
ALL_INIT_PROBS=(0.5)
ALL_OCC_PROBS=(0.7)
ALL_FREE_PROBS=(0.3)

for INPUT_FILE in "$INPUT_DIRECTORY"/*.txt; do
    for GRID_SIZE in "${ALL_GRID_SIZES[@]}"; do
        for INITP in "${ALL_INIT_PROBS[@]}"; do
            for OCCP in "${ALL_OCC_PROBS[@]}"; do
                for FREEP in "${ALL_FREE_PROBS[@]}"; do
                    execute "$INPUT_FILE" "$GRID_SIZE" "$INITP" "$OCCP" "$FREEP" &
                done
            done
        done
    done
done

wait

echo "**************************************"

exit 0
