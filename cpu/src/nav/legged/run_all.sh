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

BINARY="legged.out"
INPUT_DIRECTORY="./input-maps"
OUTPUT_DIRECTORY="./output-paths"

if [[ ! -f $BINARY || ! -d $INPUT_DIRECTORY ]]; then
    echo "Missing required files!"
    echo "Binary: $BINARY ?"
    echo "Input directory: $INPUT_DIRECTORY ?"
    exit 1
fi

execute() {
    local INPUT_FILE=$1
    local GRID=$2
    local WEIGHT=$3
    local HEURISTIC=$4
    local MAP_SCALE=$5
    local DEEPENING=$6

    mkdir -p "$OUTPUT_DIRECTORY"
    local MAP_NAME=$(basename "$INPUT_FILE" .map)

    local OUTPUT_FILE="$OUTPUT_DIRECTORY/"
    OUTPUT_FILE="${OUTPUT_FILE}map-${MAP_NAME}_"
    OUTPUT_FILE="${OUTPUT_FILE}g-${GRID}_"
    OUTPUT_FILE="${OUTPUT_FILE}w-${WEIGHT}_"
    OUTPUT_FILE="${OUTPUT_FILE}h-${HEURISTIC}_"
    OUTPUT_FILE="${OUTPUT_FILE}mscal-${MAP_SCALE}_"
    OUTPUT_FILE="${OUTPUT_FILE}deep-${DEEPENING}"
    OUTPUT_FILE="${OUTPUT_FILE}.txt"

    local ARGUMENTS=""
    ARGUMENTS="$ARGUMENTS--map=$INPUT_FILE "
    ARGUMENTS="$ARGUMENTS--grid=$GRID "
    ARGUMENTS="$ARGUMENTS--weight=$WEIGHT "
    ARGUMENTS="$ARGUMENTS--heuristic=$HEURISTIC "
    ARGUMENTS="$ARGUMENTS--scale-map=$MAP_SCALE "
    ARGUMENTS="$ARGUMENTS--output=$OUTPUT_FILE "

    if [[ "$DEEPENING" == 1 ]]; then
        ARGUMENTS="$ARGUMENTS--deepening "
    fi

    ./$BINARY $ARGUMENTS
    if [[ $? != 0 ]]; then
        echo "Error in running ./${BINARY} ${ARGUMENTS}"
    fi

    echo "--------------------------------------"
}

ALL_GRIDS=(4 8)
ALL_WEIGHTS=(1 10)
ALL_HEURISTICS=("Euclidean" "Manhattan")
ALL_MAP_SCALES=(1 2)
ALL_DEEPENINGS=(0)

for INPUT_FILE in "$INPUT_DIRECTORY"/*; do
    for GRID in "${ALL_GRIDS[@]}"; do
        for WEIGHT in "${ALL_WEIGHTS[@]}"; do
            for HEURISTIC in "${ALL_HEURISTICS[@]}"; do
                for MAP_SCALE in "${ALL_MAP_SCALES[@]}"; do
                    for DEEPENING in "${ALL_DEEPENINGS[@]}"; do
                        execute "$INPUT_FILE" "$GRID" "$WEIGHT" "$HEURISTIC" "$MAP_SCALE" "$DEEPENING" &
                    done
                done
            done
        done
    done
done

wait

echo "**************************************"

exit 0
