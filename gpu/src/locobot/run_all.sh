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

BINARY="locobot.out"
MAPS_DIRECTORY="./input-maps"
OUTPUT_DIRECTORY="./output-paths"

if [[ ! -f $BINARY || ! -d $MAPS_DIRECTORY ]]; then
    echo "Missing required files!"
    echo "Binary: $BINARY ?"
    echo "Input maps: $MAPS_DIRECTORY ?"
    exit 1
fi

execute() {
    local INPUT_FILE=$1
    local BIAS=$2
    local SAMPLE=$3
    local THRESHOLD=$4

    local MAP_NAME=$(basename "$INPUT_FILE" .txt)

    mkdir -p "$OUTPUT_DIRECTORY"
    local OUTPUT_FILE="$OUTPUT_DIRECTORY/"
    OUTPUT_FILE="${OUTPUT_FILE}map-${MAP_NAME}_"
    OUTPUT_FILE="${OUTPUT_FILE}b-${BIAS}_"
    OUTPUT_FILE="${OUTPUT_FILE}s-${SAMPLE}_"
    OUTPUT_FILE="${OUTPUT_FILE}t-${THRESHOLD}"
    OUTPUT_FILE="${OUTPUT_FILE}.txt"

    local ARGUMENTS=""
    ARGUMENTS="$ARGUMENTS--map=$INPUT_FILE "
    ARGUMENTS="$ARGUMENTS--bias=$BIAS "
    ARGUMENTS="$ARGUMENTS--samples=$SAMPLE "
    ARGUMENTS="$ARGUMENTS--thresh=$THRESHOLD "
    ARGUMENTS="$ARGUMENTS--output=$OUTPUT_FILE "

    ./$BINARY $ARGUMENTS
    if [[ $? != 0 ]]; then
        echo "Error in running ./${BINARY} ${ARGUMENTS}"
    fi

    echo "--------------------------------------"
}

ALL_GOAL_BIASES=(0.05)
ALL_SAMPLES=(1000)
ALL_THRESHOLDS=(0.25 2.0)


for INPUT_FILE in "$MAPS_DIRECTORY"/*; do
    for BIAS in "${ALL_GOAL_BIASES[@]}"; do
        for SAMPLE in "${ALL_SAMPLES[@]}"; do
            for THRESHOLD in "${ALL_THRESHOLDS[@]}"; do
                execute "$INPUT_FILE" "$BIAS" "$SAMPLE" "$THRESHOLD" &
            done
        done
    done
done

wait

echo "**************************************"

exit 0
