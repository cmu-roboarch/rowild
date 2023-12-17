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

BINARY="pid.out"
INPUT_DIRECTORY="./input-logs"
OUTPUT_DIRECTORY="./output-logs"

if [[ ! -f $BINARY || ! -d $INPUT_DIRECTORY ]]; then
    echo "Missing required files!"
    echo "Binary: $BINARY ?"
    echo "Input directory: $INPUT_DIRECTORY ?"
    exit 1
fi

execute() {
    local LOG_FILE=$1
    local KP=$2
    local KI=$3
    local KD=$4

    local LOG_NAME=$(basename "$LOG_FILE" .txt)

    mkdir -p "$OUTPUT_DIRECTORY"
    local OUTPUT_FILE="$OUTPUT_DIRECTORY/"
    OUTPUT_FILE="${OUTPUT_FILE}log-${LOG_NAME}_"
    OUTPUT_FILE="${OUTPUT_FILE}kp-${KP}_"
    OUTPUT_FILE="${OUTPUT_FILE}ki-${KI}_"
    OUTPUT_FILE="${OUTPUT_FILE}kd-${KD}"
    OUTPUT_FILE="${OUTPUT_FILE}.txt"

    local ARGUMENTS=""
    ARGUMENTS="$ARGUMENTS--log=$LOG_FILE "
    ARGUMENTS="$ARGUMENTS--kp=$KP "
    ARGUMENTS="$ARGUMENTS--ki=$KI "
    ARGUMENTS="$ARGUMENTS--kd=$KD "
    ARGUMENTS="$ARGUMENTS--output=$OUTPUT_FILE "

    ./$BINARY $ARGUMENTS
    if [[ $? != 0 ]]; then
        echo "Error in running ./${BINARY} ${ARGUMENTS}"
    fi

    echo "--------------------------------------"
}

ALL_KPS=(1.0)
ALL_KIS=(0.1)
ALL_KDS=(0.01)

for LOG_FILE in "$INPUT_DIRECTORY"/*.txt; do
    for KP in "${ALL_KPS[@]}"; do
        for KI in "${ALL_KIS[@]}"; do
            for KD in "${ALL_KDS[@]}"; do
                execute "$LOG_FILE" "$KP" "$KI" "$KD" &
            done
        done
    done
done

wait

echo "**************************************"

exit 0
