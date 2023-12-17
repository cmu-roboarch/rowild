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

BINARY="movtop.out"
INPUT_DIRECTORY="./input-data"
OUTPUT_DIRECTORY="./output-trajs"

if [[ ! -f $BINARY || ! -d $INPUT_DIRECTORY ]]; then
    echo "Missing required files!"
    echo "Binary: $BINARY ?"
    echo "Input directory: $INPUT_DIRECTORY ?"
    exit 1
fi

execute() {
    local LOG_FILE=$1
    local DT=$2
    local THRESHOLD=$3
    local MAX_LIN_SPEED=$4
    local MAX_ANG_SPEED=$5

    local TRAJ_NAME=$(basename "$LOG_FILE" .txt)

    mkdir -p "$OUTPUT_DIRECTORY"
    local OUTPUT_FILE="$OUTPUT_DIRECTORY/"
    OUTPUT_FILE="${OUTPUT_FILE}traj-${TRAJ_NAME}_"
    OUTPUT_FILE="${OUTPUT_FILE}dt-${DT}_"
    OUTPUT_FILE="${OUTPUT_FILE}thresh-${THRESHOLD}_"
    OUTPUT_FILE="${OUTPUT_FILE}maxlinspd-${MAX_LIN_SPEED}_"
    OUTPUT_FILE="${OUTPUT_FILE}maxangspd-${MAX_ANG_SPEED}"
    OUTPUT_FILE="${OUTPUT_FILE}.txt"

    local ARGUMENTS=""
    ARGUMENTS="$ARGUMENTS--log=$LOG_FILE "
    ARGUMENTS="$ARGUMENTS--dt=$DT "
    ARGUMENTS="$ARGUMENTS--threshold=$THRESHOLD "
    ARGUMENTS="$ARGUMENTS--max-lin-speed=$MAX_LIN_SPEED "
    ARGUMENTS="$ARGUMENTS--max-ang-speed=$MAX_ANG_SPEED "
    ARGUMENTS="$ARGUMENTS--output=$OUTPUT_FILE "

    ./$BINARY $ARGUMENTS
    if [[ $? != 0 ]]; then
        echo "Error in running ./${BINARY} ${ARGUMENTS}"
    fi

    echo "--------------------------------------"
}

ALL_DTS=(0.01 0.1)
ALL_THRESHOLDS=(0.001 0.01 0.1)
ALL_MAX_LIN_SPEEDS=(10 15)
ALL_MAX_ANG_SPEEDS=(7 5)

for TRAJ_FILE in "$INPUT_DIRECTORY"/*; do
    for DT in "${ALL_DTS[@]}"; do
        for THRESHOLD in "${ALL_THRESHOLDS[@]}"; do
            for MAX_LIN_SPEED in "${ALL_MAX_LIN_SPEEDS[@]}"; do
                for MAX_ANG_SPEED in "${ALL_MAX_ANG_SPEEDS[@]}"; do
                    execute "$TRAJ_FILE" "$DT" "$THRESHOLD" "$MAX_LIN_SPEED" "$MAX_ANG_SPEED" &
                done
            done
        done
    done
done

wait

echo "**************************************"

exit 0
