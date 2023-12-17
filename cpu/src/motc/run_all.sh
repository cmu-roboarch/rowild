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

BINARY="motc.out"
INPUT_DIRECTORY="./input-spcourses"
OUTPUT_DIRECTORY="./output-trajs"

if [[ ! -f $BINARY || ! -d $INPUT_DIRECTORY ]]; then
    echo "Missing required files!"
    echo "Binary: $BINARY ?"
    echo "Input directory: $INPUT_DIRECTORY ?"
    exit 1
fi

execute() {
    local SP_FILE=$1
    local TIME=$2
    local TICK=$3
    local DIST=$4
    local ITER=$5
    local NEIGHBORS=$6
    local SSPEED=$7
    local TSPEED=$8

    local SP_NAME=$(basename "$SP_FILE" .txt)

    mkdir -p "$OUTPUT_DIRECTORY"
    local OUTPUT_FILE="$OUTPUT_DIRECTORY/"
    OUTPUT_FILE="${OUTPUT_FILE}inp-${SP_NAME}_"
    OUTPUT_FILE="${OUTPUT_FILE}time-${TIME}_"
    OUTPUT_FILE="${OUTPUT_FILE}tick-${TICK}_"
    OUTPUT_FILE="${OUTPUT_FILE}dist-${DIST}_"
    OUTPUT_FILE="${OUTPUT_FILE}iter-${ITER}_"
    OUTPUT_FILE="${OUTPUT_FILE}n-${NEIGHBORS}_"
    OUTPUT_FILE="${OUTPUT_FILE}ss-${SSPEED}_"
    OUTPUT_FILE="${OUTPUT_FILE}ts-${TSPEED}"
    OUTPUT_FILE="${OUTPUT_FILE}.txt"

    local ARGUMENTS=""
    ARGUMENTS="$ARGUMENTS--input=$SP_FILE "
    ARGUMENTS="$ARGUMENTS--time=$TIME "
    ARGUMENTS="$ARGUMENTS--tick=$TICK "
    ARGUMENTS="$ARGUMENTS--dist=$DIST "
    ARGUMENTS="$ARGUMENTS--iters=$ITER "
    ARGUMENTS="$ARGUMENTS--neighbors=$NEIGHBORS "
    ARGUMENTS="$ARGUMENTS--sspeed=$SSPEED "
    ARGUMENTS="$ARGUMENTS--tspeed=$TSPEED "
    ARGUMENTS="$ARGUMENTS--output=$OUTPUT_FILE "

    ./$BINARY $ARGUMENTS
    if [[ $? != 0 ]]; then
        echo "Error in running ./${BINARY} ${ARGUMENTS}"
    fi

    echo "--------------------------------------"
}

ALL_TIMES=(100 500)
ALL_TICKS=(0.2 1.0)
ALL_DISTS=(1.5 5)
ALL_ITERS=(3 10)
ALL_NEIGHBORS=(10 50)
ALL_SSPEEDS=(0.1389 0.2778)
ALL_TSPEEDS=(2.7778 5.5556)

for SP_FILE in "$INPUT_DIRECTORY"/*; do
    for TIME in "${ALL_TIMES[@]}"; do
        for TICK in "${ALL_TICKS[@]}"; do
            for DIST in "${ALL_DISTS[@]}"; do
                for ITER in "${ALL_ITERS[@]}"; do
                    for NEIGHBORS in "${ALL_NEIGHBORS[@]}"; do
                        for SSPEED in "${ALL_SSPEEDS[@]}"; do
                            for TSPEED in "${ALL_TSPEEDS[@]}"; do
                                execute "$SP_FILE" "$TIME" "$TICK" "$DIST" "$ITER" "$NEIGHBORS" "$SSPEED" "$TSPEED" &
                            done
                        done
                    done
                done
            done
        done
    done
done

wait

echo "**************************************"

exit 0
