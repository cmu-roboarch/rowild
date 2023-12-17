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

BINARY="arm2d.out"
MAPS_DIRECTORY="./input-maps"
CFGS_DIRECTORY="./input-cfgs"
OUTPUT_DIRECTORY="./output-paths"

if [[ ! -f $BINARY || ! -d $MAPS_DIRECTORY || ! -d $CFGS_DIRECTORY ]]; then
    echo "Missing required files!"
    echo "Binary: $BINARY ?"
    echo "Input maps: $MAPS_DIRECTORY ?"
    echo "Config files: $CFGS_DIRECTORY ?"
    exit 1
fi

execute() {
    local INPUT_FILE=$1
    local PLANNER=$2
    local EPSILON=$3
    local RADIUS=$4
    local SAMPLE=$5
    local WEIGHT=$6
    local BIAS=$7
    local ITERS=$8

    local MAP_NAME=$(basename "$INPUT_FILE" .txt)
    local CFG_FILE="${CFGS_DIRECTORY}/${MAP_NAME}_cfgs.txt"

    mkdir -p "$OUTPUT_DIRECTORY"
    local OUTPUT_FILE="$OUTPUT_DIRECTORY/"
    OUTPUT_FILE="${OUTPUT_FILE}planner-${PLANNER}_"
    OUTPUT_FILE="${OUTPUT_FILE}map-${MAP_NAME}_"
    OUTPUT_FILE="${OUTPUT_FILE}eps-${EPSILON}_"
    OUTPUT_FILE="${OUTPUT_FILE}r-${RADIUS}_"
    OUTPUT_FILE="${OUTPUT_FILE}s-${SAMPLE}_"
    OUTPUT_FILE="${OUTPUT_FILE}w-${WEIGHT}_"
    OUTPUT_FILE="${OUTPUT_FILE}b-${BIAS}_"
    OUTPUT_FILE="${OUTPUT_FILE}i-${ITERS}"
    OUTPUT_FILE="${OUTPUT_FILE}.txt"

    local ARGUMENTS=""
    ARGUMENTS="$ARGUMENTS--map=$INPUT_FILE "
    ARGUMENTS="$ARGUMENTS--config=$CFG_FILE "
    ARGUMENTS="$ARGUMENTS--planner=$PLANNER "
    ARGUMENTS="$ARGUMENTS--epsilon=$EPSILON "
    ARGUMENTS="$ARGUMENTS--radius=$RADIUS "
    ARGUMENTS="$ARGUMENTS--samples=$SAMPLE "
    ARGUMENTS="$ARGUMENTS--weight=$WEIGHT "
    ARGUMENTS="$ARGUMENTS--bias=$BIAS "
    ARGUMENTS="$ARGUMENTS--iterations=$ITERS "
    ARGUMENTS="$ARGUMENTS--output=$OUTPUT_FILE "

    ./$BINARY $ARGUMENTS
    if [[ $? != 0 ]]; then
        echo "Error in running ./${BINARY} ${ARGUMENTS}"
    fi

    echo "--------------------------------------"
}

ALL_PLANNERS=("PRM" "RRT" "RRTStar" "RRTPostProc")
ALL_EPSILONS=(0.21)
ALL_RADIUSES=(2.1)
ALL_SAMPLES=(10000)
ALL_ASTAR_WEIGHTS=(1 1000)
ALL_GOAL_BIASES=(0.05)
ALL_PP_ITERS=(100)

for INPUT_FILE in "$MAPS_DIRECTORY"/*; do
    for PLANNER in "${ALL_PLANNERS[@]}"; do
        for EPSILON in "${ALL_EPSILONS[@]}"; do
            for RADIUS in "${ALL_RADIUSES[@]}"; do
                for SAMPLE in "${ALL_SAMPLES[@]}"; do
                    for WEIGHT in "${ALL_ASTAR_WEIGHTS[@]}"; do
                        for BIAS in "${ALL_GOAL_BIASES[@]}"; do
                            for ITERS in "${ALL_PP_ITERS[@]}"; do
                                execute "$INPUT_FILE" "$PLANNER" "$EPSILON" "$RADIUS" "$SAMPLE" "$WEIGHT" "$BIAS" "$ITERS" &
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
