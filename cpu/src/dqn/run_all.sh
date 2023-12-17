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

BINARY="dqn.out"
OUTPUT_DIRECTORY="./output-logs"

if [[ ! -f $BINARY ]]; then
    echo "Missing required files!"
    echo "Binary: $BINARY ?"
    exit 1
fi

execute() {
    local EPISODES=$1
    local EPSILON=$2
    local DISCOUNT=$3
    local LRATE=$4
    local STATESIZE=$5

    mkdir -p "$OUTPUT_DIRECTORY"
    local OUTPUT_FILE="$OUTPUT_DIRECTORY/"
    OUTPUT_FILE="${OUTPUT_FILE}episodes-${EPISODES}_"
    OUTPUT_FILE="${OUTPUT_FILE}eps-${EPSILON}_"
    OUTPUT_FILE="${OUTPUT_FILE}disc-${DISCOUNT}_"
    OUTPUT_FILE="${OUTPUT_FILE}lrate-${LRATE}_"
    OUTPUT_FILE="${OUTPUT_FILE}ssize-${STATESIZE}"
    OUTPUT_FILE="${OUTPUT_FILE}.txt"

    local ARGUMENTS=""
    ARGUMENTS="$ARGUMENTS--episodes=$EPISODES "
    ARGUMENTS="$ARGUMENTS--epsilon=$EPSILON "
    ARGUMENTS="$ARGUMENTS--discount=$DISCOUNT "
    ARGUMENTS="$ARGUMENTS--lrate=$LRATE "
    ARGUMENTS="$ARGUMENTS--size=$STATESIZE "
    ARGUMENTS="$ARGUMENTS--output=$OUTPUT_FILE "

    ./$BINARY $ARGUMENTS
    if [[ $? != 0 ]]; then
        echo "Error in running ./${BINARY} ${ARGUMENTS}"
    fi

    echo "--------------------------------------"
}

ALL_EPISODES=(1000)
ALL_EPSILONS=(1.0)
ALL_DISCOUNTS=(0.99)
ALL_LEARNINGRATES=(0.001)
ALL_STATESIZES=(10)

for EPISODES in "${ALL_EPISODES[@]}"; do
    for EPSILON in "${ALL_EPSILONS[@]}"; do
        for DISCOUNT in "${ALL_DISCOUNTS[@]}"; do
            for LRATE in "${ALL_LEARNINGRATES[@]}"; do
                for STATESIZE in "${ALL_STATESIZES[@]}"; do
                    execute "$EPISODES" "$EPSILON" "$DISCOUNT" "$LRATE" "$STATESIZE" &
                done
            done
        done
    done
done

wait

echo "**************************************"

exit 0
