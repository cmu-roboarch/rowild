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

BINARY="patrolbot.out"
MOBILENET_INPUT="./mobilenet-data"
IMAGES_DIR="./cmu-tour"
SENSOR_INPUT="./sensor-logs"
PATH_INPUT="./path-logs"
OUTPUT_DIRECTORY="./output-logs"

LABELS_FILE="${MOBILENET_INPUT}/object_detection_classes_coco.txt"
MODEL_FILE="${MOBILENET_INPUT}/frozen_inference_graph.pb"
CFG_FILE="${MOBILENET_INPUT}/ssd_mobilenet_v2_coco_2018_03_29.pbtxt.txt"


if [[ ! -f $BINARY || ! -d $MOBILENET_INPUT || ! -d $IMAGES_DIR || ! -d $SENSOR_INPUT ]]; then
    echo "Missing required files!"
    echo "Binary: $BINARY ?"
    echo "Input data/mobilenet: $MOBILENET_INPUT ?"
    echo "Input data/images: $IMAGES_DIR ?"
    echo "Input data/sensor: $SENSOR_INPUT ?"
    exit 1
fi

execute() {
    local PATH_LOG=$1
    local SENSOR_LOG=$2
    local SCALE=$3
    local CONFIDENCE=$4

    local SENSOR_LOGNAME=$(basename "$SENSOR_LOG" .txt)
    local PATH_NAME=$(basename "$PATH_LOG" .txt)

    mkdir -p "$OUTPUT_DIRECTORY"
    local OUTPUT_FILE="$OUTPUT_DIRECTORY/"
    OUTPUT_FILE="${OUTPUT_FILE}sensor-${SENSOR_LOGNAME}_"
    OUTPUT_FILE="${OUTPUT_FILE}path-${PATH_NAME}"
    OUTPUT_FILE="${OUTPUT_FILE}.txt"

    local ARGUMENTS=""
    ARGUMENTS="$ARGUMENTS--cfg=$CFG_FILE "
    ARGUMENTS="$ARGUMENTS--path=$PATH_LOG "
    ARGUMENTS="$ARGUMENTS--imgdir=$IMAGES_DIR "
    ARGUMENTS="$ARGUMENTS--labels=$LABELS_FILE "
    ARGUMENTS="$ARGUMENTS--model=$MODEL_FILE "
    ARGUMENTS="$ARGUMENTS--scale=$SCALE "
    ARGUMENTS="$ARGUMENTS--confidence=$CONFIDENCE "
    ARGUMENTS="$ARGUMENTS--log=$SENSOR_LOG "
    ARGUMENTS="$ARGUMENTS--output=$OUTPUT_FILE "

    ./$BINARY $ARGUMENTS
    if [[ $? != 0 ]]; then
        echo "Error in running ./${BINARY} ${ARGUMENTS}"
    fi

    echo "--------------------------------------"
}

ALL_SCALES=(0.01 0.1 0.5 0.8 1)
ALL_CONFIDENCES=(0.4)

for PATH_LOG in "$PATH_INPUT"/*; do
    for SENSOR_LOG in "$SENSOR_INPUT"/*; do
        for SCALE in "${ALL_SCALES[@]}"; do
            for CONFIDENCE in "${ALL_CONFIDENCES[@]}"; do
                execute "$PATH_LOG" "$SENSOR_LOG" "$SCALE" "$CONFIDENCE" &
            done
        done
    done
done

wait

echo "**************************************"

exit 0
