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

BINARY="imgdnn.out"
INPUT_DIRECTORY="./input-data"
OUTPUT_DIRECTORY="./output-images"

LABELS_FILE="${INPUT_DIRECTORY}/object_detection_classes_coco.txt"
MODEL_FILE="${INPUT_DIRECTORY}/frozen_inference_graph.pb"
CFG_FILE="${INPUT_DIRECTORY}/ssd_mobilenet_v2_coco_2018_03_29.pbtxt.txt"


if [[ ! -f $BINARY || ! -d $INPUT_DIRECTORY ]]; then
    echo "Missing required files!"
    echo "Binary: $BINARY ?"
    echo "Input data: $INPUT_DIRECTORY ?"
    exit 1
fi

execute() {
    local INPUT_IMAGE=$1
    local SCALE=$2
    local CONFIDENCE=$3

    local IMAGE_NAME=$(basename "$INPUT_IMAGE" .jpg)

    mkdir -p "$OUTPUT_DIRECTORY"
    local OUTPUT_FILE="$OUTPUT_DIRECTORY/"
    OUTPUT_FILE="${OUTPUT_FILE}img-${IMAGE_NAME}_"
    OUTPUT_FILE="${OUTPUT_FILE}s-${SCALE}_"
    OUTPUT_FILE="${OUTPUT_FILE}c-${CONFIDENCE}"
    OUTPUT_FILE="${OUTPUT_FILE}.jpg"

    local ARGUMENTS=""
    ARGUMENTS="$ARGUMENTS--labels=$LABELS_FILE "
    ARGUMENTS="$ARGUMENTS--model=$MODEL_FILE "
    ARGUMENTS="$ARGUMENTS--cfg=$CFG_FILE "
    ARGUMENTS="$ARGUMENTS--img=$INPUT_IMAGE "
    ARGUMENTS="$ARGUMENTS--scale=$SCALE "
    ARGUMENTS="$ARGUMENTS--confidence=$CONFIDENCE "
    ARGUMENTS="$ARGUMENTS--output=$OUTPUT_FILE "

    ./$BINARY $ARGUMENTS
    if [[ $? != 0 ]]; then
        echo "Error in running ./${BINARY} ${ARGUMENTS}"
    fi

    echo "--------------------------------------"
}

ALL_SCALES=(0.01 0.1 0.5 0.8 1)
ALL_CONFIDENCES=(0.4)

for INPUT_IMAGE in "$INPUT_DIRECTORY"/*.jpg; do
    for SCALE in "${ALL_SCALES[@]}"; do
        for CONFIDENCE in "${ALL_CONFIDENCES[@]}"; do
            execute "$INPUT_IMAGE" "$SCALE" "$CONFIDENCE"
        done
    done
done

wait

echo "**************************************"

exit 0
