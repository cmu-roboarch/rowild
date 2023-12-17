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


# PARALLEL_MODE=false
PARALLEL_MODE=true

if [[ $# -eq 0 ]]; then
    ARG=""
elif [[ $# -eq 1 ]]; then
    ARG=$1
else
    echo "Usage: $0 [benchmark]"
    exit 1
fi


INIT_PATH=$( pwd )

ALL_BENCHMARKS=(
"DeliBot"
"PatrolBot"
"MoveBot"
"HomeBot"
"FlyBot"
"CarriBot"
)

ALL_PATHS=(
"DeliBot"
"PatrolBot"
"MoveBot"
"HomeBot"
"FlyBot"
"CarriBot"
)


if [[ "${#ALL_BENCHMARKS[@]}" != "${#ALL_PATHS[@]}" ]]; then
    echo "Inconsistent benchmark names/paths"
    exit 1
fi

MATCHING_IDXS=()

for i in "${!ALL_BENCHMARKS[@]}"; do
    BENCH_NAME="${ALL_BENCHMARKS[$i]}"
    BENCH_PATH="${ALL_PATHS[$i]}"
    if [[ "$BENCH_NAME" =~ $ARG ]] || [[ "$BENCH_PATH" =~ $ARG ]]; then
        MATCHING_IDXS+=( "$i" )
    fi
done

ROOT="$( readlink -f "$( dirname "${BASH_SOURCE[0]}" )" )"

PIDS=()
SUCCESS=true

for IDX in "${MATCHING_IDXS[@]}"; do
    BENCH_NAME="${ALL_BENCHMARKS[$IDX]}"
    BENCH_PATH="${ROOT}/${ALL_PATHS[$IDX]}"

    if $PARALLEL_MODE; then
        (
            cd "$BENCH_PATH" || exit 1
            [[ -f "Makefile" ]] || exit 1

            rm build.log > /dev/null 2>&1
            if make clean > /dev/null 2>&1; then
                exit 0
            else
                exit 1
            fi
        ) &
        PIDS+=($!)
    else
        echo "Cleaning $BENCH_PATH"

        cd "$BENCH_PATH" || { echo "Could not cd to $BENCH_PATH"; continue; }
        if [[ ! -f "Makefile" ]]; then
            echo "Could not find Makefile in $BENCH_PATH"
        fi

        rm build.log > /dev/null 2>&1
        make clean > /dev/null 2>&1 || echo "Error in cleaning $BENCH_PATH"

        cd - > /dev/null || true
    fi
done

if $PARALLEL_MODE; then
    for PID in "${PIDS[@]}"; do
        wait $PID || SUCCESS=false
    done

    if $SUCCESS; then
        echo "All cleanups completed successfully."
    else
        echo "One or more cleanups failed."
    fi
fi

cd "$INIT_PATH" || true

exit 0
