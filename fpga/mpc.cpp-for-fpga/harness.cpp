/*
 * MIT License
 *
 * Copyright (c) 2023 Carnegie Mellon University
 *
 * This file is part of RoWild.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#include <math.h>

int main() {

    static constexpr const int n = 10;
    float dt = 0.1;
    const float gGravity = 9.81;
    float goal[3] = {10.0, 10.0, 10.0};
    float initialState[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float controls[1024 * 3];

    for (int idx = 0; idx < 1024; idx++) {
        float state[6];
        for (int i = 0; i < 6; i++) state[i] = initialState[i];
        float controlSequence[3 * n];

        for (int i = 0; i < n; i++) {
            controlSequence[i] = controls[3 * idx];
        }

        float cost = 0.0;
        for (int i = 0; i < n; i++) {
            state[0] += dt * state[3];
            state[1] += dt * state[4];
            state[2] += dt * state[5];
            state[3] += dt * (controlSequence[0] - gGravity);
            state[4] += dt * controlSequence[1];
            state[5] += dt * controlSequence[2];

            cost += sqrt((goal[0] - state[0]) * (goal[0] - state[0]) + (goal[1] - state[1]) * (goal[1] - state[1]) + (goal[2] - state[2]) * (goal[2] - state[2]));
        }

        float alpha = 0.01;
        for (int iter = 0; iter < 100; iter++) {
            for (int i = 0; i < n; i++) {
                for (int dim = 0; dim < 3; dim++) {
                    controlSequence[3 * i + dim] -= alpha * (cost / n);
                }
            }
        }
    }

    return 0;
}
