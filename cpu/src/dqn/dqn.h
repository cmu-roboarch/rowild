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

#pragma once

#include "env.h"
#include <iostream>
#include <vector>

class DQN {
  public:
    DQN(double _eps, double _discount, double _lRate, double stateSize)
        : epsilon(_eps), discountFactor(_discount), learningRate(_lRate) {
        this->model = new SimpleLinearModel(stateSize);
    }

    Action selectAction(const std::vector<double> &state) {
        if (((double)rand() / (RAND_MAX)) < epsilon) {
            return static_cast<Action>(rand() % ACTION_COUNT);
        } else {
            double maxQValue = -1e9;
            Action bestAction = MOVE_FORWARD;
            for (int action = MOVE_FORWARD; action < ACTION_COUNT; action++) {
                double qValue =
                    model->forward(state, static_cast<Action>(action));
                if (qValue > maxQValue) {
                    maxQValue = qValue;
                    bestAction = static_cast<Action>(action);
                }
            }
            return bestAction;
        }
    }

    void train(const std::vector<double> &state, Action action, double reward,
               const std::vector<double> &nextState) {
        double target = reward + discountFactor * maxQValue(nextState);
        double prediction = model->forward(state, action);

        double loss = (target - prediction) * (target - prediction);
        model->updateWeights(loss, state, action);
        epsilon *= 0.995; // Decaying epsilon
    }

  private:
    double maxQValue(const std::vector<double> &state) {
        double maxQ = -1e9;
        for (int action = MOVE_FORWARD; action < ACTION_COUNT; action++) {
            double qValue = model->forward(state, static_cast<Action>(action));
            if (qValue > maxQ) maxQ = qValue;
        }
        return maxQ;
    }

    double epsilon, discountFactor, learningRate;
    SimpleLinearModel *model;
};
