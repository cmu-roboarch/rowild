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

#include "args.h"
#include "dqn.h"
#include "env.h"
#include "log.h"
#include "zsim_hooks.h"
#include <fstream>

int main(int argc, const char **argv) {
    using args::KVArg;
    using args::Parser;

    Parser parser(argv[0], argc, argv, false);
    KVArg<int> episodesArg(parser, "episodes", "",
                           "The number of planning episodes");
    KVArg<double> epsilonArg(parser, "epsilon", "", "DQN epsilon");
    KVArg<double> discountArg(parser, "discount", "", "DQN discount factor");
    KVArg<double> lRateArg(parser, "lrate", "", "DQN learning rate");
    KVArg<int> stateSizeArg(parser, "size", "", "Environment state size");
    KVArg<std::string> outArg(parser, "output", "", "Output log file");

    if (!parser.parse()) assert(false);

    int episodes = episodesArg.found() ? episodesArg.value() : 1000;
    double dqnEpsilon = epsilonArg.found() ? epsilonArg.value() : 1.0;
    double dqnDiscountFactor = discountArg.found() ? discountArg.value() : 0.99;
    double dqnLearningRate = lRateArg.found() ? lRateArg.value() : 0.001;
    int stateSize = stateSizeArg.found() ? stateSizeArg.value() : 10;
    std::string outputFile = outArg.found() ? outArg.value() : "/dev/null";

    std::vector<int> actionLog;
    DQN *agent =
        new DQN(dqnEpsilon, dqnDiscountFactor, dqnLearningRate, stateSize);
    SimpleEnvironment *env = new SimpleEnvironment(stateSize);

    // ROI begins
    zsim_roi_begin();

    for (int episode = 1; episode <= episodes; episode++) {
        auto state = env->getState();
        for (int t = 0; t < 100; t++) {
            Action action = agent->selectAction(state);
            double reward = env->step(action);
            auto nextState = env->getState();
            agent->train(state, action, reward, nextState);
            state = nextState;

            actionLog.push_back(action);
        }
    }

    zsim_roi_end();
    // ROI ends

    // Write the output trajectory
    std::ofstream outLog;
    outLog.open(outputFile);
    for (double a : actionLog) {
        outLog << a << std::endl;
        ;
    }
    outLog.close();

    return 0;
}
