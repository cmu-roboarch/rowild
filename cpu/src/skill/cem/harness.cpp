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
#include "log.h"
#include "rowild_utils.h"
#include "zsim_hooks.h"
#include <algorithm>
#include <fstream>
#include <string>
#include <vector>

typedef std::vector<double> PARAM;

std::vector<PARAM> parameters;
std::vector<double> positions;
std::default_random_engine generator;

static std::vector<double> sample(const PARAM &mu, const PARAM &cov) {
    /*
     * This is not an accurate sampling from multivariate normal (MVN)
     * distribution. It considers the variables are *independent*, which is not
     * the case always.
     * */
    assert(mu.size() == cov.size());

    std::vector<double> r;
    for (int i = 0; i < static_cast<int>(mu.size()); i++) {
        std::normal_distribution<double> distribution(mu[i], cov[i]);
        r.push_back(distribution(generator));
    }

    return r;
}

void readSimulatorLog(std::string simLogFile) {
    std::ifstream logFile;
    logFile.open(simLogFile);
    assert(logFile.good());

    std::string line;
    std::size_t pos;

    std::getline(logFile, line);
    assert(line == "Parameters Position");

    while (std::getline(logFile, line)) {
        pos = line.find(']');
        assert(pos != std::string::npos);

        std::string paramStr = line.substr(1, pos);
        std::stringstream paramSS(paramStr);
        PARAM params;
        double param;
        while (paramSS >> param) {
            params.push_back(param);
        }
        parameters.push_back(params);

        std::string posStr = line.substr(pos + 2);
        std::stringstream posSS(posStr);
        double position;
        posSS >> position;
        positions.push_back(position);
    }
    logFile.close();

    // Sanity check
    assert(parameters.size() == positions.size());
    for (int i = 1; i < static_cast<int>(parameters.size()); i++) {
        assert(parameters[i].size() == parameters[0].size());
    }
}

double getPosition(const PARAM &param) {
    int minIdx = 0;
    double minCost = getManhattanDistance<PARAM>(param, parameters[0]);

    for (int i = 1; i < static_cast<int>(parameters.size()); i++) {
        double cost = getManhattanDistance<PARAM>(param, parameters[i]);
        if (cost < minCost) {
            minIdx = i;
            minCost = cost;
        }
    }

    return positions[minIdx];
}

int main(int argc, const char **argv) {
    using args::KVArg;
    using args::Parser;

    Parser parser(argv[0], argc, argv, false);
    KVArg<std::string> simulatorArg(parser, "simulator", "", "Simulator log");
    KVArg<int> updatesArg(parser, "updates", "", "Number of policy updates");
    KVArg<int> samplesArg(parser, "samples", "", "Number of samples");
    KVArg<int> topnArg(parser, "topn", "", "Top-N chosen for policy update");
    KVArg<std::string> outputArg(parser, "output", "", "Policies log");

    if (!parser.parse()) assert(false);

    assert_msg(simulatorArg.found(), "Simulator log file is not provided");

    std::string simLogFile = simulatorArg.value();
    int numUpdates = updatesArg.found() ? updatesArg.value() : 5;
    int numSamples = samplesArg.found() ? samplesArg.value() : 15;
    int topN = topnArg.found() ? topnArg.value() : 5;
    std::string outputFile =
        outputArg.found() ? outputArg.value() : "/dev/null";

    assert(topN >= 0 && topN <= numSamples);

    // Read simulator log
    readSimulatorLog(simLogFile);

    const double goalPosition = -2;
    auto getReward = [goalPosition](double p) {
        return 2.0 - std::abs(p - goalPosition);
    };

    std::vector<PARAM> policiesLog;

    // Initialize policy
    std::vector<double> policyMu = {-5.0, -5.0, 0.0, 0.0};
    std::vector<double> policyCov = {0.1, 0.1, 1.0, 1.0};

    // ROI begins
    zsim_roi_begin();

    for (int u = 0; u < numUpdates; u++) {
        std::vector<PARAM> skillParams;
        std::vector<double> rewards;
        skillParams.resize(numSamples);
        rewards.resize(numSamples);

        policiesLog.push_back(policyMu);

        // [Verification] Don't expect to get the same results with different
        // number of threads; different random numbers change the execution
        // results
        // #pragma omp parallel for num_threads(4)
        for (int s = 0; s < numSamples; s++) {
            auto sampledParams = sample(policyMu, policyCov);

            // Exclude from ROI: the following function
            // *fakes* the actual rollout.  This is
            // supposed to happen in the real world, and
            // its latency heavily depends on the
            // mechanics, not computation.
            zsim_roi_end();
            auto projectedPos = getPosition(sampledParams);
            zsim_roi_begin();

            auto reward = getReward(projectedPos);

            skillParams[s] = sampledParams;
            rewards[s] = reward;
        }

        std::vector<double> sortedRewards(rewards);
        sort(sortedRewards.begin(), sortedRewards.end());

        double rewardThreshold = sortedRewards[numSamples - topN];
        std::vector<int> chosenIndices;
        for (int i = 0; i < numSamples; i++) {
            if (rewards[i] >= rewardThreshold) {
                chosenIndices.push_back(i);
                if (static_cast<int>(chosenIndices.size()) == topN) break;
            }
        }

        assert(static_cast<int>(chosenIndices.size()) == topN);

// Update policy
#pragma omp parallel for num_threads(2)
        for (int i = 0; i < static_cast<int>(policyMu.size()); i++) {
            // Update mean
            double sum = policyMu[i];
            for (auto idx : chosenIndices) {
                assert(idx < static_cast<int>(skillParams.size()));
                sum += skillParams[idx][i];
            }
            double mean = sum / (1 + topN);

            // Update variance
            double var = (policyMu[i] - mean) * (policyMu[i] - mean);
            for (auto idx : chosenIndices) {
                var +=
                    (skillParams[idx][i] - mean) * (skillParams[idx][i] - mean);
            }
            var /= (1 + topN);

            policyMu[i] = mean;
            policyCov[i] = var;
        }
    }

    zsim_roi_end();
    // ROI ends

    // Write the output log
    std::ofstream outLogFile;
    outLogFile.open(outputFile);
    for (auto policy : policiesLog) {
        for (auto param : policy) {
            outLogFile << param << " ";
        }
        outLogFile << std::endl;
    }
    outLogFile.close();

    return 0;
}
