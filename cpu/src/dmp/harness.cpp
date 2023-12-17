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
#include "zsim_hooks.h"
#include <fstream>
#include <math.h>
#include <string>
#include <vector>

std::vector<double> Time, Q, Qd, Qdd;
std::vector<double> weights;

void loadTrajectory(std::string inputFile) {
    std::ifstream inTrajFile;
    inTrajFile.open(inputFile);
    assert(inTrajFile.good());

    std::string line;
    std::getline(inTrajFile, line);
    assert(line == "Time Q Qd Qdd");

    double t, q, qd, qdd;
    while (inTrajFile >> t >> q >> qd >> qdd) {
        Time.push_back(t);
        Q.push_back(q);
        Qd.push_back(qd);
        Qdd.push_back(qdd);
    }
    inTrajFile.close();
}

void loadWeights(std::string inputFile) {
    std::ifstream weightsFile;
    weightsFile.open(inputFile);
    assert(weightsFile.good());

    double w;
    while (weightsFile >> w)
        weights.push_back(w);
    weightsFile.close();
}

int main(int argc, const char **argv) {
    using args::KVArg;
    using args::Parser;

    Parser parser(argv[0], argc, argv, false);
    KVArg<std::string> trajArg(parser, "trajectory", "",
                               "Input trajectory file");
    KVArg<std::string> weightsArg(parser, "weights", "", "Learned DMP weights");
    KVArg<double> dtArg(parser, "dt", "", "Time step");
    KVArg<std::string> outputArg(parser, "output", "",
                                 "Output trajectory file");

    if (!parser.parse()) assert(false);

    assert_msg(trajArg.found(), "Input file is not provided");
    assert_msg(weightsArg.found(), "Weights file is not provided");

    std::string trajFile = trajArg.value();
    std::string weightsFile = weightsArg.value();
    double dt = dtArg.found() ? dtArg.value() : 0.001;
    std::string outputFile =
        outputArg.found() ? outputArg.value() : "/dev/null";

    loadTrajectory(trajFile);
    loadWeights(weightsFile);

    std::vector<double> Traj, execTime;
    double q0 = 0;          // Initial
    double g = 1;           // Goal
    double T = Time.back(); // Trajectory time

    double q = q0;
    double qd = 0, qdd;
    double t = 0;

    int nrBasis = static_cast<int>(weights.size());
    double K = 25.0 * 25 / 4; // Virtual spring coefficient
    double B = 25.0;          // Virtual damper coefficient

    std::vector<double> C; // Basis function centers
    for (int i = 0; i < nrBasis; i++) {
        C.push_back(static_cast<double>(i) / (nrBasis - 1));
    }

    // Basis function widths
    double H = 0.65 * (1.0 / (nrBasis - 1.0)) * (1.0 / (nrBasis - 1.0));

    // ROI begins
    zsim_roi_begin();

    while (t <= T) {
        t += dt;

        // Compute the basis function values and force term
        double sum = 0;
        std::vector<double> Phi;
        for (auto c : C) {
            double p = exp(-0.5 * ((t / T - c) * (t / T - c) / H));
            Phi.push_back(p);

            sum += p;
        }

        for (int i = 0; i < static_cast<int>(Phi.size()); i++) {
            Phi[i] /= sum;
        }

        double f = 0;
        assert(Phi.size() == weights.size());
        for (int i = 0; i < static_cast<int>(weights.size()); i++) {
            f += Phi[i] * weights[i];
        }

        qdd = K * (g - q) / (T * T) - B * qd / T + (g - q0) * f / (T * T);
        qd += qdd * dt;
        q += qd * dt;

        Traj.push_back(q);
        execTime.push_back(t);
    }

    zsim_roi_end();
    // ROI ends

    // Write the output trajectory
    std::ofstream outTraj;
    outTraj.open(outputFile);
    for (auto q : Traj) {
        outTraj << q << std::endl;
    }
    outTraj.close();

    return 0;
}
