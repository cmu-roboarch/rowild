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
#include "pfilter.h"
#include "rowild_utils.h"
#include "timer.h"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

#define NUM_ODOMETRY_MEASUR (3) // X, Y, Theta
#define NUM_LASER_MEASUR (180)  // All angles

typedef std::vector<READING> MEASUREMENT_LOG;

void readMeasurements(std::string inputFile, MEASUREMENT_LOG *odometryLog,
                      MEASUREMENT_LOG *laserLog) {
    std::ifstream mFile;
    mFile.open(inputFile);
    assert(mFile.good());

    std::string line;
    while (std::getline(mFile, line)) {
        thrust::host_vector<double> l;

        std::stringstream ss(line);
        double e;
        while (ss >> e) {
            l.push_back(e);
            if (static_cast<int>(l.size()) == NUM_ODOMETRY_MEASUR) {
                READING gl = l;
                odometryLog->push_back(gl);
                break;
            }
        }
        l.clear();

        while (ss >> e) {
            l.push_back(e);
        }
        assert(static_cast<int>(l.size()) == NUM_LASER_MEASUR);
        READING gl = l;
        laserLog->push_back(gl);
    }

    mFile.close();
}

int main(int argc, const char **argv) {
    using args::FlagArg;
    using args::KVArg;
    using args::Parser;
    using std::chrono::duration_cast;
    using std::chrono::high_resolution_clock;
    using std::chrono::nanoseconds;

    Parser parser(argv[0], argc, argv, false);
    KVArg<std::string> inputMapArg(parser, "map", "", "Input map file");
    KVArg<std::string> measurementsArg(parser, "measurements", "",
                                       "Input measurements file");
    KVArg<int> particlesArg(parser, "particles", "", "Number of particles");
    KVArg<int> subsampleArg(parser, "subsample", "",
                            "Subsampling laser readings");
    KVArg<int> maxUpdatesArg(parser, "updates", "",
                             "Maximum number of updates");
    FlagArg kidnappedArg(parser, "kidnapped", "",
                         "Enable the kidnapped robot algorithm");
    KVArg<double> kidnappedThresholdArg(
        parser, "kidnapped-threshold", "",
        "The threshold to identify the kidnapped scenario");
    FlagArg adaptiveArg(parser, "adaptive", "",
                        "Enable the adaptive particle algorithm");
    KVArg<int> maxParticlesArg(
        parser, "max-particles", "",
        "Maximum number of particles with the adaptive particle algorithm");
    KVArg<int> minParticlesArg(
        parser, "min-particles", "",
        "Minimum number of particles with the adaptive particle algorithm");
    KVArg<int> particleChangeStepArg(
        parser, "step", "",
        "Particle increase/decrease with the adaptive particle algorithm");
    KVArg<int> intervalArg(
        parser, "interval", "",
        "The interval length with the adaptive particle algorithm");
    KVArg<std::string> outputArg(parser, "output", "", "Output log file");

    if (!parser.parse()) assert(false);

    assert_msg(inputMapArg.found(), "Input map file is not provided");
    assert_msg(measurementsArg.found(),
               "Input measurements file is not provided");

    std::string mapFile = inputMapArg.value();
    std::string measurementsFile = measurementsArg.value();
    int numParticles = particlesArg.found() ? particlesArg.value() : 500;
    int subsample = subsampleArg.found() ? subsampleArg.value() : 1;
    int maxUpdates = maxUpdatesArg.found() ? maxUpdatesArg.value() : 10'000;
    bool kidnapped = kidnappedArg.found();
    double kidnappedThreshold =
        kidnappedThresholdArg.found() ? kidnappedThresholdArg.value() : 0.25;
    bool adaptive = adaptiveArg.found();
    int maxParticles = maxParticlesArg.found() ? maxParticlesArg.value() : 1000;
    int minParticles = minParticlesArg.found() ? minParticlesArg.value() : 10;
    int step =
        particleChangeStepArg.found() ? particleChangeStepArg.value() : 100;
    int interval = intervalArg.found() ? intervalArg.value() : 10;
    std::string outputFile =
        outputArg.found() ? outputArg.value() : "/dev/null";

    assert(numParticles > 0);
    assert(subsample > 0);
    assert(maxUpdates >= 2);
    assert(kidnappedThreshold > 0 && kidnappedThreshold < 1);
    assert(numParticles <= maxParticles);
    assert(numParticles >= minParticles);
    assert(step < maxParticles);

    MEASUREMENT_LOG *odometryLog = new MEASUREMENT_LOG();
    MEASUREMENT_LOG *laserLog = new MEASUREMENT_LOG();
    readMeasurements(measurementsFile, odometryLog, laserLog);
    assert(odometryLog->size() == laserLog->size());

    ParticleFilter *pf = new ParticleFilter(mapFile, numParticles, subsample);
    MEASUREMENT_LOG outputLog;
    outputLog.push_back(pf->getBelief());

    double execTime = 0;
    for (int i = 1 /*Skip the first reading*/;
         i < static_cast<int>(odometryLog->size()); i++) {
        READING prevOdometry = odometryLog->at(i - 1);
        READING currOdometry = odometryLog->at(i);
        assert(static_cast<int>(prevOdometry.size()) == NUM_ODOMETRY_MEASUR);
        assert(static_cast<int>(currOdometry.size()) == NUM_ODOMETRY_MEASUR);

        READING laserReading = laserLog->at(i);
        assert(static_cast<int>(laserReading.size()) == NUM_LASER_MEASUR);

        timer t;

        // ROI begins
        pf->updateMotion(prevOdometry, currOdometry);
        pf->updateSensor(laserReading);
        pf->resample();
        // ROI ends

        execTime += t.elapsed();

        outputLog.push_back(pf->getBelief());
        if (i >= maxUpdates) break;

        if (kidnapped) {
            pf->handleKidnapping(kidnappedThreshold);
        }

        if (adaptive) {
            if (i % interval == 0) {
                pf->adjustParticles(minParticles, maxParticles, step);
            }
        }
    }

    // Write the output log
    std::ofstream outLogFile;
    outLogFile.open(outputFile);
    for (auto l : outputLog) {
        for (auto e : l) {
            outLogFile << std::setprecision(4) << e << " ";
        }
        outLogFile << std::endl;
    }
    outLogFile.close();

    std::cout << "execTime: " << execTime << std::endl;

    delete pf;

    return 0;
}
