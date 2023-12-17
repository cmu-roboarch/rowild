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

#include "log.h"
#include "rowild_utils.h"
#include <random>
#include <string>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <vector>

typedef thrust::device_vector<double> READING;

struct updateXYFunctor {
    const double dTh;

    __host__ __device__ explicit updateXYFunctor(double _dTh) : dTh(_dTh) {}

    __host__ __device__ void operator()(double &x, double &y,
                                        const double &thetaP) {
        x += dTh * cos(thetaP);
        y += dTh * sin(thetaP);
    }
};

struct updateThetaFunctor {
    const double dRh1, dRh2;

    __host__ __device__ updateThetaFunctor(double _dRh1, double _dRh2)
        : dRh1(_dRh1), dRh2(_dRh2) {}

    __host__ __device__ void operator()(double &theta) {
        theta += dRh1 + dRh2;
        while (theta > PI)
            theta -= TWO_PI;
        while (theta < -PI)
            theta += TWO_PI;
    }
};

struct calcProbabilityFunctor {
    const int numRays, subsample;
    const int mapSizeX, mapSizeY;
    const double resolution;

    thrust::device_ptr<double> gParticlesX, gParticlesY, gParticlesTheta;
    thrust::device_ptr<double> gLaserReading;
    thrust::device_ptr<double> occGrid;

    // These numbers are tuned for CMU's Wean Hall
    double zHit = 10;
    double zShort = 0.01;
    double zMax = 0.1;
    double zRand = 10;
    double sigmaHit = 50.0;
    double lambdaShort = 0.1;
    double minProbability = 0.35;
    double maxRange = 1000.0;
    double sensorOffset = 25.0;

    __host__ __device__ calcProbabilityFunctor(
        int _numRays, int _subsample, int _mapSizeX, int _mapSizeY,
        double _resolution, thrust::device_ptr<double> _gParticlesX,
        thrust::device_ptr<double> _gParticlesY,
        thrust::device_ptr<double> _gParticlesTheta,
        thrust::device_ptr<double> _gLaserReading,
        thrust::device_ptr<double> _occGrid)
        : numRays(_numRays), subsample(_subsample), mapSizeX(_mapSizeX),
          mapSizeY(_mapSizeY), resolution(_resolution),
          gParticlesX(_gParticlesX), gParticlesY(_gParticlesY),
          gParticlesTheta(_gParticlesTheta), gLaserReading(_gLaserReading),
          occGrid(_occGrid) {}

    __host__ __device__ void operator()(const int &probabilityIdx,
                                        double &probability) {
        int particleIdx = probabilityIdx / numRays;
        int rayIdx = probabilityIdx % numRays;
        double degree = rayIdx * subsample;

        double x = gParticlesX[particleIdx];
        double y = gParticlesY[particleIdx];
        double theta = gParticlesTheta[particleIdx];

        // >>> Ray-casting
        double xRay = x + sensorOffset * cos(theta);
        double yRay = y + sensorOffset * sin(theta);

        double step = resolution;
        double xStep = step * cos(PI / 2 + theta - degree * (PI / 180));
        double yStep = step * sin(PI / 2 + theta - degree * (PI / 180));

        double dist = 0;
        while (true) {
            dist += step;
            xRay += xStep;
            yRay += yStep;

            int xIdx = static_cast<int>(xRay / resolution);
            int yIdx = static_cast<int>(yRay / resolution);

            if (dist >= maxRange || xIdx >= mapSizeX || yIdx >= mapSizeY ||
                xIdx < 0 || yIdx < 0) {
                break;
            }

            int gIdx = xIdx * mapSizeY + yIdx;
            double occ = occGrid[gIdx];
            if (occ == -1 || occ >= minProbability) {
                break;
            }
        }
        // <<< Ray-casting

        double zkt = gLaserReading[degree];
        double zktStar = dist;
        double pRand = (zkt >= 0 && zkt < maxRange) ? 1.0 / maxRange : 0.0;
        double pMax = (zkt >= maxRange) ? 1.0 : 0.0;
        double pShort = (zkt >= 0 && zkt <= zktStar)
                            ? (1.0 / (1 - exp(-lambdaShort * zktStar))) *
                                  lambdaShort * exp(-lambdaShort * zkt)
                            : 0.0;
        double pHit = (zkt >= 0 && zkt <= maxRange)
                          ? exp(-0.5 * (zkt - zktStar) * (zkt - zktStar) /
                                (sigmaHit * sigmaHit)) /
                                sqrt(TWO_PI * sigmaHit * sigmaHit)
                          : 0.0;
        probability =
            zHit * pHit + zShort * pShort + zMax * pMax + zRand * pRand;
    }
};

class ParticleFilter {
  public:
    ParticleFilter(std::string inputMapFile, int numParticles, int subsample);
    ~ParticleFilter();
    void updateMotion(const READING &prevOdometry, const READING &currOdometry);
    void updateSensor(const READING &laserReading);
    void resample();
    std::vector<double> getBelief() const;
    void handleKidnapping(double threshold);
    void adjustParticles(int minParticles, int maxParticles, int step);

  private:
    void initializeOccGrid(std::string inputMapFile);
    void initializeParticles(int numParticles);
    void initializeMotionModel();
    void initializeResampler();
    void addParicles(int numNewParticles);
    double getDevIdx() const;

    thrust::device_vector<double> particlesX;
    thrust::device_vector<double> particlesY;
    thrust::device_vector<double> particlesTheta;
    thrust::device_vector<double> particlesW;

    // Motion parameters
    double alpha1, alpha2, alpha3, alpha4;

    // Sensor parameters
    double subsample, resolution;

    // Resampling
    std::default_random_engine resamRandGen;
    std::uniform_real_distribution<double> *resamDist;

    // Environment map
    thrust::device_vector<double> occGrid;
    std::vector<int> mapFreeXs, mapFreeYs;
    int mapSizeX, mapSizeY;

    // Kidnapping parameters
    int numKidnaps = 0;
    double initialDev, prevDev;

    // Adaptive particle parameters
    int initialNumParticles;
    int numReconfigurations = 0;
    double maxDev;
};
