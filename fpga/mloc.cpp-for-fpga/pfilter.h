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

#include <random>
#include <string>
#include <vector>
#include "envmap.h"

struct Particle {
    double x, y, theta, w;

    Particle() {
        x = y = theta = w = 0;
    }

    Particle(double _x, double _y, double _theta, double _w) :
        x(_x), y(_y), theta(_theta), w(_w) {}

    Particle(const Particle &_p) {
        x = _p.x;
        y = _p.y;
        theta = _p.theta;
        w = _p.w;
    }
};


class ParticleFilter {
 public:
     ParticleFilter();
     void updateMotion(float *prevOdometry, float *currOdometry);
     void updateSensor(float *laserReading);
     void resample();

 private:
     void initializeParticles(int numParticles);
     void initializeMotionModel();
     void initializeSensorModel();
     void initializeResampler();
     double rayCast(double x, double y, double theta, double degree) const;
     double calcProbability(double zkt, double zktStar);

     static constexpr int numParticles = 500;
     Particle particles[numParticles];

     static constexpr int mapX = 800;
     static constexpr int mapY = 800;
     float map[mapX][mapY];

     // Motion parameters
     double alpha1, alpha2, alpha3, alpha4;

     // Sensor parameters
     double zHit, zShort, zMax, zRand;
     double sigmaHit, lambdaShort;
     double minProbability, maxRange, resolution;
     double sensorOffset;
     int subsample = 1;

     // Resampling
     std::default_random_engine resamRandGen;
     std::uniform_real_distribution<double> *resamDist;
};
