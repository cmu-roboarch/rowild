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

#include "pfilter.h"
#include <algorithm>
#include <string>
#include <vector>
#include <cmath>

#define PI (3.141592654)
#define TWO_PI (6.283185308)


template<typename T>
T degToRad(T angle) {
    return angle * (PI / 180);
}


template<typename T>
T wrapToPi(T angle) {
    while (angle > PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}

float uniform_random() {
    static unsigned int seed = 123456789; // Example seed
    unsigned int a = 1103515245;
    unsigned int c = 12345;
    unsigned int m = static_cast<unsigned int>(pow(2, 31));

    seed = (a * seed + c) % m;
    return static_cast<float>(seed) / m;
}

void box_muller(float mean, float stddev, float &normal1, float &normal2) {
    float u1 = uniform_random();
    float u2 = uniform_random();

    normal1 = mean + sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2) * stddev;
    normal2 = mean + sqrt(-2.0 * log(u1)) * sin(2.0 * M_PI * u2) * stddev;
}


ParticleFilter::ParticleFilter() {
    this->resolution = 10;
    this->initializeParticles(numParticles);
    this->initializeMotionModel();
    this->initializeSensorModel();
    this->initializeResampler();
}


void ParticleFilter::initializeParticles(int numParticles) {
    for (int i = 0; i < numParticles; i++) {
        double x = (i * 1234) % 800;
        double y = (i * 4321) % 800;

        double theta = 1.0 * (i % 3);
        double weight = 1.0 / numParticles;
        particles[i].x = x;
        particles[i].y = y;
        particles[i].theta = theta;
        particles[i].w = weight;
    }
}


void ParticleFilter::initializeMotionModel() {
    // These numbers are tuned for CMU's Wean Hall
    this->alpha1 = 0.001;
    this->alpha2 = 0.001;
    this->alpha3 = 0.1;
    this->alpha4 = 0.8;
}


void ParticleFilter::updateMotion(float *prevOdometry, float *currOdometry) {
    double xDiff = currOdometry[0] - prevOdometry[0];
    double yDiff = currOdometry[1] - prevOdometry[1];
    double thetaDiff = currOdometry[2] - prevOdometry[2];

    if (std::abs(xDiff) + std::abs(yDiff) + std::abs(thetaDiff) < 1e-10) {
        // Avoid the overheads if the difference is minor
        return;
    }

    double dRot1 = atan2(yDiff, xDiff) - prevOdometry[2];
    double dTrans = sqrt(xDiff*xDiff + yDiff*yDiff);
    double dRot2 = currOdometry[2] - prevOdometry[2] - dRot1;

    double scaleH1 = sqrt(this->alpha1*dRot1*dRot1 + this->alpha2*dTrans*dTrans);
    double scaleTh = sqrt(this->alpha3*dTrans*dTrans + this->alpha4*dRot1*dRot1 + this->alpha4*dRot2*dRot2);
    double scaleH2 = sqrt(this->alpha1*dRot2*dRot2 + this->alpha2*dTrans*dTrans);

    std::normal_distribution<double> h1Dist(0, scaleH1);
    std::normal_distribution<double> thDist(0, scaleTh);
    std::normal_distribution<double> h2Dist(0, scaleH2);

    float random1, random2;
    box_muller(0, scaleH1, random1, random2);
    double dRh1 = dRot1 - random1;

    box_muller(0, scaleTh, random1, random2);
    double dTh = dTrans - random1;

    box_muller(0, scaleH2, random1, random2);
    double dRh2 = dRot2 - random1;

    for (int i = 0; i < this->numParticles; i++) {
        double thetaPrime = this->particles[i].theta + dRh1;

        this->particles[i].x += dTh * cos(thetaPrime);
        this->particles[i].y += dTh * sin(thetaPrime);
        this->particles[i].theta = wrapToPi(this->particles[i].theta + dRh1 + dRh2);
    }
}


void ParticleFilter::initializeSensorModel() {
    // These numbers are tuned for CMU's Wean Hall
    this->zHit = 10;
    this->zShort = 0.01;
    this->zMax = 0.1;
    this->zRand = 10;
    this->sigmaHit = 50.0;
    this->lambdaShort = 0.1;

    this->minProbability = 0.35;
    this->maxRange = 1000.0;

    // The distance of the laser from the robot
    this->sensorOffset = 25.0;
}


void ParticleFilter::updateSensor(float *laserReading) {
    for (int i = 0; i < this->numParticles; i++) {
        double x = this->particles[i].x;
        double y = this->particles[i].y;
        double theta = this->particles[i].theta;

        double zStar[180];
        for (int d = 0; d < 180; d += this->subsample) {
            zStar[d] = rayCast(x, y, theta, static_cast<double>(d));
        }

        double probability = 1.0;
        for (int eIdx = 0; eIdx < 180; eIdx++) {
            int rIdx = eIdx * this->subsample;

            double expDist = zStar[eIdx];
            double readDist = laserReading[rIdx];
            probability *= this->calcProbability(readDist, expDist);
        }

        this->particles[i].w = probability;
    }
}


double ParticleFilter::calcProbability(double zkt, double zktStar) {
    double pRand = 0;
    if (zkt >= 0 && zkt < this->maxRange) {
        pRand = 1.0 / this->maxRange;
    }

    double pMax = 0;
    if (zkt >= this->maxRange) {
        pMax = 1.0;
    }

    double pShort = 0;
    if (zkt >= 0 && zkt <= zktStar) {
        double n = 1.0 / (1 - exp(-this->lambdaShort * zktStar));
        pShort = n * this->lambdaShort * exp(-this->lambdaShort * zkt);
    }

    double pHit = 0;
    if (zkt >= 0 && zkt <= this->maxRange) {
        pHit = exp(-0.5 * (zkt - zktStar) * (zkt - zktStar) / (this->sigmaHit * this->sigmaHit));
        pHit /= sqrt(TWO_PI * this->sigmaHit * this->sigmaHit);
    }

    return this->zHit*pHit + this->zShort*pShort + zMax*pMax + this->zRand*pRand;
}


double ParticleFilter::rayCast(double x, double y, double theta, double degree) const {
    double xRay = x + this->sensorOffset * cos(theta);
    double yRay = y + this->sensorOffset * sin(theta);

    double step = this->resolution;
    double xStep = step * cos(PI/2 + theta - degToRad(degree));
    double yStep = step * sin(PI/2 + theta - degToRad(degree));

    double dist = 0;
    while (true) {
        dist += step;
        xRay += xStep;
        yRay += yStep;

        int xIdx = static_cast<int>(xRay / this->resolution);
        int yIdx = static_cast<int>(yRay / this->resolution);

        if (dist >= this->maxRange || xIdx >= 800 || yIdx >= 800 || xIdx < 0 || yIdx < 0) {
            break;
        }

        double occ = this->map[xIdx][yIdx];
        if (occ == -1 || occ >= this->minProbability) {
            break;
        }
    }
    return dist;
}


void ParticleFilter::initializeResampler() {
#ifndef __SYNTHESIS__
    this->resamDist = new std::uniform_real_distribution<double> (0, 1.0/this->numParticles);
#endif
}


void ParticleFilter::resample() {
#ifndef __SYNTHESIS__
	// Low-variance sampling

    double wSum = 0;
    for (int i = 0; i < numParticles; i++) {
        wSum += this->particles[i].w;
    }
    for (int i = 0; i < numParticles; i++) {
        this->particles[i].w /= wSum;
    }

    Particle newParticles[numParticles];
    int npidx = 0;

    double M1 = 1.0 / this->numParticles;
    double r = (*this->resamDist)(this->resamRandGen);
    double c = this->particles[0].w;
    int i = 0;

    for (int m = 0; m < this->numParticles; m++) {
        double u = r + m*M1;
        while (u > c) {
            i++;
            c += this->particles[i].w;
        }
        newParticles[npidx++] = this->particles[i];
    }

    for (int i = 0; i < numParticles; i++) {
        this->particles[i] = newParticles[i];
    }
#endif
}
