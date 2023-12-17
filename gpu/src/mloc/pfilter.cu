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
#include <fstream>
#include <string>
#include <thrust/for_each.h>
#include <thrust/gather.h>
#include <thrust/iterator/zip_iterator.h>
#include <thrust/zip_function.h>
#include <vector>

ParticleFilter::ParticleFilter(std::string inputMapFile, int _numParticles,
                               int _subsample) {
    this->initializeOccGrid(inputMapFile);
    this->subsample = _subsample;

    this->initializeParticles(_numParticles);
    this->initializeMotionModel();
    this->initializeResampler();

    this->maxDev = this->getDevIdx();
    this->initialNumParticles = _numParticles;
    this->initialDev = maxDev;
    this->prevDev = INT_MAX;
}

ParticleFilter::~ParticleFilter() { delete this->resamDist; }

void ParticleFilter::initializeOccGrid(std::string inputMapFile) {
    std::ifstream mapFile(inputMapFile);
    assert(mapFile.good());

    std::string strToken;
    int intToken;

    mapFile >> strToken >> intToken;
    assert(strToken == "mapsize_x");
    this->mapSizeX = intToken;

    mapFile >> strToken >> intToken;
    assert(strToken == "mapsize_y");
    this->mapSizeY = intToken;

    mapFile >> strToken >> intToken;
    assert(strToken == "resolution");
    this->resolution = intToken;

    this->mapSizeX /= resolution;
    this->mapSizeY /= resolution;

    thrust::host_vector<double> hOccGrid(this->mapSizeX * this->mapSizeY);

    std::getline(mapFile, strToken);
    assert(strToken.empty());

    for (int i = 0; i < this->mapSizeX; i++) {
        std::getline(mapFile, strToken);
        assert(!strToken.empty());

        std::stringstream row(strToken);

        int j = 0;
        double e;
        while (row >> e) {
            int gIdx = i * this->mapSizeY + j;
            hOccGrid[gIdx] = e;
            if (e == 0) {
                this->mapFreeXs.push_back(i);
                this->mapFreeYs.push_back(j);
            }
            j++;
        }

        assert(j == this->mapSizeY);
    }
    mapFile.close();

    this->occGrid = hOccGrid;
}

void ParticleFilter::initializeParticles(int numParticles) {
    // Randomly sample from free space
    assert(this->mapFreeXs.size() == this->mapFreeYs.size());
    int numFreeLocs = static_cast<int>(this->mapFreeXs.size());

    std::default_random_engine intGen;
    std::uniform_int_distribution<int> intDist(0, numFreeLocs - 1);

    std::default_random_engine realGen;
    std::uniform_real_distribution<double> realDist(-PI, PI);

    this->particlesX.resize(numParticles);
    this->particlesY.resize(numParticles);
    this->particlesTheta.resize(numParticles);
    this->particlesW.resize(numParticles);

    for (int i = 0; i < numParticles; i++) {
        int idx = intDist(intGen);
        assert(idx >= 0 && idx < numFreeLocs);

        double x = this->mapFreeXs[idx];
        double y = this->mapFreeYs[idx];
        int gIdx = static_cast<int>(x) * this->mapSizeY + static_cast<int>(y);
        assert(this->occGrid[gIdx] == 0);

        x *= this->resolution;
        y *= this->resolution;

        double theta = realDist(realGen);
        assert(theta >= -PI && theta < PI);

        double weight = 1.0 / numParticles;
        this->particlesX[i] = x;
        this->particlesY[i] = y;
        this->particlesTheta[i] = theta;
        this->particlesW[i] = weight;
    }
}

void ParticleFilter::addParicles(int numNewParticles) {
    size_t numParticles = this->particlesX.size();
    assert(numParticles == this->particlesY.size());
    assert(numParticles == this->particlesTheta.size());
    assert(numParticles == this->particlesW.size());

    assert(this->mapFreeXs.size() == this->mapFreeYs.size());
    int numFreeLocs = static_cast<int>(this->mapFreeXs.size());

    std::default_random_engine intGen;
    std::uniform_int_distribution<int> intDist(0, numFreeLocs - 1);

    std::default_random_engine realGen;
    std::uniform_real_distribution<double> realDist(-PI, PI);

    this->particlesX.resize(numParticles + numNewParticles);
    this->particlesY.resize(numParticles + numNewParticles);
    this->particlesTheta.resize(numParticles + numNewParticles);
    this->particlesW.resize(numParticles + numNewParticles);

    for (size_t i = numParticles; i < numParticles + numNewParticles; i++) {
        int idx = intDist(intGen);
        assert(idx >= 0 && idx < numFreeLocs);

        double x = this->mapFreeXs[idx];
        double y = this->mapFreeYs[idx];
        int gIdx = static_cast<int>(x) * this->mapSizeY + static_cast<int>(y);
        assert(this->occGrid[gIdx] == 0);

        x *= this->resolution;
        y *= this->resolution;

        double theta = realDist(realGen);
        assert(theta >= -PI && theta < PI);

        double weight = 1.0 / (numParticles + numNewParticles);
        this->particlesX[i] = x;
        this->particlesY[i] = y;
        this->particlesTheta[i] = theta;
        this->particlesW[i] = weight;
    }

    assert(this->particlesX.size() == numParticles + numNewParticles);
}

void ParticleFilter::initializeMotionModel() {
    // These numbers are tuned for CMU's Wean Hall
    this->alpha1 = 0.001;
    this->alpha2 = 0.001;
    this->alpha3 = 0.1;
    this->alpha4 = 0.8;
}

void ParticleFilter::updateMotion(const READING &prevOdometry,
                                  const READING &currOdometry) {
    double xDiff = currOdometry[0] - prevOdometry[0];
    double yDiff = currOdometry[1] - prevOdometry[1];
    double thetaDiff = currOdometry[2] - prevOdometry[2];

    if (std::abs(xDiff) + std::abs(yDiff) + std::abs(thetaDiff) < 1e-10) {
        // Avoid the overheads if the difference is minor
        return;
    }

    double dRot1 = atan2(yDiff, xDiff) - prevOdometry[2];
    double dTrans = sqrt(xDiff * xDiff + yDiff * yDiff);
    double dRot2 = currOdometry[2] - prevOdometry[2] - dRot1;

    std::default_random_engine generator;

    double scaleH1 =
        sqrt(this->alpha1 * dRot1 * dRot1 + this->alpha2 * dTrans * dTrans);

    double scaleTh =
        sqrt(this->alpha3 * dTrans * dTrans + this->alpha4 * dRot1 * dRot1 +
             this->alpha4 * dRot2 * dRot2);

    double scaleH2 =
        sqrt(this->alpha1 * dRot2 * dRot2 + this->alpha2 * dTrans * dTrans);

    std::normal_distribution<double> h1Dist(0, scaleH1);
    std::normal_distribution<double> thDist(0, scaleTh);
    std::normal_distribution<double> h2Dist(0, scaleH2);

    double dRh1 = dRot1 - h1Dist(generator);
    double dTh = dTrans - thDist(generator);
    double dRh2 = dRot2 - h2Dist(generator);

    size_t numParticles = this->particlesX.size();

    // ThetaPrime <- Theta + dRh1
    thrust::device_vector<float> thetaPrime(numParticles);
    thrust::transform(this->particlesTheta.begin(), this->particlesTheta.end(),
                      thrust::constant_iterator<double>(dRh1),
                      thetaPrime.begin(), thrust::plus<double>());

    // X <- X + dTh * cos(thetaPrime)
    // Y <- Y + dTh * sin(thetaPrime)
    thrust::for_each(thrust::make_zip_iterator(thrust::make_tuple(
                         this->particlesX.begin(), this->particlesY.begin(),
                         thetaPrime.begin())),

                     thrust::make_zip_iterator(thrust::make_tuple(
                         this->particlesX.end(), this->particlesY.begin(),
                         thetaPrime.end())),

                     thrust::make_zip_function(updateXYFunctor(dTh)));

    // Theta <- wrap (Theta + dRh1 + dRh2)
    thrust::for_each(this->particlesTheta.begin(), this->particlesTheta.end(),
                     updateThetaFunctor(dRh1, dRh2));
}

void ParticleFilter::updateSensor(const READING &laserReading) {
    int numParticles = static_cast<int>(this->particlesX.size());
    int numRays = 180 / static_cast<int>(this->subsample);
    thrust::device_vector<double> probs(numParticles * numRays);
    thrust::device_vector<double> gLaserReading = laserReading;

    thrust::for_each(
        thrust::make_zip_iterator(thrust::make_tuple(
            thrust::counting_iterator<int>(0), probs.begin())),

        thrust::make_zip_iterator(thrust::make_tuple(
            thrust::counting_iterator<int>(0), probs.begin())) +
            probs.size(),

        thrust::make_zip_function(calcProbabilityFunctor(
            numRays, this->subsample, this->mapSizeX, this->mapSizeY,
            this->resolution, this->particlesX.data(), this->particlesY.data(),
            this->particlesTheta.data(), gLaserReading.data(),
            this->occGrid.data())));

    // Reduce
    thrust::device_vector<int> probIndices(numParticles);
    thrust::reduce_by_key(
        thrust::make_transform_iterator(thrust::counting_iterator<int>(0),
                                        linearIdxToRowIdx<int>(numRays)),

        thrust::make_transform_iterator(thrust::counting_iterator<int>(0),
                                        linearIdxToRowIdx<int>(numRays)) +
            (numParticles * numRays),

        probs.begin(), probIndices.begin(), this->particlesW.begin(),
        thrust::equal_to<int>(), thrust::multiplies<double>());
}

void ParticleFilter::initializeResampler() {
    size_t numParticles = this->particlesX.size();

    assert(numParticles > 0);
    this->resamDist =
        new std::uniform_real_distribution<double>(0, 1.0 / numParticles);
}

void ParticleFilter::resample() {
    // I implemented low-variance sampling, which is an awfully serial process
    // and must run on CPU. An alternative could be be sampling from a
    // multinomial distribution constructed using importance weights of all
    // particles, which could be parallelized on GPU. However, repetitive
    // resampling using such a technique may cause the variance of the particle
    // set (as an estimator of the true belief) to increase.

    size_t numParticles = this->particlesX.size();

    double init = 0;
    double wSum =
        thrust::reduce(this->particlesW.begin(), this->particlesW.end(), init,
                       thrust::plus<double>());

    thrust::transform(this->particlesW.begin(), this->particlesW.end(),
                      thrust::constant_iterator<double>(wSum),
                      this->particlesW.begin(), thrust::divides<double>());

    thrust::host_vector<double> weights = this->particlesW;
    thrust::host_vector<int> tempIndices(numParticles);
    double M1 = 1.0 / numParticles;
    double r = (*this->resamDist)(this->resamRandGen);
    double c = weights[0];
    int i = 0;

    for (size_t m = 0; m < numParticles; m++) {
        double u = r + m * M1;
        while (u > c) {
            i++;
            c += weights[i];
        }
        tempIndices[m] = i;
    }

    thrust::device_vector<int> indices = tempIndices;

    thrust::device_vector<double> xCopies = this->particlesX;
    thrust::device_vector<double> yCopies = this->particlesY;
    thrust::device_vector<double> thetaCopies = this->particlesTheta;
    thrust::device_vector<double> wCopies = this->particlesW;

    thrust::gather(indices.begin(), indices.end(), xCopies.begin(),
                   this->particlesX.begin());

    thrust::gather(indices.begin(), indices.end(), yCopies.begin(),
                   this->particlesY.begin());

    thrust::gather(indices.begin(), indices.end(), thetaCopies.begin(),
                   this->particlesTheta.begin());

    thrust::gather(indices.begin(), indices.end(), wCopies.begin(),
                   this->particlesW.begin());
}

std::vector<double> ParticleFilter::getBelief() const {
    size_t numParticles = this->particlesX.size();

    double init = 0;
    double xMean =
        thrust::reduce(this->particlesX.begin(), this->particlesX.end(), init,
                       thrust::plus<double>());

    double yMean =
        thrust::reduce(this->particlesY.begin(), this->particlesY.end(), init,
                       thrust::plus<double>());

    double thetaMean =
        thrust::reduce(this->particlesTheta.begin(), this->particlesTheta.end(),
                       init, thrust::plus<double>());

    xMean /= numParticles;
    yMean /= numParticles;
    thetaMean /= numParticles;

    return {xMean, yMean, thetaMean};
}

double ParticleFilter::getDevIdx() const {
    size_t numParticles = this->particlesX.size();
    double init = 0;

    double xMean =
        thrust::reduce(this->particlesX.begin(), this->particlesX.end(), init,
                       thrust::plus<double>());

    double yMean =
        thrust::reduce(this->particlesY.begin(), this->particlesY.end(), init,
                       thrust::plus<double>());

    xMean /= numParticles;
    yMean /= numParticles;

    thrust::plus<double> binOp;
    double xDev = thrust::transform_reduce(
        this->particlesX.begin(), this->particlesX.end(),
        stdDevFunctor<double>(xMean), init, binOp);
    double yDev = thrust::transform_reduce(
        this->particlesY.begin(), this->particlesY.end(),
        stdDevFunctor<double>(yMean), init, binOp);

    xDev /= numParticles;
    yDev /= numParticles;

    return xDev + yDev;
}

void ParticleFilter::handleKidnapping(double threshold) {
    double currDev = this->getDevIdx();
    if (unlikely(currDev - this->prevDev > threshold * this->initialDev)) {
        this->numKidnaps++;
        size_t num = this->particlesX.size();
        this->initializeParticles(static_cast<int>(num));
        assert(this->particlesX.size() == num);
        this->initialDev = this->getDevIdx();
        this->prevDev = INT_MAX;
    } else {
        this->prevDev = currDev;
    }
}

void ParticleFilter::adjustParticles(int minParticles, int maxParticles,
                                     int step) {
    int numParticles = this->particlesX.size();
    double currDev = this->getDevIdx();
    int expectedParticles =
        round((currDev / this->maxDev) * this->initialNumParticles);

    if (unlikely(expectedParticles > maxParticles)) {
        expectedParticles = maxParticles;
    } else if (expectedParticles < minParticles) {
        expectedParticles = minParticles;
    }

    if (numParticles < expectedParticles) {
        if (numParticles + step > maxParticles) return;

        this->addParicles(step);
        this->numReconfigurations++;
    } else if (numParticles > expectedParticles) {
        if (numParticles - step < minParticles) return;

        this->particlesX.resize(numParticles - step);
        this->particlesY.resize(numParticles - step);
        this->particlesTheta.resize(numParticles - step);
        this->particlesW.resize(numParticles - step);
        this->numReconfigurations++;
    }
}
