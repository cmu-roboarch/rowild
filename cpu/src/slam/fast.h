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
#include "slam.h"
#include <random>
#include <vector>

class FastSLAM : public SLAM {
  public:
    FastSLAM(int N, int num_landmarks, double sX2, double sY2, double sA2,
             double sB2, double sR2)
        : SLAM(num_landmarks), numParticles(N), sigX2(sX2), sigY2(sY2),
          sigAlpha2(sA2), sigBeta2(sB2), sigR2(sR2),
          generator(std::default_random_engine(0)) {
        for (int i = 0; i < numParticles; i++) {
            Particle p = {0, 0, 0,
                          std::vector<Landmark>(num_landmarks, {0, 0, 0}),
                          1.0 / N};
            particles.push_back(p);
        }
    }

    void motionUpdate(double x, double y, double theta) {
        std::normal_distribution<double> distributionX(0, sqrt(sigX2));
        std::normal_distribution<double> distributionY(0, sqrt(sigY2));
        std::normal_distribution<double> distributionTheta(0, sqrt(sigAlpha2));
        for (auto &p : particles) {
            p.x += x + distributionX(generator);
            p.y += y + distributionY(generator);
            p.theta += theta + distributionTheta(generator);
        }
    }

    void measurementUpdate(const std::vector<double> &measurements) {
        for (auto &p : particles) {
            double weight_product = 1.0;

            for (int i = 0; i < 6; i++) {
                double range = measurements[2 * i];
                double bearing = measurements[2 * i + 1];

                double predicted_range =
                    std::hypot(p.landmarks[i].x - p.x, p.landmarks[i].y - p.y);
                double predicted_bearing =
                    atan2(p.landmarks[i].y - p.y, p.landmarks[i].x - p.x) -
                    p.theta;

                std::normal_distribution<double> distributionR(predicted_range,
                                                               sqrt(sigR2));
                std::normal_distribution<double> distributionB(
                    predicted_bearing, sqrt(sigBeta2));

                // Update weights
                weight_product *=
                    distributionR(generator) * distributionB(generator);

                // Update landmark estimate
                p.landmarks[i].x += range * cos(bearing + p.theta);
                p.landmarks[i].y += range * sin(bearing + p.theta);
            }

            p.weight *= weight_product;
        }
    }

    void resample() {
        double totalWeight = 0;
        for (auto &p : particles) {
            totalWeight += p.weight;
        }

        for (auto &p : particles) {
            p.weight /= totalWeight;
        }

        // Calculate cumulative weights directly
        std::vector<double> cumulativeWeights(particles.size());
        cumulativeWeights[0] = particles[0].weight;
        for (size_t i = 1; i < particles.size(); ++i) {
            cumulativeWeights[i] =
                cumulativeWeights[i - 1] + particles[i].weight;
        }

        std::vector<Particle> newParticles;
        std::uniform_real_distribution<double> distribution(0.0,
                                                            1.0 / numParticles);
        double step = distribution(generator);
        int index = 0;
        for (int i = 0; i < numParticles; i++) {
            while (step > cumulativeWeights[index]) {
                index = (index + 1) % numParticles;
            }
            newParticles.push_back(particles[index]);
            step += 1.0 / numParticles;
        }

        particles = newParticles;
    }

    std::vector<double> getStatus() const override {
        std::vector<double> status;

        // Compute estimated robot position (mean position of particles)
        double meanX = std::accumulate(particles.begin(), particles.end(), 0.0,
                                       [](double acc, const Particle &p) {
                                           return acc + p.x;
                                       }) /
                       numParticles;
        double meanY = std::accumulate(particles.begin(), particles.end(), 0.0,
                                       [](double acc, const Particle &p) {
                                           return acc + p.y;
                                       }) /
                       numParticles;

        status.push_back(meanX);
        status.push_back(meanY);

        // Compute estimated landmarks' positions
        for (int i = 0; i < 6; i++) {
            double sumLandmarkX = 0.0;
            double sumLandmarkY = 0.0;

            for (const auto &p : particles) {
                sumLandmarkX += p.landmarks[i].x;
                sumLandmarkY += p.landmarks[i].y;
            }

            double meanLandmarkX = sumLandmarkX / numParticles;
            double meanLandmarkY = sumLandmarkY / numParticles;

            status.push_back(meanLandmarkX);
            status.push_back(meanLandmarkY);
        }

        return status;
    }

  private:
    std::vector<Particle> particles;
    int numParticles;
    double sigX2;
    double sigY2;
    double sigAlpha2;
    double sigBeta2;
    double sigR2;
    std::default_random_engine generator;
};
