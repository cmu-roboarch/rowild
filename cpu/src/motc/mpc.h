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
#include "solver.h"
#include <vector>

typedef std::vector<double> STATE_VEC;
typedef std::vector<STATE_VEC> TRAJ_VEC;

struct State {
    double x, y, yaw, v;
    State(double _x, double _y, double _yaw, double _v) {
        x = _x;
        y = _y;
        yaw = _yaw;
        v = _v;
    }

    explicit State(State *_state) {
        x = _state->x;
        y = _state->y;
        yaw = _state->yaw;
        v = _state->v;
    }
};

class ModelPredictiveControl {
  public:
    ModelPredictiveControl(TRAJ_VEC *splineCourse, double targetSpeed,
                           double dt);
    ~ModelPredictiveControl();
    TRAJ_VEC *simulate(double maxTime, double goalDist, double stopSpeed,
                       double dl, int maxIters, int neighbors);

  private:
    void calcSpeedProfile();
    void initializeSolver();
    int calcNearestIndex(State *state, int pIdx, int numNeighbors) const;
    int calcRefTraj(State *state, double dl, int pIdx, double **xRef,
                    int numNeighbors) const;
    void updateState(State *state, double a, double delta) const;
    void predictMotion(double **xBar, State *state, double *oA,
                       double *oDelta) const;
    bool linearMpcControl(double **xRef, double **xBar, State *state,
                          double *oA, double *oDelta);
    bool iterativeMpcControl(double **xRef, State *state, bool validO,
                             double *oA, double *oDelta, int maxIters);
    void smoothYaw() const;

    // These parameters must not be changed; CVXPY code is generated
    // based on the these parameters
    int NX = 4; // (x, y, v, yaw)
    int NU = 2; // (acceleration, steer)
    int T = 5;  // Time horizon

    TRAJ_VEC *splineCourse;
    STATE_VEC *speedProfile;
    State *initState;

    double targetSpeed;
    double dt;
    double duTh = 0.1;
    double wheelBase = 2.5;
    double maxSteer = PI / 4;
    double maxSpeed = 55.0 / 3.6;
    double minSpeed = -20.0 / 3.6;
    double maxAccel = 1.0;
};
