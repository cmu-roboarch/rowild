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

#include "mpc.h"
#include "solver.h"
#include <algorithm>
#include <limits>
#include <utility>

// CVXGEN needs the following structures to be
// statically allocated as global variables
Vars vars;
Params params;
Workspace work;
Settings settings;

ModelPredictiveControl::ModelPredictiveControl(TRAJ_VEC *_splineCourse,
                                               double _targetSpeed,
                                               double _dt) {
    this->splineCourse = new TRAJ_VEC(*_splineCourse);
    this->targetSpeed = _targetSpeed;
    this->dt = _dt;

    this->speedProfile = new STATE_VEC();
    this->calcSpeedProfile();
    this->initState = new State(this->splineCourse->at(0).at(0),
                                this->splineCourse->at(0).at(1),
                                this->splineCourse->at(0).at(2), 0.0);

    set_defaults();
    setup_indexing();
    this->initializeSolver();
    settings.verbose = 0;
}

ModelPredictiveControl::~ModelPredictiveControl() {
    delete splineCourse;
    delete speedProfile;
    delete initState;
}

TRAJ_VEC *ModelPredictiveControl::simulate(double maxTime, double goalDist,
                                           double stopSpeed, double dl,
                                           int maxIters, int neighbors) {
    int sCourseLength = static_cast<int>(this->splineCourse->size());
    std::pair<double, double> goal = {
        this->splineCourse->at(sCourseLength - 1).at(0), // X
        this->splineCourse->at(sCourseLength - 1).at(1)  // Y
    };

    auto isGoal = [&goal, goalDist, stopSpeed, sCourseLength](State *state,
                                                              int targetIdx) {
        double dX = state->x - goal.first;
        double dY = state->y - goal.second;
        double d = sqrt(dX * dX + dY * dY);

        bool g = (d <= goalDist);

        if (unlikely(std::abs(targetIdx - sCourseLength) >= 5)) {
            g = false;
        }

        bool s = (std::abs(state->v) <= stopSpeed);

        return (g && s);
    };

    double cYaw0 = this->splineCourse->at(0).at(2);
    State *state = new State(this->initState);
    if (state->yaw - cYaw0 >= PI)
        state->yaw -= TWO_PI;
    else if (state->yaw - cYaw0 <= -PI)
        state->yaw += TWO_PI;

    TRAJ_VEC *traj = new TRAJ_VEC();

    double time = 0, d = 0, a = 0;
    traj->push_back(
        STATE_VEC({time, state->x, state->y, state->yaw, state->v, d, a}));

    int targetIdx = this->calcNearestIndex(state, 0, neighbors);

    this->smoothYaw();

    double *oDelta = new double[this->T];
    double *oA = new double[this->T];
    bool validO = false;
    double di = -1, ai = -1;

    while (time <= maxTime) {
        double **xRef = new double *[this->NX];
        for (int i = 0; i < this->NX; i++) {
            xRef[i] = new double[this->T + 1]();
        }

        targetIdx = this->calcRefTraj(state, dl, targetIdx, xRef, neighbors);

        validO = this->iterativeMpcControl(xRef, state, validO, oA, oDelta,
                                           maxIters);

        if (validO) {
            di = oDelta[0];
            ai = oA[0];

            time += this->dt;
            this->updateState(state, ai, di);
            traj->push_back(STATE_VEC(
                {time, state->x, state->y, state->yaw, state->v, di, ai}));

            if (unlikely(isGoal(state, targetIdx))) break;
        }

        for (int i = 0; i < this->NX; i++)
            delete[] xRef[i];
        delete[] xRef;

        if (!validO) break;
    }

    delete state;
    delete[] oDelta;
    delete[] oA;

    return traj;
}

void ModelPredictiveControl::calcSpeedProfile() {
    bool forward = true;

    for (int i = 0; i < static_cast<int>(this->splineCourse->size()) - 1; i++) {
        double cx0 = this->splineCourse->at(i).at(0);
        double cx1 = this->splineCourse->at(i + 1).at(0);
        double cy0 = this->splineCourse->at(i).at(1);
        double cy1 = this->splineCourse->at(i + 1).at(1);
        double dx = cx1 - cx0;
        double dy = cy1 - cy0;

        double cYaw = this->splineCourse->at(i).at(2);
        double moveDir = atan2(dy, dx);

        if (dx != 0 && dy != 0) {
            double dangle = std::abs(wrapToPi(moveDir - cYaw));
            forward = dangle < PI / 4;
        }

        if (forward) {
            this->speedProfile->push_back(this->targetSpeed);
        } else {
            this->speedProfile->push_back(-this->targetSpeed);
        }
    }

    this->speedProfile->push_back(0.0);
}

void ModelPredictiveControl::initializeSolver() {
    params.R[0] = 0.01;
    params.R[1] = 0.01;

    params.Q[0] = 1.0;
    params.Q[1] = 1.0;
    params.Q[2] = 0.5;
    params.Q[3] = 0.5;

    params.Rd[0] = 0.01;
    params.Rd[1] = 1.0;

    params.Qf[0] = 1.0;
    params.Qf[1] = 1.0;
    params.Qf[2] = 0.5;
    params.Qf[3] = 0.5;

    params.MAX_STEER_DT[0] = this->maxSteer * this->dt;
    params.MAX_SPEED[0] = this->maxSpeed;
    params.MIN_SPEED[0] = this->minSpeed;
    params.MAX_ACCEL[0] = this->maxAccel;
    params.MAX_STEER[0] = this->maxSteer;
}

int ModelPredictiveControl::calcNearestIndex(State *state, int pIdx,
                                             int numNeighbors) const {
    int minIdx = 0;
    double minVal = std::numeric_limits<float>::max();
    int lastIdx = std::min(pIdx + numNeighbors,
                           static_cast<int>(this->splineCourse->size()));
    for (int i = pIdx; i < lastIdx; i++) {
        double cx = this->splineCourse->at(i).at(0);
        double cy = this->splineCourse->at(i).at(1);
        double dX = state->x - cx, dY = state->y - cy;
        double d = dX * dX + dY * dY;

        if (d < minVal) {
            minIdx = i;
            minVal = d;
        }
    }

    return minIdx;
}

int ModelPredictiveControl::calcRefTraj(State *state, double dl, int pIdx,
                                        double **xRef, int numNeighbors) const {
    int nCourse = static_cast<int>(this->splineCourse->size());
    int idx = this->calcNearestIndex(state, pIdx, numNeighbors);

    if (pIdx >= idx) idx = pIdx;

    xRef[0][0] = this->splineCourse->at(idx).at(0);
    xRef[1][0] = this->splineCourse->at(idx).at(1);
    xRef[2][0] = this->speedProfile->at(idx);
    xRef[3][0] = this->splineCourse->at(idx).at(2);

    double travel = 0;

    for (int i = 0; i < this->T + 1; i++) {
        travel += std::abs(state->v) * this->dt;
        int dIdx = static_cast<int>(round(travel / dl));

        int fIdx = idx + dIdx;
        if (fIdx >= nCourse) fIdx = nCourse - 1;

        xRef[0][i] = this->splineCourse->at(fIdx).at(0);
        xRef[1][i] = this->splineCourse->at(fIdx).at(1);
        xRef[2][i] = this->speedProfile->at(fIdx);
        xRef[3][i] = this->splineCourse->at(fIdx).at(2);
    }

    return idx;
}

void ModelPredictiveControl::updateState(State *state, double a,
                                         double delta) const {
    if (delta >= this->maxSteer)
        delta = this->maxSteer;
    else if (delta <= -this->maxSteer)
        delta = -this->maxSteer;

    state->x = state->x + state->v * cos(state->yaw) * this->dt;
    state->y = state->y + state->v * sin(state->yaw) * this->dt;
    state->yaw =
        state->yaw + state->v / this->wheelBase * tan(delta) * this->dt;
    state->v = state->v + a * this->dt;

    if (state->v > this->maxSpeed)
        state->v = this->maxSpeed;
    else if (state->v < this->minSpeed)
        state->v = this->minSpeed;
}

void ModelPredictiveControl::predictMotion(double **xBar, State *state,
                                           double *oA, double *oDelta) const {
    xBar[0][0] = state->x;
    xBar[1][0] = state->y;
    xBar[2][0] = state->v;
    xBar[3][0] = state->yaw;

    State *tempState = new State(state->x, state->y, state->yaw, state->v);

    for (int i = 1; i < this->T + 1; i++) {
        double ai = oA[i - 1];
        double di = oDelta[i - 1];
        this->updateState(tempState, ai, di);

        xBar[0][i] = tempState->x;
        xBar[1][i] = tempState->y;
        xBar[2][i] = tempState->v;
        xBar[3][i] = tempState->yaw;
    }

    delete tempState;
}

bool ModelPredictiveControl::linearMpcControl(double **xRef, double **xBar,
                                              State *state, double *oA,
                                              double *oDelta) {
    for (int i = 1; i <= this->T; i++) {
        for (int j = 0; j < this->NX; j++) {
            params.xRef[i][j] = xRef[j][i];
        }
    }

    // CVXGEN stores data structures in column-major form
    auto getAIdx = [this](int row, int col) { return row + col * this->NX; };

    auto getBIdx = [this](int row, int col) { return row + col * this->NX; };

    for (int t = 0; t < this->T; t++) {
        double v = xBar[2][t];
        double phi = xBar[3][t];
        double sinPhi = sin(phi);
        double cosPhi = cos(phi);

        params.A[t][getAIdx(0, 0)] = 1.0;
        params.A[t][getAIdx(0, 1)] = 0.0;
        params.A[t][getAIdx(0, 2)] = this->dt * cosPhi;
        params.A[t][getAIdx(0, 3)] = -this->dt * v * sinPhi;
        params.A[t][getAIdx(1, 0)] = 0.0;
        params.A[t][getAIdx(1, 1)] = 1.0;
        params.A[t][getAIdx(1, 2)] = this->dt * sinPhi;
        params.A[t][getAIdx(1, 3)] = this->dt * v * cosPhi;
        params.A[t][getAIdx(2, 0)] = 0.0;
        params.A[t][getAIdx(2, 1)] = 0.0;
        params.A[t][getAIdx(2, 2)] = 1.0;
        params.A[t][getAIdx(2, 3)] = 0.0;
        params.A[t][getAIdx(3, 0)] = 0.0;
        params.A[t][getAIdx(3, 1)] = 0.0;
        params.A[t][getAIdx(3, 2)] = 0.0;
        params.A[t][getAIdx(3, 3)] = 1.0;

        params.B[t][getBIdx(0, 0)] = 0.0;
        params.B[t][getBIdx(0, 1)] = 0.0;
        params.B[t][getBIdx(1, 0)] = 0.0;
        params.B[t][getBIdx(1, 1)] = 0.0;
        params.B[t][getBIdx(2, 0)] = this->dt;
        params.B[t][getBIdx(2, 1)] = 0.0;
        params.B[t][getBIdx(3, 0)] = 0.0;
        params.B[t][getBIdx(3, 1)] = this->dt * v / this->wheelBase;

        params.C[t][0] = this->dt * v * sinPhi * phi;
        params.C[t][1] = -this->dt * v * cosPhi * phi;
        params.C[t][2] = 0.0;
        params.C[t][3] = 0.0;
    }

    params.x0[0] = state->x;
    params.x0[1] = state->y;
    params.x0[2] = state->v;
    params.x0[3] = state->yaw;

    int numIters = solve();
    if (unlikely(work.converged != 1)) {
        info("Couldn't solve the optimization problem. numIters:%d", numIters);
        return false;
    }

    for (int i = 0; i < this->T; i++) {
        oA[i] = vars.u[i][0];
        oDelta[i] = vars.u[i][1];
    }

    return true;
}

bool ModelPredictiveControl::iterativeMpcControl(double **xRef, State *state,
                                                 bool validO, double *oA,
                                                 double *oDelta, int maxIters) {
    if (!validO) {
        std::fill(oA, oA + this->T, 0);
        std::fill(oDelta, oDelta + this->T, 0);
    }

    for (int i = 0; i < maxIters; i++) {
        double **xBar = new double *[this->NX];
        for (int j = 0; j < this->NX; j++) {
            xBar[j] = new double[this->T + 1]();
        }

        this->predictMotion(xBar, state, oA, oDelta);
        double *prevOA = new double[this->T];
        double *prevOD = new double[this->T];
        std::copy(oA, oA + this->T, prevOA);
        std::copy(oDelta, oDelta + this->T, prevOD);

        validO = this->linearMpcControl(xRef, xBar, state, oA, oDelta);
        if (validO) {
            double dU = 0;
            for (int j = 0; j < this->T; j++) {
                dU += std::abs(oA[j] - prevOA[j]) +
                      std::abs(oDelta[j] - prevOD[j]);
            }
            if (dU <= this->duTh) break;
        }

        for (int j = 0; j < this->NX; j++)
            delete[] xBar[j];
        delete[] xBar;
        delete[] prevOA;
        delete[] prevOD;

        if (!validO) return false;
    }

    assert(validO);
    return true;
}

void ModelPredictiveControl::smoothYaw() const {
    for (int i = 0; i < static_cast<int>(this->splineCourse->size()) - 1; i++) {
        double yaw0 = this->splineCourse->at(i).at(2);
        double yaw1 = this->splineCourse->at(i + 1).at(2);
        double dYaw = yaw1 - yaw0;

        while (dYaw >= PI / 2) {
            this->splineCourse->at(i + 1).at(2) -= TWO_PI;
            yaw1 = this->splineCourse->at(i + 1).at(2);
            dYaw = yaw1 - yaw0;
        }

        while (dYaw <= -PI / 2) {
            this->splineCourse->at(i + 1).at(2) += TWO_PI;
            yaw1 = this->splineCourse->at(i + 1).at(2);
            dYaw = yaw1 - yaw0;
        }
    }
}
