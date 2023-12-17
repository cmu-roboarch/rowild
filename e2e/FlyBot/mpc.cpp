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

MpcController::MpcController(const std::vector<double> &currentState,
                             const std::vector<double> &desiredState)
    : currentState(currentState), desiredState(desiredState) {}

bool MpcController::get_nlp_info(Index &n, Index &m, Index &nnzJacG,
                                 Index &nnzHLag, IndexStyleEnum &indexStyle) {
    n = controlSize;
    m = 0;
    nnzJacG = 0;
    nnzHLag = 0;
    indexStyle = TNLP::C_STYLE;
    return true;
}

bool MpcController::get_bounds_info(Index n, Number *xLower, Number *xUpper,
                                    Index m, Number *gLower, Number *gUpper) {
    for (Index i = 0; i < n; i++) {
        xLower[i] = -10.0;
        xUpper[i] = 10.0;
    }
    return true;
}

bool MpcController::get_starting_point(Index n, bool initX, Number *x,
                                       bool initZ, Number *zLower,
                                       Number *zUpper, Index m, bool initLambda,
                                       Number *lambda) {
    assert(initX == true);
    for (Index i = 0; i < n; i++) {
        x[i] = 0.0;
    }
    return true;
}

bool MpcController::eval_f(Index n, const Number *x, bool newX,
                           Number &objValue) {
    objValue = 0;
    std::vector<double> control(x, x + n);
    std::vector<double> nextState = droneDynamics(currentState, control);
    for (int i = 0; i < stateSize; ++i) {
        objValue +=
            (nextState[i] - desiredState[i]) * (nextState[i] - desiredState[i]);
    }
    return true;
}

bool MpcController::eval_grad_f(Index n, const Number *x, bool newX,
                                Number *gradF) {
    std::vector<double> control(x, x + n);
    std::vector<double> nextState = droneDynamics(currentState, control);
    for (Index i = 0; i < n; i++) {
        gradF[i] = 2 * (nextState[i] - desiredState[i]);
    }
    return true;
}

bool MpcController::eval_g(Index n, const Number *x, bool newX, Index m,
                           Number *g) {
    return true;
}

bool MpcController::eval_jac_g(Index n, const Number *x, bool new_x, Index m,
                               Index nele_jac, Index *iRow, Index *jCol,
                               Number *values) {
    return true;
}

void MpcController::finalize_solution(SolverReturn status, Index n,
                                      const Number *x, const Number *z_L,
                                      const Number *z_U, Index m,
                                      const Number *g, const Number *lambda,
                                      Number obj_value,
                                      const IpoptData *ip_data,
                                      IpoptCalculatedQuantities *ip_cq) {
    /*
    std::cout << "Solution of the primal variables, x:" << std::endl;
    for (Index i = 0; i < n; ++i) {
        std::cout << "x[" << i << "] = " << x[i] << std::endl;
    }
    std::cout << "Objective value: " << obj_value << std::endl;
    */
}
