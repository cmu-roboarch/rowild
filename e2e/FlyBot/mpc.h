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

#include "coin/IpIpoptApplication.hpp"
#include "coin/IpTNLP.hpp"
#include "log.h"
#include <vector>

using namespace Ipopt;

const int stateSize = 3;
const int controlSize = 3;

template <typename T>
std::vector<T> droneDynamics(const std::vector<T> &state,
                             const std::vector<T> &control) {
    assert(state.size() == stateSize);
    assert(control.size() == controlSize);

    std::vector<T> newState(stateSize);
    for (int i = 0; i < stateSize; ++i) {
        newState[i] = state[i] + control[i];
    }
    return newState;
}

class MpcController : public TNLP {
  public:
    MpcController(const std::vector<double> &currentState,
                  const std::vector<double> &desiredState);

    virtual bool get_nlp_info(Index &n, Index &m, Index &nnzJacG,
                              Index &nnzHLag,
                              IndexStyleEnum &indexStyle) override;

    virtual bool get_bounds_info(Index n, Number *xLower, Number *xUpper,
                                 Index m, Number *gLower,
                                 Number *gUpper) override;

    virtual bool get_starting_point(Index n, bool initX, Number *x, bool initZ,
                                    Number *zLower, Number *zUpper, Index m,
                                    bool initLambda, Number *lambda) override;

    virtual bool eval_f(Index n, const Number *x, bool newX,
                        Number &objValue) override;

    virtual bool eval_grad_f(Index n, const Number *x, bool newX,
                             Number *gradF) override;

    virtual bool eval_g(Index n, const Number *x, bool newX, Index m,
                        Number *g) override;

    virtual bool eval_jac_g(Index n, const Number *x, bool new_x, Index m,
                            Index nele_jac, Index *iRow, Index *jCol,
                            Number *values) override;

    virtual void finalize_solution(SolverReturn status, Index n,
                                   const Number *x, const Number *z_L,
                                   const Number *z_U, Index m, const Number *g,
                                   const Number *lambda, Number obj_value,
                                   const IpoptData *ip_data,
                                   IpoptCalculatedQuantities *ip_cq) override;

  private:
    std::vector<double> currentState;
    std::vector<double> desiredState;
};
