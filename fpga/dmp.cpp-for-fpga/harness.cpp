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

#include <math.h>
#include <fstream>
#include <string>
#include <vector>


int main() {
    double dt = 0.001;

    double q0 = 0;
    double g = 1;
    double T = 150;

    double q = q0;
    double qd = 0, qdd;
    double t = 0;

    constexpr const int nrBasis = 15;
    double K = 25.0 * 25 / 4;
    double B = 25.0;

    double C[nrBasis];
    double weights[nrBasis];
    for (int i = 0; i < nrBasis; i++) {
        C[i] = 1.0 * i / (nrBasis - 1);
        weights[i] = 1.0 * i / nrBasis;
    }

    // Basis function widths
    double H = 0.65 * (1.0/(nrBasis-1.0)) * (1.0/(nrBasis-1.0));

    while (t <= T) {
        t += dt;

        // Compute the basis function values and force term
        double sum = 0;
        double Phi[nrBasis];

        for (int i = 0; i < nrBasis; i++) {
            double p = exp(-0.5 * ((t/T-C[i])*(t/T-C[i]) / H));
            Phi[i] = p;
            sum += p;
        }

        for (int i = 0; i < nrBasis; i++) {
            Phi[i] /= sum;
        }

        double f = 0;
        for (int i = 0; i < nrBasis; i++) {
            f += Phi[i] * weights[i];
        }

        qdd = K*(g-q)/(T*T) - B*qd/T + (g-q0)*f/(T*T);
        qd += qdd*dt;
        q += qd*dt;
    }

    return 0;
}
