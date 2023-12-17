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

#include "env.h"
#include "rowild_utils.h"
#include <vector>

#define MAX_ANGLE (TWO_PI)
#define MIN_ANGLE (0.0)

class Planner {
  public:
    Planner(const Environment *env, int dof, int linkLength, double epsilon);
    virtual std::vector<double *> query(double *start, double *goal) = 0;
    double calcPathCost(std::vector<double *> const &path) const;

  protected:
    double getNorm(double *cfg1, double *cfg2) const;
    double *generateRandomCfg() const;
    double getDistance(double *cfg1, double *cfg2, bool *bitmap) const;
    double *getMiddleCfg(double *cfg1, double *cfg2, int sampleNumber,
                         int totalSamples, bool *bitmap) const;
    bool isValidArmConfiguration(double *angles) const;
    bool equalCfgs(double *cfg1, double *cfg2) const;
    bool canConnect(double *cfg1, double *cfg2) const;

    int dof;
    double epsilon;

  private:
    typedef struct {
        int x1, y1;
        int x2, y2;
        int increment;
        int usingYIndex;
        int deltaX, deltaY;
        int dTerm;
        int incrE, incrNE;
        int xIndex, yIndex;
        int flipped;
    } Bresenham_Param;

    void contXY2Cell(double x, double y, uint16_t *pX, uint16_t *pY) const;
    void getBresenhamParameters(int p1x, int p1y, int p2x, int p2y,
                                Bresenham_Param *params) const;
    void getCurrentPoint(Bresenham_Param *params, int *x, int *y) const;
    bool getNextPoint(Bresenham_Param *params) const;
    bool isValidLineSegment(double x0, double y0, double x1, double y1) const;

    const Environment *env;
    int linkLength, envX, envY;
};
