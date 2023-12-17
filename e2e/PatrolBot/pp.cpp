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

#include "pp.h"

PurePursuit::PurePursuit(const std::vector<Point> &path,
                         double lookAheadDistance)
    : path(path), lookAheadDistance(lookAheadDistance) {}

Point PurePursuit::getLookAheadPoint(const Point &robotPosition) const {
    Point lookAhead;
    double closestDist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < path.size() - 1; i++) {
        Point A = path[i];
        Point B = path[i + 1];

        // Calculate intersection of the circle with line segment
        double a = std::pow(B.x - A.x, 2) + std::pow(B.y - A.y, 2);
        double b = 2 * ((B.x - A.x) * (A.x - robotPosition.x) +
                        (B.y - A.y) * (A.y - robotPosition.y));
        double c = std::pow(A.x - robotPosition.x, 2) +
                   std::pow(A.y - robotPosition.y, 2) -
                   std::pow(lookAheadDistance, 2);

        double discriminant = b * b - 4 * a * c;

        if (discriminant < 0) {
            continue; // No intersection
        }

        double t1 = (-b - std::sqrt(discriminant)) / (2 * a);
        double t2 = (-b + std::sqrt(discriminant)) / (2 * a);

        if (t1 >= 0 && t1 <= 1) {
            Point candidate = {A.x + t1 * (B.x - A.x), A.y + t1 * (B.y - A.y)};

            double distance = robotPosition.distanceTo(candidate);
            if (distance < closestDist) {
                closestDist = distance;
                lookAhead = candidate;
            }
        }

        if (t2 >= 0 && t2 <= 1) {
            Point candidate = {A.x + t2 * (B.x - A.x), A.y + t2 * (B.y - A.y)};

            double distance = robotPosition.distanceTo(candidate);
            if (distance < closestDist) {
                closestDist = distance;
                lookAhead = candidate;
            }
        }
    }

    return lookAhead;
}

// This function computes the curvature to achieve the desired heading to
// the look-ahead point.
double PurePursuit::computeCurvature(const Point &robotPosition,
                                     double robotOrientation) const {
    Point lookAheadPoint = getLookAheadPoint(robotPosition);
    double x = lookAheadPoint.x - robotPosition.x;
    double y = lookAheadPoint.y - robotPosition.y;

    double ld2 = x * x + y * y;
    double yRelative =
        -std::cos(robotOrientation) * y + std::sin(robotOrientation) * x;

    double curvature = 2 * yRelative / ld2;
    return curvature;
}
