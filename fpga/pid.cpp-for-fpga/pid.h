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

class PID {
 public:
     PID(double Kp, double Ki, double Kd)
         : Kp(Kp), Ki(Ki), Kd(Kd), previous_error(0.0), integral(0.0) {}

     double calculate(double setpoint, double actual_value) {
         double error = setpoint - actual_value;
         integral += error;  // Integration with respect to time
         double derivative = error - previous_error;

         double output = Kp * error + Ki * integral + Kd * derivative;

         previous_error = error;

         return output;
     }

     void reset() {
         previous_error = 0.0;
         integral = 0.0;
     }

     void setGains(double Kp, double Ki, double Kd) {
         this->Kp = Kp;
         this->Ki = Ki;
         this->Kd = Kd;
     }

 private:
     double Kp;   // Proportional gain
     double Ki;   // Integral gain
     double Kd;   // Derivative gain

     double previous_error;
     double integral;
};
