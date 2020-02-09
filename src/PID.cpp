#include "PID.h"

#include <cmath>
#include <limits>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, bool twiddle_,
               int twiddleSteps_, int warmupSteps_, double tolerance_) {
  coeffs = {Kp_, Ki_, Kd_};
  for (int i = 0; i < coeffs.size(); ++i) {
    deltas.push_back(0.1 * coeffs[i]);
    errors.push_back(0.0);
  }

  twiddle = twiddle_;
  tolerance = tolerance_;
  twiddleSteps = twiddleSteps_;
  warmupSteps = warmupSteps_;
  tolerance_ = tolerance;
  numSteps = 0;
  twiddleStage = 0;
  twiddleParam = 0;
  lastTwiddleDone = false;
  bestError = std::numeric_limits<double>::max();
}

void PID::UpdateError(double cte) {
  errors[2] = cte - errors[0];
  errors[1] += cte;
  errors[0] = cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return total_error;
}

double PID::ControlOutput(double cte) {

  UpdateError(cte);
  double output =
      -coeffs[0] * errors[0] - coeffs[2] * errors[2] - coeffs[1] * errors[1];
  if (output < -1) {
    output = -1;
  }
  if (output > 1) {
    output = 1;
  }

  numSteps++;

  if (twiddle && numSteps == (warmupSteps + twiddleSteps)) {
    std::cout << "\n\nFinal coefficients: ";
    std::cout << "Kp = " << coeffs[0] << ", Ki = " << coeffs[1]
              << ", Kd = " << coeffs[2];
    std::cout << ", Total error = " << total_error << "\n" << std::endl;
    Twiddle();
    // lastTwiddleDone = true;
    numSteps = 0;
    RestartSimulator();
    for (int i = 0; i < errors.size(); ++i) {
      errors[i] = 0.0;
    }
  }

  return output;
}

/*
 * Restart simulator to initial conditions (for twiddle)
 */
void PID::RestartSimulator() {
  std::string reset_txt = "42[\"reset\",{}]";
  ws.send(reset_txt.data(), reset_txt.length(), uWS::OpCode::TEXT);
}

void PID::SetWebsocket(uWS::WebSocket<uWS::SERVER> &ws_) { ws = ws_; }

void PID::Twiddle() {
  if (lastTwiddleDone) {
    double totalD = 0.0;
    for (int i = 0; i < deltas.size(); ++i) {
      totalD += fabs(deltas[i]);
    }
    std::cout << "totalD: " << totalD << ", tolerance: " << tolerance
              << std::endl;
    if (totalD < tolerance) {
      std::cout << "Optimization finished. Exiting" << std::endl;
      exit(0);
    }
    twiddleStage = 0;
  }
  // TODO: fix this;
  double currError = 0.0;
  bool isBetter = currError < bestError;
  switch (twiddleStage) {
  case 0:
    if (isBetter) {
      bestError = currError;
      // TODO: Assign current updates to best Kp, Ki, Kd
      deltas[twiddleParam] *= 1.1;
      lastTwiddleDone = true;
    } else {
      coeffs[twiddleParam] -= 2 * deltas[twiddleParam];
      twiddleStage = 1;
    }
    break;
  case 1:
    if (isBetter) {
      bestError = currError;
      deltas[twiddleParam] *= 1.1;
      lastTwiddleDone = true;
    } else {
      coeffs[twiddleParam] += deltas[twiddleParam];
      deltas[twiddleParam] *= 0.9;
    }
    twiddleStage = 0;
    lastTwiddleDone = true;
    break;
  }
}
