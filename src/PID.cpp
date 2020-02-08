#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, bool twiddle_, int twiddleSteps_, int warmupSteps_, double tolerance_) {
	Kp = Kp_;
       	Ki = Ki_;
       	Kd = Kd_;
       	twiddle = twiddle_;
	twiddleSteps = twiddleSteps_;
	warmupSteps = warmupSteps_;
	p_error = 0.0;
       	i_error = 0.0;
       	d_error = 0.0;
	tolerance_ = tolerance;
	numSteps = 0;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  i_error += cte;
  p_error = cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return total_error; 
}

double PID::ControlOutput(double cte) {

  UpdateError(cte);
  double output = -Kp * p_error - Kd * d_error - Ki * i_error;
  if (output < -1) {
    output = -1;
  }
  if (output > 1) {
    output = 1;
  }

  numSteps++;

  if (twiddle && numSteps == (warmupSteps + twiddleSteps)) {
    numSteps = 0;
    RestartSimulator();
    p_error = 0;
    i_error = 0;
    d_error = 0;
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

void PID::SetWebsocket(uWS::WebSocket<uWS::SERVER>& ws_) {
  ws = ws_;
}

void PID::Twiddle() {
}
