#ifndef PID_H
#define PID_H

#include <vector>
#include <uWS/uWS.h>

using std::vector;

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_, bool twiddle, int twiddleSteps_, int warmupSteps, double tolerance_);

  /**
   *  Provide WebSocket handle
   */
  void setWebsocket(uWS::WebSocket<uWS::SERVER>& ws);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  /**
   * Obtain the control output.
   * @param the cross-track error.
   * @output control parameter (e.g. steering angle)
   */
  double ControlOutput(double cte);

  void SetWebsocket(uWS::WebSocket<uWS::SERVER>& ws);


 private:
  /**
   * Restart simulator to the initial conditions (for twiddle)
   */
  void RestartSimulator();

  void Twiddle();

  /**
   * PID Errors
   */
   vector<double> errors;

  /**
   * PID Coefficients
   */
   vector<double> prevCoeffs;
   vector<double> coeffs;
   vector<double> deltas;

  /*
   * Helper settings
   */
  double total_error;
  bool twiddle;
  int twiddleSteps;
  int warmupSteps;
  double tolerance;
  uWS::WebSocket<uWS::SERVER> ws;
  int numSteps;
  double dp;
  double di;
  double dd;
  int twiddleStage;
  int twiddleParam;
  bool lastTwiddleDone;
  double bestError;
};

#endif  // PID_H
