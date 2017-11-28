#ifndef PID_H
#define PID_H

#include <limits>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double s_error;

  /*
  * Coefficients
  */ 
  double K[3];
    double dp[3];
//  double Ki;
//  double Kd;

    double best_lap = 0;

    u_long step =0;

  /*
  * Constructor
  */
  PID(double Kp, double Ki, double Kd);

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte, double speed);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

    bool shouldRestart(double d);

    void restart();

    int it, tuned;

    enum ParamsState { inc_param, calc_best_err_after_inc, calc_best_err_after_dec };
    int state_ = inc_param; // 0-run after inc pd, 1-run after decrease dp
    double s_speed = 0;

    void next_param();

    void save_best(double speed);
};

#endif /* PID_H */
