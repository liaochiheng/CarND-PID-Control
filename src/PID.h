#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Kd;
  double Ki;

  /*
  * Twiddle variables
  */
  bool twiddle_switch;
  // Twiddle iterator number
  int twiddle_it;
  // Number of Calculate-error in one run
  int twiddle_num;
  // Index of Calculate-error in one run
  int twiddle_idx;
  // Index of twiddle_P
  int twiddle_idx_p;
  // Current add-up error in one run
  double twiddle_err;
  // Is twiddle down?
  bool twiddle_down;
  // 
  double twiddle_p[3];
  double twiddle_dp[3];
  double twiddle_p_best[3];
  double twiddle_err_best;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Kd, double Ki);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Initialize twiddle.
  */
  void Twiddle_Init(bool tw_switch, int tw_num);

  /*
  * Twiddle on each run.
  */
  bool Twiddle(double cte);

  /*
  * Twiddle next run.
  */
  void Twiddle_NextRun();
};

#endif /* PID_H */
