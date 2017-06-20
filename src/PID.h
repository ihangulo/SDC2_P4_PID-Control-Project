#ifndef PID_H
#define PID_H


#define TWIDDLE_MODE false  // change this true to twiddle mode


/*
  I use TWIDDLE to get best coefficients of PID controller.
  Starts from very short course, increase the time(max counts) until almost 1 lap.
  I used every previous best results as starting values.

*/

/* 1st
#define TWIDDLE_MAX_COUNT 100
#define TWIDDLE_MIN_COUNT 20
*/

/* 2nd
#define TWIDDLE_MAX_COUNT 300
#define TWIDDLE_MIN_COUNT 100
*/

/* 3rd
#define TWIDDLE_MAX_COUNT 550
#define TWIDDLE_MIN_COUNT 250
*/

/* 4th
#define TWIDDLE_MAX_COUNT 850 
#define TWIDDLE_MIN_COUNT 50
*/


#define TWIDDLE_MAX_COUNT 1200 // almost 1 lap
#define TWIDDLE_MIN_COUNT 50

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
//  double Kp;
//  double Ki;
//  double Kd;
  double p[3];
  double err;
  int total_count;
  int epoch;

 /*
  * Twiddle
  */
 
  double dp[3]; // dp[i]
  double best_err;
  double total_err; // total error 

  int depth;
  int pid_num; // 0 = p / 1 = i / 2 = d




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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  void Twiddle();

  int getTotalCount();
  void setTotalCount(int num);



  // restart simulator
  //void PID::Restart(uWS::WebSocket<uWS::SERVER> ws);
};

#endif /* PID_H */
