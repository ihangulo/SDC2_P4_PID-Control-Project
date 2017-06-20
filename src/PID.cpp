#include "PID.h"
#include <iostream>
using namespace std;
#define KP 0
#define KI 1
#define KD 2



/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
//  this->Kp = Kp;
//  this->Ki = Ki;
//  this->Kd = Kd;
  p[KP] = Kp;
  p[KI] = Ki;
  p[KD] = Kd;

  p_error =0.0;
  i_error =0.0;
  d_error =0.0;



  best_err = 1000000000; // big number
  total_err = 0; // total error
  depth = 0; // if .. else depth from twiddle algorithm



  total_count = 0;




  // for twiddle 

/*
  I use TWIDDLE to get best coefficients of PID controller.
  Starts from very short course, increase the time(max counts) until almost 1 lap.
  I used every previous best results as starting values.


*/



  if(TWIDDLE_MODE) {

/*
  1st values

  I checked some random values, I assumed.

  I got all values ABS are smaller than 1.
  And p[KI] must very small, because it is more and more increased.
  And p[KD] is not divided by (0.1sec). So It must be bigger than p[KP] about 10 times.

  So I put start values like below.

    p[KP] = .1;
    p[KI] = .01;
    p[KD] = .0;

    dp[KP] = .1;
    dp[KI] = 0.01;
    dp[KD] = 1.;

  Please see the file 'twiddle_log.txt'. It has full logs.

*/
// 2nd -- the best vaules of 1st try
//0.487765/0.00518348/1.8019  dp : 0.0630247 0.00630247 0.421901

// 3rd -- the best values of 2nd try
//  p 0.545736/0.00449651/2.56212  dp : 0.0555935 0.00555935 0.372155

// 4th -- the best values of 3rd try
// p 0.496042/0.00899958/2.93427  dp : 0.0441346 0.00361101 0.241729

// 5th -- the best values of 4th try
//  p 0.540177/0.00574606/2.75805  dp : 0.0353915 0.00432563 0.17622

    p[KP] = 0.540177;
    p[KI] = 0.00574606;
    p[KD] =2.75805;

    dp[KP] = 0.0353915;
    dp[KI] = 0.00432563;
    dp[KD] = 0.17622;


    epoch =0;


    depth=1;
    pid_num=0; // 0 = p / 1 = i / 2 = d
    p[pid_num] += dp[pid_num];
  }

}

void PID::UpdateError(double cte) {
        //total_count ++;

	d_error = cte - p_error; // dt =0.1sec , but ignore this.
	p_error = cte;
	i_error += cte;

	if(total_count++ > TWIDDLE_MIN_COUNT )
	  total_err += (cte*cte);


 
}

double PID::TotalError() {
   //return ( -Kp* p_error - Kd * d_error- Ki * i_error );
   return ( -p[KP]* p_error - p[KD] * d_error- p[KI] * i_error );

}

int PID::getTotalCount() {
  return total_count;
}

void PID::setTotalCount(int num) {
  total_count = num;

}

void PID::Twiddle() {

  err = total_err / (total_count- TWIDDLE_MIN_COUNT); 

  std::cout << "EPOCH "<< epoch++ ;
  std::cout <<" PID= " << pid_num <<" TWIDDLE " <<   p[0] <<":"<<p[1]<<":"<<p[2]<< " BEST ERR: "<< best_err<< " / " << err << std::endl;
  if(err<best_err) {

    std::cout << "[BEST]>>> p "<< p[0] << "/" << p[1] << "/" << p[2] << "  dp : " << dp[0] << " " << dp[1] << " " << dp[2] << std::endl;

  	best_err = err;
  	dp[pid_num] *=1.1;
   
    pid_num = (pid_num+1)%3;
    depth=1;
    p[pid_num] += dp[pid_num];



  } else {



    switch (depth) {
      case 1 :
        p[pid_num] -= (2* dp[pid_num]);
       
        depth=2;

   
        break;


      case 2 :
        p[pid_num] += dp[pid_num];
        dp[pid_num] *= 0.9;
       

        pid_num = (pid_num+1)%3;
        depth=1;
        p[pid_num] += dp[pid_num];
 

        break;

    }

  }

  p_error =0.0;
  i_error =0.0;
  d_error =0.0;
  total_err =0.0;

}


// https://discussions.udacity.com/t/twiddle-application-in-pid-controller/243427/9
//void PID::Restart(uWS::WebSocket<uWS::SERVER> ws){
//  std::string reset_msg = "42[\"reset\",{}]";
//  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
//}

