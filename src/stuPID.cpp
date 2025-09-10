#include <Arduino.h>
#include <stuPID.h>



PID_ctrl::PID_ctrl(double min, double max, uint16_t T_s, double Kp, double Ki, double Kd){
  out_bounds[0] = min;
  out_bounds[1] = max;
  this->T_s = T_s;
  K[0] = Kp;
  K[1] = Ki;
  K[2] = Kd;
  I = 0;
}




double PID_ctrl::compute(double current_state){
  //I = 0.0; //The overall I term
  double error = setpoint - current_state;

  // Terms
  double P = K[0] * error;
  double dI = K[1] * error * ((double)T_s/(1000.0));
  double D = -K[2] * (current_state - last_state) / ((double)T_s/(1000.0));  

  //Calculates a candidate for the resultthe result should be.
  double result = P + (dI + I) + D; 

  I = I +dI;
  if(K[1] != 0){

  I = min(max(I,out_bounds[0]/K[1]*I_percent_bound), out_bounds[1]/K[1]*I_percent_bound);

  }

  /*
  //Probably making things worse, but it should make it so we dont continue to integrate if the system already is responding at its max
  if(I <= 2*out_bounds[0]){ //If we're too low
    I = I + min(dI, 0.0); //We only integrate if it helps us go up
  }
  else if(I >= 2*out_bounds[1]){ //If we're too high
    I = I + max(dI,0.0); //We only integrate if it helps to go down
  }
  else{ //If none of the above conditions apply, just go ahead and integrate
    I = I + dI;
  }
    */

  last_state = current_state;
  //clamps and returns what our output should be
  output = min(max(result, out_bounds[0]),out_bounds[1]); 
  return output; 
}




/*********
 * Cat-gpt, made the following diagram cause why not... 
 *  ／l、
 *（ﾟ､ ｡７_
 *  l、ﾞ~ ヽ
 *  じしf_, )ノ 
                 +------------------+
                 |   Setpoint       |
                 +------------------+
                           |
                           v
                     +-----------+
                     |   Error   |  <- Error = setpoint - current_state
                     +-----------+
                           |
          +----------------+----------------+
          |                |                |
          v                v                v
      +-------+        +---------+      +-----------+
      |   P   |        |   I     |      |    D      |
      | K[0]* |        | K[1]*e* |      | -K[2]*(Δs)/T_s |
      | error |        | dt      |      |           |
      +-------+        +---------+      +-----------+
          |                |                |
          +--------+-------+----------------+
                   |
                   v
             +-----------+
             | Candidate |
             |  Result   |  <- P + (I+dI) + D
             +-----------+
                   |
           +-------+-------+
           | Check Output  |
           | Saturation    |
           +-------+-------+
                   |
         +---------+---------+
         | Clamp to Out_bounds |
         +-------------------+
                   |
                   v
               +-------+
               |Output |
               +-------+
****/