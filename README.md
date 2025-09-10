# stuPID
A flawed product of a deranged mind (homecooked PID controller)

Thanks to Lasse for a look at his arduino sketch as inspiration.
And Thanks to David for sitting down and blaming every questionable choice made <3.


# How to use
Inclue the lib by

```cpp
#include <Arduino.h>
#include <stuPID.h>

void main(){
    TickType_t lastWake = xTaskGetTickCount();

    int T_s = 10; //Sample period in miliseconds
    //            out: min max,  T_s   Kp  Ki  Kd 
    //                  V    V    V    V   V   V
    PID_ctrl ctrl(-155, 155, T_s, 0.6, 1, 0.12);


    double k = {0.1, 0.5, 0.1}
    //If you would want to set the parameters dynamically
    ctrl.set_parameters(k);

    //Setting the reference
    ctrl.set_point(0);


    while(true){

        double result = ctrl.compute(current_pos);
        //Do something with the result update motor or alike
        motor.set_speed(result);

        //Code to make sure we run at the expected sample frequency
        xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(T_s));

    }
}

```
