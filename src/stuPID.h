/*=================================================
* stuPID, another PID.... 
* some code & knowlegde most graciously stolen from:
* - Lasse Værum
* - David Sechardes
* Thx you guys <3
* xoxo Malthe
*==================================================
*/



/// @brief The PID control class, should be run at a fixed regularly, ideal for KRNL and alike...
struct PID_ctrl
{
  
  private:
    double output; //the output variable 
    double out_bounds[2]; //[0] minium, [1] maximum
    double K[3]; //constants, P,I,D
    double setpoint; //The point we want to be at
    double last_state; //The last errror of the system
    uint16_t T_s; //Sample period in ms, how often the system is run.
    double I_percent_bound = 0.2; //A limit for the I therm, so we dont windup. scales with the allowed output
  public:
    double I;   // I term

    /// @brief Gets the output
    /// @return The output variable within specified bounds determined by the controller 
    double get_output() const { return output;} 
    void set_parameters(double K[3]) {this->K[0] = K[0]; this->K[1] = K[1]; this->K[2] = K[2];};
    double get_windup_limit() {return I_percent_bound;}
    double set_windup_limit(double input) {this->I_percent_bound = input;}
    /// @brief Sets the setpoint
    /// @param input The state we want the system to be in, e.g a target position, speed, something like that.  
    void set_point(double input) {setpoint = input;}
    /// @brief computes what the output should be, given the current positions
    /// @param input The current state of the system, e.g posistion, speed, something like that 
    /// @return The output variables
    double compute(double input);

    /// @brief Initalizes the controll with given bounds and samplerate, optional paramters
    /// @param min minimum output
    /// @param max maxmimum output
    /// @param T_s Sample period in ms
    /// @param Kp Proportional parameter
    /// @param Ki Integral parameter
    /// @param Kd Derivative parameter
    PID_ctrl(double min, double max, uint16_t T_s ,double Kp=1.0, double Ki=0.5, double Kd=0.0);

};
