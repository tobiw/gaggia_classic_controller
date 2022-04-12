#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

typedef long int int32_t;
typedef unsigned long int uint32_t;

class PidController
{
public:
    PidController();
    void reset();
    double update(int32_t measured_value, uint32_t current_time);
    inline void set_setpoint(int32_t sp) { set_point = sp; }
    inline void set_pid_parameters(int32_t p, int32_t i, int32_t d) { Kp = p; Ki = i; Kd = d; }
    inline void set_sample_time(uint32_t t) { sample_time = t; }

protected:
    // PID configuration:
    double Kp; // Determines how aggressively the PID reacts to the current error with setting Proportional Gain
    double Ki; // Determines how aggressively the PID reacts to the current error with setting Integral Gain
    double Kd; // Determines how aggressively the PID reacts to the current error with setting Derivative Gain
    uint32_t sample_time; // How often to update the PID values
    /* Integral windup, also known as integrator windup or reset windup, refers
     * to the situation in a PID feedback controller where a large change in
     * setpoint occurs (say a positive change) and the integral terms
     * accumulates a significant error during the rise (windup), thus
     * overshooting and continuing to increase as this accumulated error is
     * unwound (offset by errors in the other direction). The specific problem
     * is the excess overshooting. */
    int32_t windup_guard;

    // Runtime values:
    uint32_t last_time;
    int32_t set_point;
    double last_error, Pterm, Iterm, Dterm, output;
};

#endif
