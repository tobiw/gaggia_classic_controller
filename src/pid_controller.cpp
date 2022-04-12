#include "pid_controller.h"

PidController::PidController()
{
    Kp = 0;
    Ki = 0;
    Kd = 0;
    sample_time = 0;
    last_time = 0;
    reset();
}

void PidController::reset()
{
    set_point = 0;
    Pterm = Iterm = Dterm = 0;
    windup_guard = 20;
    output = 0;
}

/* Calculates PID value for given reference feedback
 * u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
 * Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)
 * loop:
   error := setpoint − measured_value
   proportional := error;
   integral := integral + error × dt
   derivative := (error − previous_error) / dt
   output := Kp × proportional + Ki × integral + Kd × derivative
   previous_error := error
   wait(dt)
   goto loop
 **/
double PidController::update(int32_t measured_value, uint32_t current_time)
{
    const int32_t error = set_point - measured_value;

    const uint32_t delta_time = current_time - last_time;
    const double delta_error = error - last_error;

    // ???
    //if (delta_time < sample_time) return output;

    if (delta_time >= sample_time)
    {
        Pterm = error;
        Iterm += error * delta_time;
    }

    if (Iterm < -windup_guard)
    {
        Iterm = -windup_guard;
    }
    else if (Iterm > windup_guard)
    {
        Iterm = windup_guard;
    }

    Dterm = 0;
    if (delta_time > 0) // gets calculated even if between samples
    {
        Dterm = delta_error / delta_time;
    }

    last_time = current_time;
    last_error = error;

    output = (Kp * Pterm) + (Ki * Iterm) + (Kd * Dterm);
    return output;
}
