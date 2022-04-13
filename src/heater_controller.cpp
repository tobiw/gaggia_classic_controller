#include "heater_controller.h"

HeaterController::HeaterController()
{
    current_temperature = 0;
    target_temperature = 0;
}

void HeaterController::set_target(int t)
{
    target_temperature = t;
}

void HeaterController::update(int current_temp)
{
    // Use PID as required

    current_temperature = current_temp;
}

int HeaterController::get_output()
{
    // Use PID as required

    if (current_temperature < (target_temperature - 2))
    {
        return 1;
    }
    return 0;
}
