#ifndef __HEATER_CONTROLLER_H__
#define __HEATER_CONTROLLER_H__

// Can use PID or a simple on/off based on temperature
class HeaterController {
    public:
        HeaterController();
        void set_target(int t);
        void update(int current_temperature);
        int get_output();
    private:
        int current_temperature;
        int target_temperature;
};

#endif
