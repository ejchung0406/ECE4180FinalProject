#include "mbed.h"

class PID {
    public: 
        float Kp, Ki, Kd;
        void initPID(float p, float i, float d, float period);
        float updatePID(float process_variable, float set_point);
    private:
        uint16_t max, error_division_factor;
        double dead_zone, error, p_error, error_d, error_sum, out;
        volatile bool PID_flag;
        uint32_t PID_elapsed_time;
        void update();
        Ticker loop;
};
