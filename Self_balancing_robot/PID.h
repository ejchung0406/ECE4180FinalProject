#include "mbed.h"

class PID {
    public: 
        float Kp, Ki, Kd;
        void initPID(float p, float i, float d, float period);
    private:
        uint16_t max, error_division_factor;
        double set_point, dead_zone, error, p_error, error_d, error_sum, out;
        volatile bool PID_flag;
        uint32_t PID_elapsed_time;
        void update();
        float updatePID();
};
