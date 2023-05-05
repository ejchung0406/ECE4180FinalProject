#include "mbed.h"
#include <cstdint>

class PID {
    public: 
        float Kp, Ki, Kd;
        void initPID(float p, float i, float d, uint16_t dead_zone, uint16_t max);
        int updatePID(float process_variable, float set_point);
        void restartPID();
    private:
        uint16_t dead_zone, max;
        double error, p_error, error_d, error_sum;
        int out;
        uint32_t PID_elapsed_time;
};
