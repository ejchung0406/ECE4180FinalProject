#include "PID.h"
#include <cstdint>

void PID::initPID(float p, float i, float d, uint16_t dz, uint16_t MAX) {
    Kp = p;
    Ki = i;
    Kd = d;
    dead_zone = dz;
    max = MAX;
    PID_elapsed_time = us_ticker_read();
}

int PID::updatePID(float process_variable, float set_point) {
    float dt = us_ticker_read() - PID_elapsed_time;
    PID_elapsed_time = us_ticker_read();
    error = set_point - process_variable;
    error_d = (error - p_error) / dt;
    error_sum += error * dt * Ki;
    p_error = error;
    if (error_sum > max) error_sum = max;
    else if (error_sum < -max) error_sum = -max;
    out = error * Kp + error_d * Kd + error_sum;
    if (out > max) out = max;
    else if (out < -max) out = -max;
    else if (out < dead_zone && out > -dead_zone) out = 0;
    //if (.speed < 0) .set_point += 0.0015;
    //if (.speed > 0) .set_point -= 0.0015;
    return out;
}

void PID::restartPID() {
    error_sum = 0;
    p_error = 0;
}
