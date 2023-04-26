#include "PID.h"

void PID::initPID(float p, float i, float d, float freq) {
    Kp = p;
    Ki = i;
    Kd = d;
    loop.attach_us(callback(this, &PID::update), 1000000/freq);
}

void PID::update() {
    PID_flag = true;
}

float PID::updatePID(float process_variable, float set_point) {
    if (PID_flag) return out;
    error = (set_point - process_variable) / error_division_factor;
    error_d = error - p_error;
    error_sum += error * Ki;
    p_error = error;
    if (error_sum > max) error_sum = max;
    else if (error_sum < -max) error_sum = -max;
    out = error * Kp + error_d * Kd + error_sum;
    if (out > max) out = max;
    else if (out < -max) out = -max;
    else if (out < dead_zone && out > -dead_zone) out = 0;
    //if (.speed < 0) .set_point += 0.0015;
    //if (.speed > 0) .set_point -= 0.0015;
    PID_flag = false;
    return out;
}
