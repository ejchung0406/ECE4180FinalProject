#include "PID.h"

void PID::initPID(float p, float i, float d, float period) {
    Kp = p;
    Ki = i;
    Kd = d;
    Ticker loop;
    loop.attach(&PID::update, period);

}

void PID::update() {
    PID_flag = true;
}

float PID::update_PID() {
    if (PID_flag) return out;
    error = (set_point - cur_val) / error_division_factor;
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

// float updatePID(struct PID *, float cur_val) {
//   if (!PID_flag) return out;
//   error = (set_point - cur_val) / error_division_factor;
//   error_d = error - p_error;
//   error_sum += error * Ki;
//   p_error = error;
//   if (error_sum > max) error_sum = max;
//   else if (error_sum < -max) error_sum = -max;
//   out = error * Kp + error_d * Kd + error_sum;
//   if (out > max) out = max;
//   else if (out < -max) out = -max;
//   else if (out < dead_zone && out > -dead_zone) out = 0;
//   //if (.speed < 0) .set_point += 0.0015;
//   //if (.speed > 0) .set_point -= 0.0015;
//   PID_flag = false;
//   return out;
// }
