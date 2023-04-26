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

float update_PID() {
    if (PID_flag) return out;
    pid_loop->error = (pid_loop->set_point - cur_val) / pid_loop->error_division_factor;
    pid_loop->error_d = pid_loop->error - pid_loop->p_error;
    pid_loop->error_sum += pid_loop->error * pid_loop->Ki;
    pid_loop->p_error = pid_loop->error;
    if (pid_loop->error_sum > pid_loop->max) pid_loop->error_sum = pid_loop->max;
    else if (pid_loop->error_sum < -pid_loop->max) pid_loop->error_sum = -pid_loop->max;
    pid_loop->out = pid_loop->error * pid_loop->Kp + pid_loop->error_d * pid_loop->Kd + pid_loop->error_sum;
    if (pid_loop->out > pid_loop->max) pid_loop->out = pid_loop->max;
    else if (pid_loop->out < -pid_loop->max) pid_loop->out = -pid_loop->max;
    else if (pid_loop->out < pid_loop->dead_zone && pid_loop->out > -pid_loop->dead_zone) pid_loop->out = 0;
    //if (pid_loop.speed < 0) pid_loop.set_point += 0.0015;
    //if (pid_loop.speed > 0) pid_loop.set_point -= 0.0015;
    pid_loop->PID_flag = false;
    return pid_loop->out;
}

// float updatePID(struct PID *pid_loop, float cur_val) {
//   if (!pid_loop->PID_flag) return pid_loop->out;
//   pid_loop->error = (pid_loop->set_point - cur_val) / pid_loop->error_division_factor;
//   pid_loop->error_d = pid_loop->error - pid_loop->p_error;
//   pid_loop->error_sum += pid_loop->error * pid_loop->Ki;
//   pid_loop->p_error = pid_loop->error;
//   if (pid_loop->error_sum > pid_loop->max) pid_loop->error_sum = pid_loop->max;
//   else if (pid_loop->error_sum < -pid_loop->max) pid_loop->error_sum = -pid_loop->max;
//   pid_loop->out = pid_loop->error * pid_loop->Kp + pid_loop->error_d * pid_loop->Kd + pid_loop->error_sum;
//   if (pid_loop->out > pid_loop->max) pid_loop->out = pid_loop->max;
//   else if (pid_loop->out < -pid_loop->max) pid_loop->out = -pid_loop->max;
//   else if (pid_loop->out < pid_loop->dead_zone && pid_loop->out > -pid_loop->dead_zone) pid_loop->out = 0;
//   //if (pid_loop.speed < 0) pid_loop.set_point += 0.0015;
//   //if (pid_loop.speed > 0) pid_loop.set_point -= 0.0015;
//   pid_loop->PID_flag = false;
//   return pid_loop->out;
// }
