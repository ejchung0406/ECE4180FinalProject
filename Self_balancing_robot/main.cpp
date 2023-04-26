#include "mbed.h"

Ticker PID;
Ticker step;

DigitalOut STEP_L(p12);
DigitalOut DIR_L(p13);
// DigitalOut STEP_R(p14);
// DigitalOut DIR_R(p15);

float angle_P = 20;
float angle_I = 1;
float angle_D = 5;
float angle_error = 0;
float angle_setpoint = 0;
float pitch_angle = 0;

int speed = 0;

uint32_t step_value = 1000;

float constrain(float input, float min, float max) {
    if (input >= max)
        return max;
    if (input <= min)
        return min;
    return input;
}

float angle_loop() {
    static float prev_error = 0;
    static float error_sum = 0;
    static float output = 0;
    error_sum += angle_error;
    output = angle_P*angle_error + angle_I*error_sum + angle_D*(angle_error-prev_error);
    output = constrain(output, -400, 400);
    return output;
}

void step_gen() {
    static uint32_t counter = 0;
    if (counter >= step_value) {
        counter = 0;
        STEP_L = !STEP_L;
    }
    counter++;
}


// main() runs in its own thread in the OS
int main()
{
    PID.attach(&angle_loop)
    step.attach(&step_gen, 100000);
    while (true) {
        angle_error = pitch_angle - angle_setpoint;
        speed = angle_loop();
        DIR_L = speed > 0;
    }
}

