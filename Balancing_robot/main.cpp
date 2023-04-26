#include "mbed.h"
#include "PID.h"

Ticker step;

DigitalOut STEP_L(p21);
DigitalOut DIR_L(p19);
// DigitalOut STEP_R(p14);
// DigitalOut DIR_R(p15);

PID angle_loop;

float pitch_angle = 0;

int speed = 50;

uint32_t step_value = 1000;

void step_gen() {
    //STEP_L = !STEP_L;
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
    angle_loop.initPID(20, 0.1, 5, 250);
    step.attach_us(&step_gen, 1000);
    while (true) {
        speed = angle_loop.updatePID(20, 0);
        step_value = 10000/speed;
        DIR_L = 0;
    }
}


