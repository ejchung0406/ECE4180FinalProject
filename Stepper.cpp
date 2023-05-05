#include "Stepper.h"

Stepper::Stepper(PinName step_pin_L, PinName dir_pin_L, PinName step_pin_R, PinName dir_pin_R)
    :step_L(step_pin_L), dir_L(dir_pin_L), step_R(step_pin_R), dir_R(dir_pin_R) {
    LPC_GPIO2->FIODIR |= (1<<5);
    stepTick.attach_us(this, &Stepper::stepGen, 20);
}

void Stepper::setSpeed(int spd_L, int spd_R) {
    speed_L = spd_L;
    if (speed_L)
        step_duration_L = 5000/abs(speed_L);
    speed_R = spd_R;
    if (speed_R)
        step_duration_R = 5000/abs(speed_R);
    dir_L = speed_L > 0;
    dir_R = speed_R > 0;
}

void Stepper::stepGen() {
    if (speed_L) {
        if (L_counter < step_duration_L)
            L_counter++;
        else if (L_counter == step_duration_L) {
            LPC_GPIO2->FIOSET |= (1<<5);
            L_counter = 0;
        }
        else
            L_counter = 0;
        if (L_counter == 1)
            LPC_GPIO2->FIOCLR |= (1<<5);
    }
    else 
        L_counter = 0;
}