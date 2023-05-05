#include "mbed.h"

class Stepper {
    public: 
        Stepper(PinName step_pin_L, PinName dir_pin_L, PinName step_pin_R, PinName dir_pin_R);
        void setSpeed(int spd_L, int spd_R);
    private:
        Ticker stepTick;
        DigitalOut step_L;
        DigitalOut dir_L;
        DigitalOut step_R;
        DigitalOut dir_R;
        int speed_L;
        int speed_R;
        uint16_t step_duration_L;
        uint16_t step_duration_R;
        volatile uint16_t L_counter;
        volatile uint16_t R_counter;
        void stepGen();
};
