#include "mbed.h"
#include "PID.h"
#include "Stepper.h"
#include "LSM9DS1.h"
#define PI 3.14159

Ticker step;

// DigitalOut STEP_R(p14);
// DigitalOut DIR_R(p15);

Serial pc(USBTX, USBRX);
PID angle_loop;
Stepper Steppers(p21, p19, p14, p15);

int speed = 0;

float pitch = 0;

float set_point = -4.2;

bool start = 0;

void comFilter(float ax, float ay, float az, float gx) {
    static float angle_accel = 0;
    static uint32_t gyro_elapsed_time = us_ticker_read();
    angle_accel = atan2(ay, az);
    angle_accel *= 180.0 / PI;
    pitch -= gx * (us_ticker_read() - gyro_elapsed_time) * 0.00000000747;
    gyro_elapsed_time = us_ticker_read();
    pitch = pitch * 0.996 + angle_accel * 0.004;
    // pc.printf("Pitch: %f, Angle accel: %f\n\r", pitch, angle_accel);
}


// main() runs in its own thread in the OS
int main()
{
    pc.baud(115200);
    LSM9DS1 IMU(p9, p10, 0xD6, 0x3C);
    IMU.begin();
    if (!IMU.begin()) {
        pc.printf("Failed to communicate with LSM9DS1.\n");
    }
    IMU.calibrate(1);
    // pc.printf("\nIMU.gBias[0] = %f\n\r", IMU.gBias[0]);
    // pc.printf("IMU.gBias[1] = %f\n", IMU.gBias[1]);
    // pc.printf("IMU.gBias[2] = %f\n", IMU.gBias[2]);
    // pc.printf("IMU.gBiasRaw[0] = %i\n", IMU.gBiasRaw[0]);
    // pc.printf("IMU.gBiasRaw[1] = %i\n", IMU.gBiasRaw[1]);
    // pc.printf("IMU.gBiasRaw[2] = %i\n", IMU.gBiasRaw[2]);
    // pc.printf("IMU.aBias[0] = %f\n", IMU.aBias[0]);
    // pc.printf("IMU.aBias[1] = %f\n", IMU.aBias[1]);
    // pc.printf("IMU.aBias[2] = %f\n", IMU.aBias[2]);
    // pc.printf("IMU.aBiasRaw[0] = %i\n", IMU.aBiasRaw[0]);
    // pc.printf("IMU.aBiasRaw[1] = %i\n", IMU.aBiasRaw[1]);
    // pc.printf("IMU.aBiasRaw[2] = %i\n", IMU.aBiasRaw[2]);
    IMU.gBias[0] = -4.538422;
    IMU.gBias[1] = 0.500946;
    IMU.gBias[2] = 0.889740;
    IMU.gBiasRaw[0] = -607;
    IMU.gBiasRaw[1] = 67;
    IMU.gBiasRaw[2] = 119;
    IMU.aBias[0] = 0.048767;
    IMU.aBias[1] = 0.020691;
    IMU.aBias[2] = -0.004700;
    IMU.aBiasRaw[0] = 799;
    IMU.aBiasRaw[1] = 339;
    IMU.aBiasRaw[2] = -77;
    //angle_loop.initPID(15.5, 0.00001, 10000, 10, 400); // Kp 15.5, Kd 10000
    angle_loop.initPID(16, 0.00001, 10000, 10, 400);

    while (true) {
        IMU.readAccel();
        IMU.readGyro();
        comFilter(IMU.ax, IMU.ay, IMU.az, IMU.gx);
        if (!start) {
            float start_angle = atan2((float)IMU.ay, (float)IMU.az) * 180.0 / PI;
            if (start_angle < 1 && start_angle > -1) {
                pitch = atan2((float)IMU.ay, (float)IMU.az) * 180.0 / PI;
                angle_loop.restartPID();
                start = 1;
            }
        }
        else {
            speed = angle_loop.updatePID(pitch, set_point);
            Steppers.setSpeed(speed, 0);
        }
        if (pitch > 40 || pitch < -40) {
            start = 0;
            Steppers.setSpeed(0, 0);
        }
        // if (speed > 0)
        //     set_point += 0.00015;
        // if (speed < 0)
        //     set_point -= 0.00015;
        pc.printf("Pitch %f, Speed: %d, Set point %f\n\r", pitch, speed, set_point);
    }
}
