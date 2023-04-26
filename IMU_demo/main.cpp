#include "mbed.h"
#include "LSM9DS1.h"
#define PI 3.14159
// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -4.94 // Declination (degrees) in Atlanta,GA.

DigitalOut myled(LED1);
Serial pc(USBTX, USBRX);
// Calculate pitch, roll, and heading.
// Pitch/roll calculations taken from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
    float roll = atan2(ay, az);
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));
// touchy trig stuff to use arctan to get compass heading (scale is 0..360)
    mx = -mx;
    float heading;
    if (my == 0.0)
        heading = (mx < 0.0) ? 180.0 : 0.0;
    else
        heading = atan2(mx, my)*360.0/(2.0*PI);
    //pc.printf("heading atan=%f \n\r",heading);
    heading -= DECLINATION; //correct for geo location
    if(heading>180.0) heading = heading - 360.0;
    else if(heading<-180.0) heading = 360.0 + heading;
    else if(heading<0.0) heading = 360.0  + heading;


    // Convert everything from radians to degrees:
    //heading *= 180.0 / PI;
    pitch *= 180.0 / PI;
    roll  *= 180.0 / PI;

    pc.printf("Pitch: %f,    Roll: %f degress\n\r",pitch,roll);
    pc.printf("Magnetic Heading: %f degress\n\r",heading);
}




int main()
{
    //LSM9DS1 lol(p9, p10, 0x6B, 0x1E);
    LSM9DS1 IMU(p9, p10, 0xD6, 0x3C);
    IMU.begin();
    if (!IMU.begin()) {
        pc.printf("Failed to communicate with LSM9DS1.\n");
    }
    IMU.calibrate(1);
    IMU.calibrateMag(0);
    while(1) {
        while(!IMU.tempAvailable());
        IMU.readTemp();
        while(!IMU.magAvailable(X_AXIS));
        IMU.readMag();
        while(!IMU.accelAvailable());
        IMU.readAccel();
        while(!IMU.gyroAvailable());
        IMU.readGyro();
        pc.printf("\nIMU Temperature = %f C\n\r",25.0 + IMU.temperature/16.0);
        pc.printf("        X axis    Y axis    Z axis\n\r");
        pc.printf("gyro:  %9f %9f %9f in deg/s\n\r", IMU.calcGyro(IMU.gx), IMU.calcGyro(IMU.gy), IMU.calcGyro(IMU.gz));
        pc.printf("accel: %9f %9f %9f in Gs\n\r", IMU.calcAccel(IMU.ax), IMU.calcAccel(IMU.ay), IMU.calcAccel(IMU.az));
        pc.printf("mag:   %9f %9f %9f in gauss\n\r", IMU.calcMag(IMU.mx), IMU.calcMag(IMU.my), IMU.calcMag(IMU.mz));
        printAttitude(IMU.calcAccel(IMU.ax), IMU.calcAccel(IMU.ay), IMU.calcAccel(IMU.az), IMU.calcMag(IMU.mx),
                      IMU.calcMag(IMU.my), IMU.calcMag(IMU.mz));
        myled = 1;
        wait(0.5);
        myled = 0;
        wait(0.5);
    }
}

