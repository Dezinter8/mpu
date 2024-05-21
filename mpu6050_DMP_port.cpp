#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"

MPU6050 mpu;


bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yaw, pitch, roll;

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

void initLED() {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
} 

void  waitForUsbConnect() {
    #ifdef _PICO_STDIO_USB_H
        while (!stdio_usb_connected()) {
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(250);
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(250);
        }
    #endif
}

int main() {
    stdio_init_all();
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);
    gpio_pull_up(0);
    gpio_pull_up(1);
    
    initLED();
    waitForUsbConnect();


    mpu.initialize();
    devStatus = mpu.dmpInitialize();


    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);	
	

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        printf("DMP Initialization failed (code %d)", devStatus);
        sleep_ms(2000);
    }

    yaw = 0.0;
    pitch = 0.0;
    roll = 0.0;


    while(1){
        if (!dmpReady) continue;
        mpuInterrupt = true;
        fifoCount = mpu.getFIFOCount();

        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            mpu.resetFIFO();
            printf("FIFO overflow!\n");
        } else if (mpuIntStatus & 0x01) {
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount -= packetSize;

            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yaw = ypr[0] * 180 / PI;
            pitch = ypr[1] * 180 / PI;
            roll = ypr[2] * 180 / PI;
            printf("ypr: %f,\t %f,\t %f\n", yaw, pitch, roll);
        }
    }

    return 0;
}
