#include "icm20948.h"
#include "imu_fusion.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "MadgwickAHRS.h"
#include <thread>
#include <chrono>

#define DEG_RAD 57.2957795131f

extern volatile float q0, q1, q2, q3;
float rx, ry, rz;

accel_gyro_data data;

void update_euler();

void update_imu();

icm20948 imu;

int main(void)
{
    if (imu.begin())
    {
        printf("imu began?\n");

        std::thread t1(update_imu);

        while (true)
        {
            // accel_gyro_data data = imu.read_accelerometer_gyro_data();
            printf("x:%f, y:%f, z:%f\n", data.ax, data.ay, data.az);
            printf("gx:%f, gy:%f, gz:%f\n", data.gx, data.gy, data.gz);

            // MadgwickAHRSupdateIMU(data.gx, data.gy, data.gz, data.ax, data.ay, data.az);

            // printf("q0:%f, q1:%f, q2:%f, q3:%f\n", q0, q1, q2, q3);
            update_euler();

            printf("rx:%f, ry:%f, rz:%f\n", rx, ry, rz);
            // mag_data mdata = imu.read_magnetometer_data();
            // printf("mx:%f, my:%f, mz:%f\n", mdata.x, mdata.y, mdata.z);
            usleep(100000);
        }
    }
}

void update_imu() {
    for (;;) {
        data = imu.read_accelerometer_gyro_data();
        MadgwickAHRSupdateIMU(data.gx / DEG_RAD, data.gy / DEG_RAD, data.gz / DEG_RAD, data.ax * 9.81f, data.ay * 9.81f, data.az * 9.81f);
        std::this_thread::sleep_for (std::chrono::milliseconds(2));
        // printf(".");
    }    
}

void update_euler() {
    float x = q0;
    float y = q1;
    float z = q2;
    float w = q3;
    ry = atan2((2 * y * w) - (2 * x * z), 1.0f - (2 * y * y) - (2 * z * z));
    rx = asin((2 * x * y) + (2 * z * w));
    rz = atan2((2 * x * w) - (2 * y * z), 1.0f - (2 * x * x) - (2 * z * z));
    rx *= DEG_RAD;
    ry *= DEG_RAD;
    rz *= DEG_RAD;    
}