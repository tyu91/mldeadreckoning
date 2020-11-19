#include "icm20948.h"
#include "gps.h"
#include "TinyGPS++.h"
#include "PCA9685.h"
#include "defines.h"
#include "MadgwickAHRS.h"

#include <unistd.h>
#include <iostream>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>
#include <wiringPiI2C.h>
#include <mutex>
#include <thread>
#include <chrono>
#include <sys/types.h>
#include <signal.h>
#include <time.h>
#include <string.h>

#define DEG_RAD 57.2957795131f
#define SERVO_MIN 876
#define SERVO_MAX 1753
#define CONTROL_TIME_STEP 0.01f // 100hz

extern volatile float q0, q1, q2, q3;
float rx, ry, rz;

accel_gyro_data data;
vehicle_info v_info;

void update_euler();

// 500hz thread
void hz500handler(int signo);
void update_imu();

// 100hz thread
void update_pwm();
void run_control();

// 10hz thread
void update_gps();

// 50hz?
void log_data();

I2CGPS gps;
TinyGPSPlus tgps;
icm20948 imu;

FILE *fp;

int main(int argc, char *argv[])
{
    if (!gps.begin())
    {
        printf("Failed to initialize GPS\n");
        exit(EXIT_FAILURE);
    }
    if (!imu.begin())
    {
        printf("Failed to initialize IMU\n");
        exit(EXIT_FAILURE);
    }

    time_t rawtime;
    struct tm *timeinfo;

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    printf("everything should be ready to go...\n");

    char* time = asctime(timeinfo);
    time[strlen(time) - 1]  = 0;

    // support prefixes

    if (argc == 2) {
        time = strcat(argv[1], time);
    }

    char* ext = ".csv";

    fp = fopen(strcat(time, ext), "w");
    fprintf(fp, "ax, ay, az, gx, gy, gz, rx, ry, rz, lat, long, alt, time, velocity\n");
    printf("File initialized...\n");

    std::thread t1(update_imu);
    std::thread t3(log_data);
    std::thread t4(update_gps);


    while (true)
    {
        // printf("LAT=%f\n", v_info.gps.lat);
        // printf("LNG=%f\n", v_info.gps.lng);
        // printf("ALT=%f\n", v_info.gps.alt);
        printf("SATS=%d\n", v_info.gps.numSats);
        // printf("TIME=%d:%d:%d.%d\n", tgps.time.hour(), tgps.time.minute(), tgps.time.second(), tgps.time.centisecond());

        // printf("x:%f, y:%f, z:%f\n", data.ax, data.ay, data.az);
        // printf("gx:%f, gy:%f, gz:%f\n", data.gx, data.gy, data.gz);

        // printf("rx:%f, ry:%f, rz:%f\n", v_info.rotation.x, v_info.rotation.y, v_info.rotation.z);
        // printf("iter_time:%d\n", v_info.iter_time);

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 0;
}

void update_imu()
{ // if
    static std::chrono::time_point<std::chrono::system_clock> last_start_time = std::chrono::system_clock::now();
    static int runtime = 0;
    for (;;)
    {
        std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();
        int iter_time = std::chrono::duration_cast<std::chrono::nanoseconds>(start_time - last_start_time).count();
        last_start_time = start_time;

        data = imu.read_accelerometer_gyro_data();
        // data.gx *= 1.2f;
        // data.gy *= 1.2f;
        // data.gz *= 1.2f;
        MadgwickAHRSupdateIMU(data.gx / DEG_RAD, data.gy / DEG_RAD, data.gz / DEG_RAD, data.ax * 9.81f, data.ay * 9.81f, data.az * 9.81f);

        v_info.mutex.lock();

        v_info.iter_time = iter_time;

        v_info.acceleration.x = data.ax;
        v_info.acceleration.y = data.ay;
        v_info.acceleration.z = data.az;

        v_info.gyro.x = data.gx;
        v_info.gyro.y = data.gy;
        v_info.gyro.z = data.gz;

        update_euler();

        v_info.rotation.x = rx;
        v_info.rotation.y = ry;
        v_info.rotation.z = rz;

        v_info.mutex.unlock();
        std::chrono::time_point<std::chrono::system_clock> end_time = std::chrono::system_clock::now();
        runtime = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
        std::this_thread::sleep_for(std::chrono::nanoseconds((4 * 1000000) - runtime));
        // printf(".");
    }
}

void run_control()
{
    for (;;)
    {
        v_info.mutex.lock();
        v_info.controlTicks++;
        v_info.mutex.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// 50hz thread
void update_gps()
{
    for (;;)
    {
        uint8_t dataBytes = gps.available();
        if (dataBytes > 0)
        { // actually have some data
            // printf("reading gps data\n");
            for (int i = 0; i < dataBytes; i++)
            {
                tgps.encode(gps.read());
            }
        }

        v_info.mutex.lock();
        v_info.gps.numSats = tgps.satellites.value();
        v_info.gps.valid = tgps.location.isValid();
        if (tgps.location.isUpdated())
        {
            v_info.gps.lat = tgps.location.lat();
            v_info.gps.lng = tgps.location.lng();
            v_info.gps.alt = tgps.altitude.meters();
            v_info.gps.hdg = tgps.course.value();
            v_info.gps.vel = tgps.speed.mps();
        }
        v_info.mutex.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// 200hz?
void log_data()
{
    static int runtime = 0;
    // delay logging two seconds to give everything time to settle
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    printf("Logging started at %d:%d:%d.%d\n", tgps.time.hour(), tgps.time.minute(), tgps.time.second(), tgps.time.centisecond());
    for (;;)
    {
        // fprintf(fp, "ax, ay, az, gx, gy, gz, rx, ry, rz, lat, long, alt, time");

        std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();
        fprintf(fp, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d:%d:%d.%d,%f\n",
                v_info.acceleration.x, v_info.acceleration.y, v_info.acceleration.z,
                v_info.gyro.x, v_info.gyro.y, v_info.gyro.z,
                v_info.rotation.x, v_info.rotation.y, v_info.rotation.z,
                v_info.gps.lat, v_info.gps.lng, v_info.gps.alt,
                tgps.time.hour(), tgps.time.minute(), tgps.time.second(), tgps.time.centisecond(),
                v_info.gps.vel);
        std::chrono::time_point<std::chrono::system_clock> end_time = std::chrono::system_clock::now();
        runtime = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
        std::this_thread::sleep_for(std::chrono::nanoseconds((5 * 1000000) - runtime));
    }
}

void update_euler()
{
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