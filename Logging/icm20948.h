// translating from github.com/pimoroni/icm20948-python/tree/master/library

#ifndef ICM20948
#define ICM20948

#include <wiringPiI2C.h>
#include <cstdlib>
#include <unistd.h>

#define CHIP_ID 0xEA
#define I2C_ADDR 0x68
#define I2C_ADDR_ALT 0x69
#define ICM20948_BANK_SEL 0x7f

#define ICM20948_I2C_MST_ODR_CONFIG 0x00
#define ICM20948_I2C_MST_CTRL 0x01
#define ICM20948_I2C_MST_DELAY_CTRL 0x02
#define ICM20948_I2C_SLV0_ADDR 0x03
#define ICM20948_I2C_SLV0_REG 0x04
#define ICM20948_I2C_SLV0_CTRL 0x05
#define ICM20948_I2C_SLV0_DO 0x06
#define ICM20948_EXT_SLV_SENS_DATA_00 0x3B
 
#define ICM20948_GYRO_SMPLRT_DIV 0x00
#define ICM20948_GYRO_CONFIG_1 0x01
#define ICM20948_GYRO_CONFIG_2 0x02

// Bank 0
#define ICM20948_WHO_AM_I 0x00
#define ICM20948_USER_CTRL 0x03
#define ICM20948_PWR_MGMT_1 0x06
#define ICM20948_PWR_MGMT_2 0x07
#define ICM20948_INT_PIN_CFG 0x0F

#define ICM20948_ACCEL_SMPLRT_DIV_1 0x10
#define ICM20948_ACCEL_SMPLRT_DIV_2 0x11
#define ICM20948_ACCEL_INTEL_CTRL 0x12
#define ICM20948_ACCEL_WOM_THR 0x13
#define ICM20948_ACCEL_CONFIG 0x14
#define ICM20948_ACCEL_XOUT_H 0x2D
#define ICM20948_ACCEL_XOUT_L 0x2E
#define ICM20948_GRYO_XOUT_H 0x33

#define AK09916_I2C_ADDR 0x0c
#define AK09916_CHIP_ID 0x09
#define AK09916_WIA 0x01
#define AK09916_ST1 0x10
#define AK09916_ST1_DOR 0b00000010   # Data overflow bit
#define AK09916_ST1_DRDY 0b00000001  # Data self.ready bit
#define AK09916_HXL 0x11
#define AK09916_ST2 0x18
#define AK09916_ST2_HOFL 0b00001000  # Magnetic sensor overflow bit
#define AK09916_CNTL2 0x31
#define AK09916_CNTL2_MODE 0b00001111
#define AK09916_CNTL2_MODE_OFF 0
#define AK09916_CNTL2_MODE_SINGLE 1
#define AK09916_CNTL2_MODE_CONT1 2
#define AK09916_CNTL2_MODE_CONT2 4
#define AK09916_CNTL2_MODE_CONT3 6
#define AK09916_CNTL2_MODE_CONT4 8
#define AK09916_CNTL2_MODE_TEST 16
#define AK09916_CNTL3 0x32

class mag_data {
    public:
        float x;
        float y;
        float z;
};

class accel_gyro_data {
    public:
        float ax;
        float ay;
        float az;
        float gx;
        float gy;
        float gz;
};

class icm20948{
    public:        
        bool magnetometer_ready(void);
        mag_data read_magnetometer_data(void);
        accel_gyro_data read_accelerometer_gyro_data(void);
        void set_accelerometer_sample_rate(int);
        void set_accelerometer_full_scale(int);
        void set_accelerometer_low_pass(bool, int);
        void set_gyro_sample_rate(int);
        void set_gyro_full_scale(int);
        void set_gyro_low_pass(bool, int);
        bool begin(int address = I2C_ADDR_ALT);
    private:
        int _current_bank = -1;
        int _i2c_fd;
        int _i2c_addr;
        u_int8_t _mag_read_buffer[64] = {0};
        u_int8_t _imu_read_buffer[64] = {0};

        void write(int, int);
        u_int8_t read(int);
        u_int8_t* read_bytes(int, int);
        void bank(int); // change device bank
        void mag_write(int, int); // write to magnetometer device?
        u_int8_t mag_read(int);
        u_int8_t* mag_read_bytes(int, int);  
        float _g_scale[4] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
        float _dps_scale[4] = {131.0f, 65.5f, 32.8f, 16.4f};
        int _g_scale_saved = 0;
        int _dps_scale_saved = 0;
};

#endif