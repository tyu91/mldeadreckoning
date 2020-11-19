#include "icm20948.h"
#include <stdlib.h>
#include <stdio.h>

// int main(void)
// {
//     icm20948 imu;
//     if (imu.begin())
//     {
//         printf("imu began?\n");

//         while (true)
//         {
//             accel_gyro_data data = imu.read_accelerometer_gyro_data();
//             printf("x:%f, y:%f, z:%f\n", data.ax, data.ay, data.az);
//             printf("gx:%f, gy:%f, gz:%f\n", data.gx, data.gy, data.gz);

//             mag_data mdata = imu.read_magnetometer_data();
//             printf("mx:%f, my:%f, mz:%f\n", mdata.x, mdata.y, mdata.z);
//             usleep(100000);
//         }
//     }
// }

void icm20948::write(int reg, int value)
{
    wiringPiI2CWriteReg8(_i2c_fd, reg, value);
    usleep(100);
}

u_int8_t icm20948::read(int reg)
{
    u_int8_t value = wiringPiI2CReadReg8(_i2c_fd, reg);
    return value;
}

u_int8_t *icm20948::read_bytes(int reg, int bytes)
{
    wiringPiI2CWrite(_i2c_fd, reg);
    // for (int i = 0; i < bytes; i++) {
    //     _imu_read_buffer[i] = wiringPiI2CReadReg8(_i2c_fd, reg + i);
    // }
    ::read(_i2c_fd, (u_int8_t *)_imu_read_buffer, bytes);
    return (u_int8_t *)_imu_read_buffer;
}

void icm20948::bank(int bank)
{
    if (_current_bank != bank)
    {
        wiringPiI2CWriteReg8(_i2c_fd, ICM20948_BANK_SEL, bank << 4);
        _current_bank = bank;
    }
}

void icm20948::mag_write(int reg, int value)
{
    bank(3);
    wiringPiI2CWriteReg8(_i2c_fd, ICM20948_I2C_SLV0_ADDR, AK09916_I2C_ADDR);
    wiringPiI2CWriteReg8(_i2c_fd, ICM20948_I2C_SLV0_REG, reg);
    wiringPiI2CWriteReg8(_i2c_fd, ICM20948_I2C_SLV0_DO, value);
    bank(0);
}

u_int8_t icm20948::mag_read(int reg)
{
    bank(3);
    wiringPiI2CWriteReg8(_i2c_fd, ICM20948_I2C_SLV0_CTRL, 0x80 | 1);
    wiringPiI2CWriteReg8(_i2c_fd, ICM20948_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80);
    wiringPiI2CWriteReg8(_i2c_fd, ICM20948_I2C_SLV0_REG, reg);
    wiringPiI2CWriteReg8(_i2c_fd, ICM20948_I2C_SLV0_DO, 0xFF);
    bank(0);
    return wiringPiI2CReadReg8(_i2c_fd, ICM20948_EXT_SLV_SENS_DATA_00);
}

u_int8_t *icm20948::mag_read_bytes(int reg, int bytes)
{
    bank(3);
    wiringPiI2CWriteReg8(_i2c_fd, ICM20948_I2C_SLV0_CTRL, 0x80 | 0x08 | bytes);
    wiringPiI2CWriteReg8(_i2c_fd, ICM20948_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80);
    wiringPiI2CWriteReg8(_i2c_fd, ICM20948_I2C_SLV0_REG, reg);
    wiringPiI2CWriteReg8(_i2c_fd, ICM20948_I2C_SLV0_DO, 0xFF);
    usleep(100);
    bank(0);
    //::read(_i2c_fd, (u_int8_t *)_mag_read_buffer, bytes);
    return read_bytes(ICM20948_EXT_SLV_SENS_DATA_00, bytes);//(u_int8_t *)_mag_read_buffer;
}

bool icm20948::magnetometer_ready(void)
{
    return (mag_read(AK09916_ST1) & 0x01) > 0;
}

// TODO MAKE IT NOT BREAK EVERYTHING DAMMIT
mag_data icm20948::read_magnetometer_data(void)
{
    mag_write(AK09916_CNTL2, 0x01);
    do {
        printf(".");
        usleep(100);
    } while (!magnetometer_ready());

    printf("\n");

    mag_data retVal = {0, 0, 0};

    u_int8_t *raw_mag_data = mag_read_bytes(AK09916_HXL, 6);

    printf("Raw mag data\n");

    for (int i = 0; i < 6; i++) {
        printf("%d\n", raw_mag_data[i]);
    }

    u_int8_t *swizzle = (u_int8_t *)malloc(2);

    // swizzle[0] = raw_mag_data[1];
    // swizzle[1] = raw_mag_data[0];
    // retVal.x = (float)(*(int16_t *)(swizzle));

    // swizzle[0] = raw_mag_data[3];
    // swizzle[1] = raw_mag_data[2];
    // retVal.y = (float)(*(int16_t *)(swizzle));

    // swizzle[0] = raw_mag_data[5];
    // swizzle[1] = raw_mag_data[4];
    // retVal.z = (float)(*(int16_t *)(swizzle));

    retVal.x = *(int16_t *)(raw_mag_data);
    retVal.y = *(int16_t *)(raw_mag_data + 2);
    retVal.z = *(int16_t *)(raw_mag_data + 4);

    retVal.x *= 0.15;
    retVal.y *= 0.15;
    retVal.z *= 0.15;

    return retVal;
}

accel_gyro_data icm20948::read_accelerometer_gyro_data(void)
{
    bank(0);
    accel_gyro_data retVal = {0, 0, 0, 0, 0, 0};
    u_int8_t *raw_accel_mag_data = read_bytes(ICM20948_ACCEL_XOUT_H, 12);

    // endian bs
    u_int8_t *swizzle = (u_int8_t *)malloc(2);

    swizzle[0] = raw_accel_mag_data[1];
    swizzle[1] = raw_accel_mag_data[0];
    retVal.ax = (float)(*(int16_t *)(swizzle));

    swizzle[0] = raw_accel_mag_data[3];
    swizzle[1] = raw_accel_mag_data[2];
    retVal.ay = (float)(*(int16_t *)(swizzle));

    swizzle[0] = raw_accel_mag_data[5];
    swizzle[1] = raw_accel_mag_data[4];
    retVal.az = (float)(*(int16_t *)(swizzle));

    swizzle[0] = raw_accel_mag_data[7];
    swizzle[1] = raw_accel_mag_data[6];
    retVal.gx = (float)(*(int16_t *)(swizzle));

    swizzle[0] = raw_accel_mag_data[9];
    swizzle[1] = raw_accel_mag_data[8];
    retVal.gy = (float)(*(int16_t *)(swizzle));

    swizzle[0] = raw_accel_mag_data[11];
    swizzle[1] = raw_accel_mag_data[10];
    retVal.gz = (float)(*(int16_t *)(swizzle));

    bank(2);

    int scale = read(ICM20948_ACCEL_CONFIG);
    scale = scale >> 1;
    scale = scale & 0b11;

    float gs = _g_scale[scale];//_g_scale_saved];

    retVal.ax /= gs;
    retVal.ay /= gs;
    retVal.az /= gs;

    scale = (read(ICM20948_GYRO_CONFIG_1) & 0x06) >> 1;

    float dps = _dps_scale[scale];//_dps_scale_saved];

    retVal.gx /= dps;
    retVal.gy /= dps;
    retVal.gz /= dps;

    return retVal;
}

void icm20948::set_accelerometer_sample_rate(int)
{
    bank(2);

    int rate = (int)((1125.0 / rate) - 1);

    write(ICM20948_ACCEL_SMPLRT_DIV_1, (rate >> 8) * 0xFF);
    write(ICM20948_ACCEL_SMPLRT_DIV_2, rate & 0xFF);
}

void icm20948::set_accelerometer_full_scale(int scale)
{
    bank(2);
    int value = read(ICM20948_ACCEL_CONFIG) & 0b11111001;
    value |= ((scale & 0b11) << 1);
    write(ICM20948_ACCEL_CONFIG, value);
    _g_scale_saved = scale;
}

void icm20948::set_accelerometer_low_pass(bool enabled, int mode)
{
    bank(2);
    int value = read(ICM20948_ACCEL_CONFIG) & 0b10001110;
    if (enabled)
    {
        value |= 0b1;
    }
    value |= (mode & 0x07) << 4;
    write(ICM20948_ACCEL_CONFIG, value);
}

void icm20948::set_gyro_sample_rate(int rate)
{
    bank(2);
    rate = (int)((1100.0f / rate) - 1);
    write(ICM20948_GYRO_SMPLRT_DIV, rate);
}

void icm20948::set_gyro_full_scale(int scale)
{
    bank(2);
    int value = read(ICM20948_GYRO_CONFIG_1) & 0b11111001;
    value |= ((scale & 0b11) << 1);
    write(ICM20948_GYRO_CONFIG_1, value);
    _dps_scale_saved = scale;
}

void icm20948::set_gyro_low_pass(bool enabled, int mode)
{
    bank(2);
    int value = read(ICM20948_GYRO_CONFIG_1) & 0b10001110;
    if (enabled)
    {
        value |= 0b1;
    }
    value |= (mode & 0x07) << 4;
    write(ICM20948_GYRO_CONFIG_1, value);
}

bool icm20948::begin(int address)
{
    _i2c_addr = address;
    _i2c_fd = wiringPiI2CSetup(address);

    bank(0);
    if (read(ICM20948_WHO_AM_I) != CHIP_ID)
    {
        printf("unable to find icm20948\n");
        return false;
    }

    write(ICM20948_PWR_MGMT_1, 0x01);
    write(ICM20948_PWR_MGMT_2, 0x00);

    bank(2);

    set_gyro_sample_rate(500);
    set_gyro_low_pass(true, 5);
    set_gyro_full_scale(1);

    set_accelerometer_sample_rate(500);
    set_accelerometer_low_pass(true, 5);
    set_accelerometer_full_scale(2);

    bank(0);

    write(ICM20948_INT_PIN_CFG, 0x30);
    write(ICM20948_USER_CTRL, 0x20);

    bank(3);

    write(ICM20948_I2C_MST_CTRL, 0x4D);
    write(ICM20948_I2C_MST_DELAY_CTRL, 0x01);

    usleep(100);

    if (mag_read(AK09916_WIA) != AK09916_CHIP_ID)
    {
        printf("unable to find ak09916\n");
        return false;
    }

    mag_write(AK09916_CNTL3, 0x01);
    while (mag_read(AK09916_CNTL3) == 0x01)
    {
        usleep(100);
    }
    
    return true;
}
