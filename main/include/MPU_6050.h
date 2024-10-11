#ifndef MPU_6050_H
#define MPU_6050_H

#include <stdio.h>
#include <string.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"

// Configuration defines
#define I2C_GPIO_SDA 21
#define I2C_GPIO_SCL 22
#define I2C_FREQUENCY_HZ 400000
#define MPU6050_ADDR 0x68

// MPU6050 registers
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_ACCEL_XOUT_H  0x3B
#define MPU6050_GYRO_XOUT_H   0x43

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float angle_x;
    float angle_y;
    float angle_z;
    float temperature;
} mpu6050_data_t;

esp_err_t mpu6050_init(void);
esp_err_t mpu6050_read_data(mpu6050_data_t *data);

#endif // MPU6050_H