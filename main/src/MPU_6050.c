#include "../include/MPU_6050.h"
#include <math.h>

static const char *TAG = "MPU_6050";
static i2c_master_dev_handle_t dev_handle;

esp_err_t mpu6050_init(void) {
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing I2C");

    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_GPIO_SCL,
        .sda_io_num = I2C_GPIO_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ret = i2c_new_master_bus(&i2c_bus_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus");
        return ret;
    }

    ESP_LOGI(TAG, "Probing for MPU6050");
    ret = i2c_master_probe(bus_handle, MPU6050_ADDR, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 not found!");
        return ret;
    }
    ESP_LOGI(TAG, "MPU6050 found!");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = I2C_FREQUENCY_HZ,
    };

    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device to bus");
        return ret;
    }

    // Wake up MPU6050 (clear sleep bit)
    uint8_t cmd[] = {MPU6050_PWR_MGMT_1, 0x00};
    ret = i2c_master_transmit(dev_handle, cmd, sizeof(cmd), -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050");
        return ret;
    }

    return ESP_OK;
}

static esp_err_t read_sensor_register(uint8_t reg_addr, uint8_t* data, size_t len) {
    esp_err_t ret;
    
    // Write register address
    ret = i2c_master_transmit(dev_handle, &reg_addr, 1, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error writing register address");
        return ret;
    }
    
    // Read data
    ret = i2c_master_receive(dev_handle, data, len, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error reading sensor data");
    }
    
    return ret;
}

esp_err_t mpu6050_read_data(mpu6050_data_t *data) {
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t raw_data[14]; // 6 bytes accel + 2 bytes temp + 6 bytes gyro
    esp_err_t ret = read_sensor_register(MPU6050_ACCEL_XOUT_H, raw_data, 14);
    if (ret != ESP_OK) {
        return ret;
    }

    // Get raw value
    int16_t accel_x = (raw_data[0] << 8) | raw_data[1];
    int16_t accel_y = (raw_data[2] << 8) | raw_data[3];
    int16_t accel_z = (raw_data[4] << 8) | raw_data[5];

    int16_t gyro_x = (raw_data[8] << 8) | raw_data[9];
    int16_t gyro_y = (raw_data[10] << 8) | raw_data[11];
    int16_t gyro_z = (raw_data[12] << 8) | raw_data[13];

    int16_t temp = (raw_data[6] << 8) | raw_data[7];

    // Convert to physical units
    data->accel_x = accel_x / MPU6050_ACCEL_ADC_CONVERSION;
    data->accel_y = accel_y / MPU6050_ACCEL_ADC_CONVERSION;
    data->accel_z = accel_z / MPU6050_ACCEL_ADC_CONVERSION;

    data->gyro_x = gyro_x / MPU6050_GYRO_ADC_CONVERSION;
    data->gyro_y = gyro_y / MPU6050_GYRO_ADC_CONVERSION;
    data->gyro_z = gyro_z / MPU6050_GYRO_ADC_CONVERSION;

    data->temperature = temp/340 + 36.53;

    // Update absolute angles (simplified - not accounting for drift)
    static float last_read_time = 0;
    float current_time = (float)esp_timer_get_time() / 1000000.0f;
    float dt = current_time - last_read_time;
    last_read_time = current_time;

    if (dt > 0.0f) {
        data->angle_x += data->gyro_x * dt;
        data->angle_y += data->gyro_y * dt;
        data->angle_z += data->gyro_z * dt;

        // Keep angles between 0 and 360 graus
        data->angle_x = fmod(data->angle_x + MPU6050_GYRO_CONVERSION_DEGREES, MPU6050_GYRO_CONVERSION_DEGREES);
        data->angle_y = fmod(data->angle_y + MPU6050_GYRO_CONVERSION_DEGREES, MPU6050_GYRO_CONVERSION_DEGREES);
        data->angle_z = fmod(data->angle_z + MPU6050_GYRO_CONVERSION_DEGREES, MPU6050_GYRO_CONVERSION_DEGREES);
    }

    return ESP_OK;
}

void mpu6050_run(void *pvParameters) {
    esp_err_t init_ret = mpu6050_init();
    if (init_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU6050");
        return;
    }

    esp_err_t read_ret;
    mpu6050_data_t sensor_data = {0};

    while(1) {
        read_ret = mpu6050_read_data(&sensor_data);
        if (read_ret == ESP_OK) {
            printf("Acelerômetro (g):\n");
            printf("X: %.2f, Y: %.2f, Z: %.2f\n", 
                   sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z);

            printf("\nTemperatura (graus):\n");
            printf("T: %.2f\n", 
                   sensor_data.temperature);
            
            printf("\nGiroscópio (graus/s):\n");
            printf("X: %.2f, Y: %.2f, Z: %.2f\n", 
                   sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z);
            
            printf("\nPosição Angular (graus):\n");
            printf("X: %.2f, Y: %.2f, Z: %.2f\n", 
                   sensor_data.angle_x, sensor_data.angle_y, sensor_data.angle_z);
        } else {
            ESP_LOGE(TAG, "Error reading sensor data");
        }

        UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
        printf("Stack High Water Mark: %d\n", highWaterMark);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}