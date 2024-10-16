#include "../include/MPU_6050.h"
#include <math.h>

static const char *MPU_TAG = "MPU_6050";
static i2c_master_dev_handle_t dev_handle;

typedef struct {
    float Q_angle;   // Process noise variance for the accelerometer
    float Q_bias;    // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float angle;     // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias;      // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate;      // Unbiased rate calculated from the rate and the calculated bias

    float P[2][2];   // Error covariance matrix - This is a 2x2 matrix
} Kalman_t;

static Kalman_t kalman_x, kalman_y;

// Kalman filter initialization
void kalman_init(Kalman_t *kalman) {
    kalman->Q_angle = 0.001f;
    kalman->Q_bias = 0.003f;
    kalman->R_measure = 0.03f;

    kalman->angle = 0.0f;
    kalman->bias = 0.0f;

    kalman->P[0][0] = 0.0f;
    kalman->P[0][1] = 0.0f;
    kalman->P[1][0] = 0.0f;
    kalman->P[1][1] = 0.0f;
}

// Kalman filter update
float kalman_update(Kalman_t *kalman, float newAngle, float newRate, float dt) {
    // Step 1: Predict
    kalman->rate = newRate - kalman->bias;
    kalman->angle += dt * kalman->rate;

    // Update estimation error covariance - Project the error covariance ahead
    kalman->P[0][0] += dt * (dt*kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
    kalman->P[0][1] -= dt * kalman->P[1][1];
    kalman->P[1][0] -= dt * kalman->P[1][1];
    kalman->P[1][1] += kalman->Q_bias * dt;

    // Step 2: Update
    float y = newAngle - kalman->angle;
    float S = kalman->P[0][0] + kalman->R_measure;
    float K[2];
    K[0] = kalman->P[0][0] / S;
    K[1] = kalman->P[1][0] / S;

    // Update state
    kalman->angle += K[0] * y;
    kalman->bias += K[1] * y;

    // Update estimation error covariance
    float P00_temp = kalman->P[0][0];
    float P01_temp = kalman->P[0][1];

    kalman->P[0][0] -= K[0] * P00_temp;
    kalman->P[0][1] -= K[0] * P01_temp;
    kalman->P[1][0] -= K[1] * P00_temp;
    kalman->P[1][1] -= K[1] * P01_temp;

    return kalman->angle;
}

esp_err_t mpu6050_init(void) {
    esp_err_t ret;

    ESP_LOGI(MPU_TAG, "Initializing I2C");

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
        ESP_LOGE(MPU_TAG, "Failed to create I2C bus");
        return ret;
    }

    ESP_LOGI(MPU_TAG, "Probing for MPU6050");
    ret = i2c_master_probe(bus_handle, MPU6050_ADDR, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(MPU_TAG, "MPU6050 not found!");
        return ret;
    }
    ESP_LOGI(MPU_TAG, "MPU6050 found!");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = I2C_FREQUENCY_HZ,
    };

    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(MPU_TAG, "Failed to add device to bus");
        return ret;
    }

    // Wake up MPU6050 (clear sleep bit)
    uint8_t cmd[] = {MPU6050_PWR_MGMT_1, 0x00};
    ret = i2c_master_transmit(dev_handle, cmd, sizeof(cmd), -1);
    if (ret != ESP_OK) {
        ESP_LOGE(MPU_TAG, "Failed to wake up MPU6050");
        return ret;
    }

    kalman_init(&kalman_x);
    kalman_init(&kalman_y);

    return ESP_OK;
}

static esp_err_t read_sensor_register(uint8_t reg_addr, uint8_t* data, size_t len) {
    esp_err_t ret;
    
    // Write register address
    ret = i2c_master_transmit(dev_handle, &reg_addr, 1, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(MPU_TAG, "Error writing register address");
        return ret;
    }
    
    // Read data
    ret = i2c_master_receive(dev_handle, data, len, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(MPU_TAG, "Error reading sensor data");
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

    // Calculate pitch and roll from accelerometer data
    float accel_angle_x = atan2f(data->accel_y, data->accel_z) * 180 / M_PI;
    float accel_angle_y = atan2f(-data->accel_x, sqrtf(data->accel_y * data->accel_y + data->accel_z * data->accel_z)) * 180 / M_PI;

    // Update Kalman filter
    static float last_read_time = 0;
    float current_time = (float)esp_timer_get_time() / 1000000.0f;
    float dt = current_time - last_read_time;
    last_read_time = current_time;

    if (dt > 0.0f) {
        data->kalman_angle_x = kalman_update(&kalman_x, accel_angle_x, data->gyro_x, dt);
        data->kalman_angle_y = kalman_update(&kalman_y, accel_angle_y, data->gyro_y, dt);
    }

    return ESP_OK;
}

void mpu6050_run(void *pvParameters) {
    esp_err_t init_ret = mpu6050_init();
    if (init_ret != ESP_OK) {
        ESP_LOGE(MPU_TAG, "Failed to initialize MPU6050");
        return;
    }

    esp_err_t read_ret;
    mpu6050_data_t sensor_data = {0};

    while(1) {
        read_ret = mpu6050_read_data(&sensor_data);
        if (read_ret == ESP_OK) {
            printf("--start frame--\n");
            printf("Accel_X (g): %.2f, Accel_Y (g): %.2f, Accel_Z (g): %.2f\n", 
                   sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z);

            printf("Temperature: %.2f\n", sensor_data.temperature);
            
            printf("Gyro_X (graus/s): %.2f, Gyro_Y (graus/s): %.2f, Gyro_Z (graus/s): %.2f\n", 
                   sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z);
            
            printf("Angle_X (graus): %.2f, Angle_Y (graus): %.2f\n", 
                   sensor_data.kalman_angle_x, sensor_data.kalman_angle_y);
            printf("--end frame--\n");
        } else {
            ESP_LOGE(MPU_TAG, "Error reading sensor data");
        }

        // UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
        // printf("Stack High Water Mark: %d\n", highWaterMark);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}