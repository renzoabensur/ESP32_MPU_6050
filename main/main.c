#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "include/MPU_6050.h"

static const char *TAG = "MAIN";

void app_main(void) {
    // Initialize MPU6050
    esp_err_t ret = mpu6050_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU6050");
        return;
    }

    // Create data structure
    mpu6050_data_t sensor_data = {0};
    
    // Main loop
    while(1) {
        ret = mpu6050_read_data(&sensor_data);
        if (ret == ESP_OK) {
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
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}