#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "include/MPU_6050.h"

void app_main(void) {
    xTaskCreate(mpu6050_run, "mpu6050_run", 8192, NULL, 5, NULL);
}