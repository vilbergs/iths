#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <LCD.h>
#include <I2CBus.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "mpu6050.h"
#include <string>
#include <format>

static const char *TAG = "MAIN";

/*
 *   - B7: data bit 3
 *   - B6: data bit 2
 *   - B5: data bit 1
 *   - B4: data bit 0
 *   - B3: backlight (BL): off = 0, on = 1
 *   - B2: enable (EN): change from 1 to 0 to clock data into controller
 *   - B1: read/write (RW): write = 0, read = 1
 *   - B0: register select (RS): command = 0, data = 1
 */

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    i2c_port_t i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ,
        },
        .clk_flags = 0,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

LCD *lcd;
mpu6050_handle_t mpu6050;

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully for master");

    lcd = new LCD();
    lcd->initialize();

    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    mpu6050_wake_up(mpu6050); // Make sure the device is awake
    mpu6050_config(mpu6050, mpu6050_acce_fs_t::ACCE_FS_8G, mpu6050_gyro_fs_t::GYRO_FS_500DPS);

    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;

    for (;;)
    {

        mpu6050_get_acce(mpu6050, &acce);
        mpu6050_get_gyro(mpu6050, &gyro);

        // ESP_LOGI(TAG, "Acce: %f %f %f", acce.acce_x, acce.acce_y, acce.acce_z);
        // ESP_LOGI(TAG, "Gyro: %f %f %f", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);

        // lcd->display_text(std::format("Acce: {} {} {}", acce.acce_x, acce.acce_y, acce.acce_z));
        lcd->display_text(std::format("Gyro: {}", gyro.gyro_x));

        vTaskDelay(pdMS_TO_TICKS(1000));

        lcd->clear_display();
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_LOGI(TAG, "Gracefully de-initializing I2C");
    lcd->deinitialize();
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "LCD de-initialized successfully");

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");

    delete lcd;
    mpu6050_delete(mpu6050);
}
