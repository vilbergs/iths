#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <LCD.h>
#include <I2CBus.h>
#include "freertos/FreeRTOS.h"

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

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully for master");

    lcd = new LCD();

    lcd->initialize();

    for (int i = 0; i < 16; i++)
    {
        lcd->display_text("Hello, World!");
        vTaskDelay(pdMS_TO_TICKS(60));

        lcd->clear_display();
        vTaskDelay(pdMS_TO_TICKS(15));

        vTaskDelay(pdMS_TO_TICKS(60));
    }

    ESP_LOGI(TAG, "Gracefully de-initializing I2C");
    lcd->deinitialize();
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "LCD de-initialized successfully");

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}
