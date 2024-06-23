#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include <LCD.h>
#include <I2CBus.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "mpu6050.h"
#include <string>
#include <format>

#define BUTTON1_GPIO GPIO_NUM_0 // Adjust the GPIO number to match your setup
#define BUTTON2_GPIO GPIO_NUM_2 // Adjust the GPIO number to match your setup

static const char *TAG = "MAIN";
static const int MAX_PAGE = 6;
volatile int current_page = 0;
// State variables for debouncing
volatile bool button1_state = false;
volatile bool button2_state = false;

// Message structure for the queue
typedef struct
{
    int page;
} button_message_t;

// Queue handle
QueueHandle_t button_queue;

TimerHandle_t debounce_timer_button1;
TimerHandle_t debounce_timer_button2;

// Debounce time (in milliseconds)
const TickType_t debounce_time = pdMS_TO_TICKS(50);

// Timer callback function for Button 1
void debounce_timer_callback_button1(TimerHandle_t xTimer)
{
    int level = gpio_get_level(BUTTON1_GPIO);

    int current_state;
    for (size_t i = 0; i < 500; i++)
    {
        current_state = gpio_get_level(BUTTON1_GPIO);
    }

    if (current_state != level)
    {
        level = current_state;
    }

    if (level == 0)
    {
        button_message_t msg = {.page = -1};
        xQueueSend(button_queue, &msg, portMAX_DELAY);
    }
}

// Timer callback function for Button 2
void debounce_timer_callback_button2(TimerHandle_t xTimer)
{
    int level = gpio_get_level(BUTTON2_GPIO);

    int current_state;
    for (size_t i = 0; i < 500; i++)
    {
        current_state = gpio_get_level(BUTTON2_GPIO);
    }

    if (current_state != level)
    {
        level = current_state;
    }

    if (level == 0)
    {
        button_message_t msg = {.page = 1};
        xQueueSend(button_queue, &msg, portMAX_DELAY);
    }
}

// ISR Handler function for Button 1
void IRAM_ATTR button1_isr_handler(void *arg)
{
    if (xTimerIsTimerActive(debounce_timer_button1) == pdFALSE)
    {
        xTimerStartFromISR(debounce_timer_button1, NULL);
    }
}

// ISR Handler function for Button 2
void IRAM_ATTR button2_isr_handler(void *arg)
{
    if (xTimerIsTimerActive(debounce_timer_button2) == pdFALSE)
    {
        xTimerStartFromISR(debounce_timer_button2, NULL);
    }
}

// Button task
void button_task(void *arg)
{
    button_message_t msg;
    while (true)
    {
        // Wait for a message from the ISR
        if (xQueueReceive(button_queue, &msg, portMAX_DELAY))
        {

            current_page = (current_page + msg.page + MAX_PAGE) % MAX_PAGE;
            ESP_LOGI(TAG, "Going to page %d", current_page + 1);
        }
    }
}

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

/**
 * @brief gpio initialization
 */
static esp_err_t gpio_init(gpio_num_t gpio_num, gpio_isr_t isr_handler)
{
    esp_err_t err;

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;        // Trigger on falling edge (button press)
    io_conf.mode = GPIO_MODE_INPUT;               // Set as input mode
    io_conf.pin_bit_mask = (1ULL << gpio_num);    // GPIO pin mask for Button 1
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;      // Enable pull-up
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // Disable pull-down
    gpio_config(&io_conf);

    // Attach the interrupt service routines
    return gpio_isr_handler_add(gpio_num, isr_handler, (void *)gpio_num);
}

LCD *lcd;
mpu6050_handle_t mpu6050;

extern "C" void app_main(void)
{
    button_queue = xQueueCreate(10, sizeof(button_message_t));
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);

    // Create debounce timers
    debounce_timer_button1 = xTimerCreate("debounce_timer_button1", debounce_time, pdFALSE, (void *)1, debounce_timer_callback_button1);
    debounce_timer_button2 = xTimerCreate("debounce_timer_button2", debounce_time, pdFALSE, (void *)2, debounce_timer_callback_button2);

    // Install ISR service
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);

    ESP_ERROR_CHECK(gpio_init(BUTTON1_GPIO, button1_isr_handler));
    ESP_ERROR_CHECK(gpio_init(BUTTON2_GPIO, button2_isr_handler));

    ESP_LOGI(TAG, "Buttons initialized successfully");

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully for master");

    lcd = new LCD();
    lcd->initialize();

    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    mpu6050_wake_up(mpu6050); // Make sure the device is awake
    mpu6050_config(mpu6050, mpu6050_acce_fs_t::ACCE_FS_8G, mpu6050_gyro_fs_t::GYRO_FS_500DPS);

    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;

    int last_page = -1;

    for (;;)
    {

        if (last_page != current_page)
        {
            lcd->clear_display();
            vTaskDelay(pdMS_TO_TICKS(50));
            lcd->display_text(std::format("Page: {}", current_page + 1));
            last_page = current_page;
        }

        mpu6050_get_acce(mpu6050, &acce);
        mpu6050_get_gyro(mpu6050, &gyro);

        lcd->set_cursor(1, 0);
        switch (current_page)
        {
        case 0:
            lcd->display_text(std::format("Gyro X: {:.2f}", gyro.gyro_x));
            break;
        case 1:
            lcd->display_text(std::format("Gyro Y: {:.2f}", acce.acce_y));
            break;
        case 2:
            lcd->display_text(std::format("Gyro Z: {:.2f}", acce.acce_z));
            break;

        case 3:
            lcd->display_text(std::format("Acce X: {}", acce.acce_x));
            break;

        case 4:
            lcd->display_text(std::format("Acce Y: {}", acce.acce_y));
            break;

        case 5:
            lcd->display_text(std::format("Acce Z: {}", acce.acce_z));
            break;

        default:
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(250));
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
