#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

#define LED_PIN GPIO_NUM_27

static const char *TAG = "MAIN";

extern "C" void app_main(void)
{
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

  ledc_timer_config_t timerConfig;

  timerConfig.clk_cfg = LEDC_AUTO_CLK;
  timerConfig.duty_resolution = LEDC_TIMER_13_BIT;
  timerConfig.freq_hz = 100;
  timerConfig.speed_mode = LEDC_LOW_SPEED_MODE;
  timerConfig.timer_num = LEDC_TIMER_0;
  timerConfig.deconfigure = false;

  // Restart on faulty timer config
  ESP_ERROR_CHECK(ledc_timer_config(&timerConfig));

  ledc_channel_config_t channelConfig;

  channelConfig.channel = LEDC_CHANNEL_0;
  channelConfig.duty = 0;
  channelConfig.gpio_num = LED_PIN;
  channelConfig.intr_type = LEDC_INTR_DISABLE;
  channelConfig.speed_mode = LEDC_LOW_SPEED_MODE;
  channelConfig.timer_sel = LEDC_TIMER_0;
  channelConfig.duty = 0;
  channelConfig.hpoint = 0;

  ESP_ERROR_CHECK(ledc_channel_config(&channelConfig));

  uint32_t duty = 0;

  for (;;)
  {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    duty += 25;
    duty = duty % 0b111111111111;
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_LOGI(TAG, "Duty %lu", duty);
  }
}
