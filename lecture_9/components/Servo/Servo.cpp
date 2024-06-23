#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "Servo.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"

static const char *TAG = "Servo";

#define SERVO_TIMEBASE_RESOLUTION_HZ 50
#define SERVO_TIMEBASE_PERIOD 2500 // 2500 ticks, 2.5ms

Servo::Servo(gpio_num_t pin) : pin(pin)
{

  mcpwm_timer_handle_t timer = NULL;
  mcpwm_timer_config_t timer_config = {
      .group_id = 0,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
      .period_ticks = SERVO_TIMEBASE_PERIOD,
  };

  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

  mcpwm_oper_handle_t oper = NULL;
  mcpwm_operator_config_t operator_config = {
      .group_id = 0,
  };

  ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

  mcpwm_cmpr_handle_t comparator = NULL;
  mcpwm_comparator_config_t comparator_config = {
      .flags = {
          .update_cmp_on_tez = true,
      },
  };

  ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

  mcpwm_gen_handle_t generator = NULL;
  mcpwm_generator_config_t generator_config = {
      .gen_gpio_num = pin,
  };

  ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, 0));

  ESP_LOGI(TAG, "Created Servo on pind %d", pin);
}

Servo::~Servo()
{
  ESP_LOGI(TAG, "Deleted Servo on pin %d", pin);
}

void Servo::update(unsigned int angle)
{
}
