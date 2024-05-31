#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include <algorithm>

#define PIN1 GPIO_NUM_13
#define PIN2 GPIO_NUM_32
#define PIN3 GPIO_NUM_33
#define PIN4 GPIO_NUM_25
#define PIN5 GPIO_NUM_26
#define PIN6 GPIO_NUM_27
#define PIN7 GPIO_NUM_14
#define PIN8 GPIO_NUM_12

// 1. Set up gpio pins
// 2. pull down all pins
// 3. Loop over row pins

static const char *TAG = "MAIN";

char get_key_char(int row, int col);

extern "C" void app_main(void)
{
  gpio_num_t cols[4]{PIN4, PIN3, PIN2, PIN1};
  gpio_num_t rows[4]{PIN8, PIN7, PIN6, PIN5};
  char characters[] = {'1', '2', '3', 'A', '4', '5', '6', 'B', '7', '8', '9', 'C', '*', '0', '#', 'D'};

  for (size_t i = 0; i < 4; i++)
  {
    esp_rom_gpio_pad_select_gpio(rows[i]);
    esp_rom_gpio_pad_select_gpio(cols[i]);

    gpio_pulldown_en(rows[i]);

    gpio_set_pull_mode(rows[i], GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(cols[i], GPIO_PULLDOWN_ONLY);

    gpio_set_direction(cols[i], GPIO_MODE_INPUT);
    gpio_set_direction(rows[i], GPIO_MODE_OUTPUT);
  }

  int last_pressed_index = -1;
  int current_char_index = -1;

  for (;;)
  {

    // Loop through rows
    for (size_t row_index = 0; row_index < 4; row_index++)
    {
      // Set current row High
      gpio_set_level(rows[row_index], 1);

      int col_index_match = -1;

      // Read  match
      for (size_t col_index = 0; col_index < 4; col_index++)
      {

        int level = gpio_get_level(cols[col_index]);

        int current_state;
        for (size_t i = 0; i < 1000; i++)
        {
          current_state = gpio_get_level(cols[col_index]);
        }

        if (current_state != level)
        {
          level = current_state;
        }

        if (level == 1)
        {

          // ESP_LOGI(TAG, "You pressed %c", characters[4 * row_index + col_index]);
          current_char_index = 4 * row_index + col_index;
          break;
        }

        // level is 0 -> check if that was the last pressed char
        // i.e - the button was released
        if (4 * row_index + col_index == current_char_index)
        {
          current_char_index = -1;
          last_pressed_index = -1;
        }
      }

      if (col_index_match >= 0)
      {
        break;
      }

      // No column active for row -> set row low.
      gpio_set_level(rows[row_index], 0);
    }

    if (current_char_index >= 0 && last_pressed_index != current_char_index)
    {
      char character = characters[current_char_index];

      ESP_LOGI(TAG, "You pressed %c", character);
      last_pressed_index = current_char_index;
    }
  }
}
