#include <stdio.h>
#include <string>
#include "freertos/FreeRTOS.h"
#include "Led.h"
#include "esp_log.h"

static const char *TAG = "LED";

Led::Led(gpio_num_t pin) : pin(pin)
{
  ESP_LOGI(TAG, "Created Led on pind %d", pin);
  esp_rom_gpio_pad_select_gpio(pin);
  gpio_set_direction(pin, GPIO_MODE_OUTPUT);
  state = LS_NONE;
}

Led::~Led()
{
  ESP_LOGI(TAG, "Deleted Led on pin %d", pin);
}

void Led::update()
{
  switch (state)
  {
  case LS_BLINKING_ON:
    if ((pdTICKS_TO_MS(xTaskGetTickCount()) - timer) > (on + off))
    {
      timer = pdTICKS_TO_MS(xTaskGetTickCount());
      Off();
      state = LS_BLINKING_OFF;
    }

    break;
  case LS_BLINKING_OFF:
    if ((pdTICKS_TO_MS(xTaskGetTickCount()) - timer) > on)
    {
      timer = pdTICKS_TO_MS(xTaskGetTickCount());
      On();
      state = LS_BLINKING_ON;
    }
    break;
  default:
    break;
  }
}

void Led::blink(unsigned int on, unsigned int off)
{
  this->on = on;
  this->off = off;
  timer = pdTICKS_TO_MS(xTaskGetTickCount());
  On();
  state = LS_BLINKING_ON;
}

void Led::On()
{
  ESP_LOGI(TAG, "Turned on led on pin %d", pin);
  gpio_set_level(pin, 1);
}

void Led::Off()
{
  ESP_LOGI(TAG, "Turned off led on pin %d", pin);
  gpio_set_level(pin, 0);
}
