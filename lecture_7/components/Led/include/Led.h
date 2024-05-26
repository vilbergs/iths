#ifndef LED_H

#define LED_H
#include "driver/gpio.h"

class Led
{
  typedef enum ledState_e
  {
    LS_NONE = 0,
    LS_BLINKING_ON,
    LS_BLINKING_OFF
  } ledState_e;

private:
  gpio_num_t pin;
  unsigned int on;
  unsigned int off;
  uint64_t timer;
  ledState_e state;

public:
  Led(gpio_num_t pin);
  ~Led();

  void update();

  void On();
  void Off();

  void blink(unsigned int on, unsigned int off);
};

#endif
