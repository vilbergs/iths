#ifndef SERVO_H

#define SERVO_H
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"

class Servo
{
private:
  gpio_num_t pin;
  unsigned int angle;

public:
  Servo(gpio_num_t pin);
  ~Servo();

  void update(unsigned int angle);
};

#endif
