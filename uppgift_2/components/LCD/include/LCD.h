
#include <stdio.h>
#include <string>
#include "driver/i2c.h"

#ifndef LCD_H

#define LCD_H

// LCD class
class LCD
{
private:
  i2c_port_t i2c_port_num;

  // Private methods
  void send_cmd(uint8_t cmd);
  void send_data(uint8_t data);
  esp_err_t write_byte(uint8_t byte, uint8_t control);
  // esp_err_t write_buffer(uint8_t[] byte, uint8_t control);

public:
  LCD();

  // Destructor
  ~LCD();

  // Public methods
  void initialize();
  void deinitialize();
  void display_text(const std::string &text);
  void display_on();
  void display_off();
  void clear_display();
  void return_home();
  void set_cursor(uint8_t row, uint8_t col);
  uint8_t get_address();
};

#endif
