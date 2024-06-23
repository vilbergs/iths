#include <stdio.h>
#include <string>
#include "LCD.h"
#include "driver/i2c_master.h"
#include "I2CBus.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define LCD_DISPLAY_ON 0x0C  // Display on, cursor off, blink off
#define LCD_DISPLAY_OFF 0x08 // Display off, cursor off, blink off
#define LCD_DISPLAY_CLEAR 0x01
#define LCD_FUNCTION_SET_4_bit 0x28 // Function set: 4-bit, 2-line, 5x8 dots
#define LCD_FUNCTION_SET 0x30       // Function set: 8-bit, 1-line
#define LCD_FUNCTION_SET_2 0x38     // Function set: 8-bit, 2-line
#define LCD_ENABLE_BIT 0x04
#define LCD_BACKLIGHT 0x08

#define LCD_DEVICE_ADDR 0x27

static const char *TAG = "LCD";

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

LCD::LCD() {}

// rs = 0 for command, rs = 1 for data
esp_err_t LCD::write_byte(uint8_t byte, uint8_t rs)
{
  uint8_t write_data[4];

  uint8_t top_nibble = byte & 0xF0;
  uint8_t bottom_nibble = (byte << 4) & 0xF0;

  write_data[0] = top_nibble | 0x0C | rs;             // Set EN bit high. and keep backlight on
  write_data[1] = top_nibble | LCD_BACKLIGHT | rs;    // Set EN bit low and keep backlight on
  write_data[2] = bottom_nibble | 0x0C | rs;          // Set EN bit high and keep backlight on
  write_data[3] = bottom_nibble | LCD_BACKLIGHT | rs; // Set EN bit low and keep backlight on

  return i2c_master_write_to_device(I2C_MASTER_NUM, LCD_DEVICE_ADDR, write_data, sizeof(write_data), 1000);
}

void LCD::send_cmd(uint8_t cmd)
{
  esp_err_t err = write_byte(cmd, 0x00); // rs = 0 for command

  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "Error sending command to LCD: %d", err);
  }
}

void LCD::send_data(uint8_t data)
{
  write_byte(data, 0x01); // rs = 0 for command
  vTaskDelay(pdMS_TO_TICKS(1));
}

void LCD::initialize()
{
  ESP_LOGI(TAG, "Initializing LCD");
  vTaskDelay(pdMS_TO_TICKS(50));
  send_cmd(0x30);
  vTaskDelay(pdMS_TO_TICKS(4.5));
  send_cmd(0x30);
  vTaskDelay(pdMS_TO_TICKS(1));
  send_cmd(0x30);
  vTaskDelay(pdMS_TO_TICKS(1));
  send_cmd(0x20);
  vTaskDelay(pdMS_TO_TICKS(1));

  send_cmd(LCD_FUNCTION_SET_4_bit);

  vTaskDelay(pdMS_TO_TICKS(1));
  display_off();

  vTaskDelay(pdMS_TO_TICKS(1));
  display_on();

  ESP_LOGI(TAG, "Clearing Display");
  clear_display();

  // Entry mode set: increment, no shift
  send_cmd(0x06);

  vTaskDelay(pdMS_TO_TICKS(10));
  ESP_LOGI(TAG, "Initialized LCD");
}

void LCD::deinitialize()
{
  clear_display();
  display_off();
}

void LCD::display_on()
{
  send_cmd(LCD_DISPLAY_ON); // Display control: display on, cursor off, blink off
  vTaskDelay(pdMS_TO_TICKS(2));
}

void LCD::display_off()
{
  send_cmd(LCD_DISPLAY_OFF); // Display on/off control --> D=0,C=0, B=0  ---> display off
  vTaskDelay(pdMS_TO_TICKS(2));
}

void LCD::display_text(const std::string &text)
{

  for (size_t i = 0; i < text.length(); i++)
  {
    // Print hex value with ESP_LOGI

    send_data(text[i]);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void LCD::clear_display()
{
  send_cmd(0x01);
  vTaskDelay(pdMS_TO_TICKS(2));
}

void LCD::return_home()
{
  send_cmd(0x02);
  vTaskDelay(pdMS_TO_TICKS(2));
}

void LCD::set_cursor(uint8_t row, uint8_t col)
{
  uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
  send_cmd(0x80 | (col + row_offsets[row]));
  vTaskDelay(pdMS_TO_TICKS(2));
}

uint8_t LCD::get_address()
{
  return LCD_DEVICE_ADDR;
}
