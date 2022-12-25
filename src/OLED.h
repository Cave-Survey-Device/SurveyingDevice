#ifndef HEADER_OLED
#define HEADER_OLED

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "config.h"

#ifndef SCREEN_WIDTH
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#endif

#ifndef SCREEN_HEIGHT
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#endif

#ifndef OLED_RESET
#define OLED_RESET -1
#endif

Adafruit_SSD1306 init_OLED(uint8_t address);

#endif