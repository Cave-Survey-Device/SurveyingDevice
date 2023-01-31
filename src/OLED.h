// OLED.h
#ifndef OLED_h
#define OLED_h

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

class OLED {
  public:

    int rand_no;
    OLED();
    void Distance(double distance);
    void Compass();
    void Clino();
    void Blutooth(bool ble_status);
    void Battery(double batt_voltage);
    void Initialise();

  private:
    
    #define SCREEN_WIDTH 128  // OLED display width, in pixels
    #define SCREEN_HEIGHT 128 // OLED display height, in pixels
    #define OLED_RESET -1     // can set an oled reset pin if desired
    Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 1000000, 100000);
    
};

#endif