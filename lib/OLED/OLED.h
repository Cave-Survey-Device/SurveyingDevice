// OLED.h
#ifndef OLED_H
#define OLED_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

class OLED {
  public:

    int rand_no;
    OLED();
    void Distance(double distance);
    void Compass(double compass);
    void Clino(double clino);
    void Sensor_cal_status(int sensor_status);
    void Blutooth(bool ble_status);
    void Battery(int batt_percentage);
    void Initialise();
    void clearDisplay();
    
    double batt_voltage;
    int batt_level;
    int batt_percentage;
    double distance;
    double compass;
    double clino;
    int sensor_status;

  private:
    
    #define SCREEN_WIDTH 128  // OLED display width, in pixels
    #define SCREEN_HEIGHT 128 // OLED display height, in pixels
    #define OLED_RESET -1     // can set an oled reset pin if desired
    Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 1000000, 100000);
    //Adafruit_SSD1327 display = Adafruit_SSD1327(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 1000000);

};

#endif