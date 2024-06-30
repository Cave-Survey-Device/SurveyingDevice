// OLED.h
#ifndef OLED_H
#define OLED_H





#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

namespace OLED
{

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 128 // OLED display height, in pixels
#define OLED_RESET -1     // can set an oled reset pin if desired
#define TOP_BAR_HEIGHT 28

struct Point
{
  Point(int px, int py);
  int x, y;
};

Point rotatePoint(const Point p, const int cx, const int cy , const float rads);

enum CompassDirection
{
  NORTH,
  NORTH_EAST,
  EAST,
  SOUTH_EAST,
  SOUTH,
  SOUTH_WEST,
  WEST,
  NORTH_WEST
};


class DisplayHandler {
  public:

    int rand_no;
    DisplayHandler();
    void Distance(double distance);
    void Compass(double compass);
    void Clino(double clino);
    void Sensor_cal_status(int sensor_status);
    void Blutooth(bool ble_status);
    void Battery(int batt_percentage);
    void Initialise();
    void clearDisplay();
    void update();
    

    void displayTopBar(bool bluetooth, int battery_level, int status);
    void drawCompass();
    void drawCompassDirection(CompassDirection direction);
    void displayOrientation();
    void displayShot();
    void displayCalibSaveYN();
    void displayData();

    double batt_voltage;
    int batt_level;
    int batt_percentage;
    double distance;
    double compass;
    double clino;
    int sensor_status;

  private:
    
    const int line_length = 40;
    const int cx = SCREEN_WIDTH/2;
    const int cy = TOP_BAR_HEIGHT + (SCREEN_HEIGHT - TOP_BAR_HEIGHT)/2;

    Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 1000000, 100000);
    //Adafruit_SSD1327 display = Adafruit_SSD1327(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 1000000);

};


};

#endif