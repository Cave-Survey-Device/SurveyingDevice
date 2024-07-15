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
  NORTH_WEST,
  UP,
  DOWN
};

const char directionsArr[10][11] 
{
  "   NORTH  ",
  "NORTH EAST",
  "   EAST   ",
  "SOUTH EAST",
  "   SOUTH  ",
  "SOUTH WEST",
  "   WEST   ",
  "NORTH WEST",
  "    UP    ",
  "   DOWN   "
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
    void drawCentered(String str, int cx, int cy,int size);
    void drawStaticCalib(CompassDirection pointing, CompassDirection facing, const char progress[5] = "");
    void drawLaserCalib(const float angle, const char centre_text[7], const char progress[5]);
    void drawCompass(int cx, int cy, int line_length, int arrow_length);
    void drawCompassDirection(int cx, int cy, int line_length, int arrow_length, CompassDirection direction);
    void displayOrientation();
    void displayShot();
    void displayYN(const char prompt[11], bool YN);
    void displayYN(const char prompt_top[11], const char prompt_btm[11], bool YN);
    void displayData();
    void displayLoading(const char prompt_top[11], const char prompt_btm[11], int count);

    double batt_voltage;
    int batt_level;
    int batt_percentage;
    double distance;
    double compass;
    double clino;
    int sensor_status;

  private:
    
    const int canvas_center_x = SCREEN_WIDTH/2;
    const int canvas_center_y = TOP_BAR_HEIGHT + (SCREEN_HEIGHT - TOP_BAR_HEIGHT)/2;

    Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
    //Adafruit_SSD1327 display = Adafruit_SSD1327(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 1000000);

};


};

#endif