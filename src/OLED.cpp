// OLED.cpp
#include "OLED.h"

OLED::OLED() {
}

void OLED::Initialise() {
  display.begin(0x3D, true); // Address 0x3D default
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SH110X_WHITE);        // Draw white text
  //display.setContrast (0); // dim display
  display.clearDisplay();
  display.display();
}

void OLED::Distance(double distance) {
  display.clearDisplay();
  display.setCursor(0,0);// Start at top-left corner
  display.println(distance);
  display.display();
}

void OLED::Compass() {
  display.clearDisplay();
  display.setCursor(0,0);// Start at top-left corner
  display.println(F("Hello, world!"));
  display.display();
}

void OLED::Clino() {
  display.clearDisplay();
  display.setCursor(0,20);// Start at top-left corner
  display.println(F("Hello, world!"));
  display.display();
}

void OLED::Blutooth(bool ble_status) {

  //insert switch case to updat

  display.clearDisplay();
  display.setCursor(0,40);// Start at top-left corner
  display.println(F("Hello, world!"));
  display.display();
}

void OLED::Battery(double batt_voltage) {
  display.clearDisplay();
  display.setCursor(0,60);// Start at top-left corner
  display.println(F("Hello, world!"));
  display.display();
}