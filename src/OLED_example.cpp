// main code: ArduinoCode.ino
#include "OLED.h"

OLED oled; // create a OLED object

void setup() {
  Serial.begin(9600);
  oled.Initialise();
}

void loop() {

  double distance = random(0,100);
  double compass = random(0,359);
  double clino = random(-90,90);
  bool ble_status = random(0,100);
  double batt_voltage = random(0,100);

  oled.Distance(distance);
  oled.Compass(compass);
  oled.Clino(clino);
  oled.Blutooth(ble_status);
  delay(100);
  oled.Battery(batt_voltage);
  delay(100);

}