// main code: ArduinoCode.ino
#include "OLED.h"

OLED oled; // create a OLED object

void setup() {
  Serial.begin(9600);
  oled.Initialise();
}

void loop() {

  double distance = 10.23;
  bool ble_status = 1;
  double batt_voltage;

  oled.Distance(distance);
  delay(3000);
  oled.Compass();
  delay(300);
  oled.Clino();
  delay(300);
  oled.Blutooth(ble_status);
  delay(50);
  oled.Battery(batt_voltage);
  delay(300);

}