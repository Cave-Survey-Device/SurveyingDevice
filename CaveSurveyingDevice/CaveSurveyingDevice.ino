#include <ArduinoEigenDense.h>

#include <NumericalMethods_csd.h>
#include <config_csd.h>
#include <interrupts_csd.h>
#include <utility_csd.h>

#include <sensors_csd.h>

#include <SCL3300.h>

#include <RM3100.h>

static InertialSensor mag(&mag_sc2);
static SCL3300 inclinometer;
static RM3100 mag_sc2;
Vector3f data;
// static InertialSensorConnection acc_sc(acc_true_vec, Ta, ha, TAmisalign);

// static InertialSensor acc(&acc_sc);
// static SensorHandler sh(&acc, &mag);


void setup() {
  
  Serial << "Init Mag\n";
  mag_sc2.init();

  Serial << "Init accel\n";
  #if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  #define Serial SERIAL_PORT_USBVIRTUAL
  #endif

  while (inclinometer.begin() == false) {
      Serial.println("Murata SCL3300 inclinometer not connected.");;
  }

  Serial << "Init finished\n";
}

void loop() {
  Serial << "Begin loop\n";

  while (true)
  {
      Serial << "Getting accel data\n";
      if (inclinometer.available()) { //Get next block of data from sensor
          Serial.print("X Accelerometer: ");
          Serial.print(inclinometer.getCalculatedAccelerometerX());
          Serial.print("\t");
          Serial.print("Y Accelerometer: ");
          Serial.print(inclinometer.getCalculatedAccelerometerY());
          Serial.print("\t");
          Serial.print("Z Accelerometer: ");
          Serial.println(inclinometer.getCalculatedAccelerometerZ());
          delay(250); //Allow a little time to see the output
      } else inclinometer.reset();

      // data = mag_sc2.GetRawData();
      Serial << "Getting mag data\n";
      displayVec(mag.GetReading());

      delay(1000);
  }
}