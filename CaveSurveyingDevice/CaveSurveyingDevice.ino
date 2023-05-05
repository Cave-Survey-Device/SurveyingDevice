#include <ArduinoEigenDense.h>

#include <NumericalMethods_csd.h>
#include <config_csd.h>
#include <interrupts_csd.h>
#include <utility_csd.h>

#include <sensors_csd.h>

#include <SCA3300SensorConnection.h>
#include <RM3100SensorConnection.h>


static RM3100 rm3100;
static SCA3300 sca3300;

static RM3100SensorConnection mag_sc(&rm3100);
static SCA3300SensorConnection acc_sc(&sca3300);

static InertialSensor mag(&mag_sc);
static InertialSensor acc(&acc_sc);

// static SensorHandler sh(&acc, &mag);

Vector3f data;



void setup() {
  Serial.begin(115200);

  Serial << "Init Mag\n";
  rm3100.begin();

  Serial << "Init accel\n";
  #if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  #define Serial SERIAL_PORT_USBVIRTUAL
  #endif

  while (sca3300.begin() == false) {
      Serial.println("Murata SCL3300 inclinometer not connected.");;
  }

  Serial << "Init finished\n";
}

bool available = false;
void loop() {
  Serial << "Begin loop\n";
  available = false;

  // if (sca3300.available()) { //Get next block of data from sensor
  //     Serial.print("X Accelerometer: ");
  //     Serial.print(sca3300.getCalculatedAccelerometerX());
  //     Serial.print("\t");
  //     Serial.print("Y Accelerometer: ");
  //     Serial.print(sca3300.getCalculatedAccelerometerY());
  //     Serial.print("\t");
  //     Serial.print("Z Accelerometer: ");
  //     Serial.println(sca3300.getCalculatedAccelerometerZ());
  //     delay(250); //Allow a little time to see the output
  //   } else sca3300.reset();

    // Serial << "Getting accel data\n";
    
    // while (!available)
    // {
    //     Serial << "avaliable()\n";
    //     available = sca3300.available();
    //     if (!available) {
    //         sca3300.reset();
    //     }
    // }
    //   Serial << "Raw accel data: " << sca3300.getCalculatedAccelerometerX() << " " << sca3300.getCalculatedAccelerometerY() << " " << sca3300.getCalculatedAccelerometerZ() << "\n";

    //   displayVec(acc_sc.GetRawData());

    //   rm3100.update();
    //   Serial << "RM3100 raw mag data\n" << rm3100.getX() << "\t" << rm3100.getY() << "\t" << rm3100.getZ() << "\n\n";



    //   Serial << "Getting sc raw mag data\n";
    //   displayVec(mag_sc.GetRawData());

      Serial << "\nGetting mag reading\n";
      displayVec(mag.GetReading());

      Serial << "\nGetting acc reading\n";
      displayVec(acc.GetReading());

      Serial << "\n\n";

      delay(1000);
}