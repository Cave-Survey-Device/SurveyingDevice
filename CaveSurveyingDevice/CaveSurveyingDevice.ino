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

static SensorHandler sh(&acc, &mag);

Vector3f data;

bool available = false;
bool btn_press = false;
bool acc_calib = false;
bool mag_calib = false;

void int_btn_press()
{
  btn_press = true;
}

void setup() {
  pinMode(T4, INPUT);
  attachInterrupt(digitalPinToInterrupt(T4), int_btn_press, RISING);


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




void loop() {
  if (btn_press)
  {
    Serial << "btn_press\n";
    btn_press = false;
    delay(100);

    if (mag.ColectCalibrationSample())
    {
      mag_calib = true;
      Serial.print("MAG: Data collected, calibrating...\n");
      mag.CalibrateLinear();
    }

    if (acc.ColectCalibrationSample())
    {
      acc_calib = true;
      Serial.print("ACC: Data collected, calibrating...\n");
      acc.CalibrateLinear();
    }
    
    if (acc_calib && mag_calib)
    {
      Serial << "SCA3300 sample data\n";
      displayMat(acc.GetCalibData());
      Serial << "\nSCA3300 calibrated data\n";
      displayMat(     (  acc.GetT() * (acc.GetCalibData().colwise() - acc.Geth())  ).transpose()    );

      Serial << "\n\n";

      Serial << "RM3100 sample data\n";
      displayMat(mag.GetCalibData());
      Serial << "\nRM3100 calibrated data\n";
      displayMat(     (  mag.GetT() * (mag.GetCalibData().colwise() - mag.Geth())  ).transpose()    );
    }

    
  }

  Serial << "\nGetting mag reading\n";
  displayVec(mag.GetReading());

  Serial << "\nGetting acc reading\n";
  displayVec(acc.GetReading());

  Serial << "\n\n";

  delay(1000);
}