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





bool calibrated = 0;
int cmd = 0;

void test_main(void * parameter) {
  while(true){
    while (Serial.available() == 0) {
    }

    cmd = Serial.parseInt();

    if (cmd == 2)
    {
      Serial << "RM3100 sample data\n";
      displayMat(mag.GetCalibData().transpose());

      Serial << "\nRM3100 calibrated data\n";
      displayMat(     (  mag.GetT() * (mag.GetCalibData().colwise() - mag.Geth())  ).transpose()    );

      Serial << "\nRM3100 T\n";
      displayMat( mag.GetT() );

      Serial << "\nRM3100 h\n\n";
      displayMat( mag.Geth() );

      Serial << "-----------------------------------------------------------\n\n";
      
      Serial << "SCA3300 sample data\n";
      displayMat(acc.GetCalibData().transpose());

      Serial << "SCA3300 calibrated data\n";
      displayMat(     (  acc.GetT() * (acc.GetCalibData().colwise() - acc.Geth())  ).transpose()    );

      Serial << "SCA3300 T\n";
      displayMat( acc.GetT() );

      Serial << "SCA3300 h\n\n";
      displayMat( acc.Geth() );

    } else if (cmd == 3) {
      Serial << "Calibrating inertial sensors...\n";
      sh.CalibrateInertial();

    } else if (cmd == 1) {
      Serial << "Get calibration data...\n";
      calibrated = sh.CollectCalibrationData();
      if (calibrated = 0)
      {
        Serial << "Calibrating inertial sensors...\n";
        sh.CalibrateInertial();
        // Serial << "SCA3300 sample data\n";
        // displayMat(acc.GetCalibData().transpose());
        // Serial << "\nSCA3300 calibrated data\n";
        // displayMat(     (  acc.GetT() * (acc.GetCalibData().colwise() - acc.Geth())  )    );

        // MatrixXf calibrated_acc_data = (  acc.GetT() * (acc.GetCalibData().colwise() - acc.Geth())  );
        // serialPlotVec(acc.GetCalibData().colwise().norm(), calibrated_acc_data.colwise().norm(), "AccSamples", "AccCorrections");

        // Serial << "\n\n";

        Serial << "RM3100 sample data\n";
        displayMat(mag.GetCalibData().transpose());
        Serial << "\nRM3100 calibrated data\n";
        displayMat(     (  mag.GetT() * (mag.GetCalibData().colwise() - mag.Geth())  )    );

        MatrixXf calibrated_mag_data = (  mag.GetT() * (mag.GetCalibData().colwise() - mag.Geth())  );
        serialPlotVec(mag.GetCalibData().colwise().norm(), calibrated_mag_data.colwise().norm(), "MagSamples", "MagCorrections");
      } else if (calibrated == -1)
      {
        Serial << "Calibration sample failed! Please keep the device steady.\n";
      } else {
        Serial << "Calibration sample succesful.\n";
      }
    }
    cmd = 0;
  }
}

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

  TaskHandle_t hardware_handle;
  xTaskCreatePinnedToCore(
      test_main, /* Function to implement the task */
      "main", /* Name of the task */
      50000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      tskIDLE_PRIORITY ,  /* Priority of the task */
      &hardware_handle,  /* Task handle. */
      0); /* Core where the task should run */
}

void loop(){}

// #include "Arduino.h"
// #include "test.h"

// void setup()
// {
//   Serial.begin(115200);
//   delay(1000);  
//   Serial.print("STARTING BAYBEE - TEST_SENSORS_MODE\n\n");

//   TaskHandle_t hardware_handle;
//   xTaskCreatePinnedToCore(
//       test_main, /* Function to implement the task */
//       "test_main", /* Name of the task */
//       40000,  /* Stack size in words */
//       NULL,  /* Task input parameter */
//       tskIDLE_PRIORITY,  /* Priority of the task */
//       &hardware_handle,  /* Task handle. */
//       0); /* Core where the task should run */
// }

// void loop(){
// }