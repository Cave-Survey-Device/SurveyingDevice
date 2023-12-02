#include <ArduinoEigenDense.h>

#include <NumericalMethods_csd.h>
#include <config_csd.h>
#include <interrupts_csd.h>
#include <utility_csd.h>

#include <sensorhandler.h>

#include <SCA3300SensorConnection.h>
#include <RM3100SensorConnection.h>
#include <LDK2MSensorConnection.h>

#include <accelerometer_csd.h>
#include <magnetometer_csd.h>
#include <LDK_2M.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <OLED.h>

#include <filesystem_csd.h>

bool available = false;
bool btn_press = false;
bool acc_calib = false;
bool mag_calib = false;


static RM3100 rm3100;
static SCA3300 sca3300;
static LDK_2M ldk_2m;

static RM3100SensorConnection mag_sc(&rm3100);
static SCA3300SensorConnection acc_sc(&sca3300);
static LDK2MSensorConnection las(&ldk_2m);

static Magnetometer mag(&mag_sc);
static Accelerometer acc(&acc_sc);
static SensorHandler sh(&acc, &mag, &las);

static OLED oled; // create a OLED object

float distance = 1.32;
float compass = 234;
float clino = -23;
bool ble_status =  true;

float batt_percentage = 25;
static Vector3f data;

bool calibrated = 0;
// int cmd = 0;


void serialPrintCalibData()
{
  // Serial << "RM3100 sample data\n";
  // displayMat(mag.getCalibData().transpose());

  // Serial << "\nRM3100 calibrated data\n";
  // displayMat(     (  mag.getCalibMat() * (mag.getCalibData().colwise() - mag.getCalibBias())  ).transpose()    );

  Serial << "\nRM3100 T\n";
  displayMat( mag.getCalibMat() );

  Serial << "\nRM3100 h\n\n";
  displayMat( mag.getCalibBias() );

  Serial << "-----------------------------------------------------------\n\n";
  
  // Serial << "SCA3300 sample data\n";
  // displayMat(acc.getCalibData().transpose());

  // Serial << "SCA3300 calibrated data\n";
  // displayMat(     (  acc.getCalibMat() * (acc.getCalibData().colwise() - acc.getCalibBias())  ).transpose()    );

  Serial << "SCA3300 T\n";
  displayMat( acc.getCalibMat() );

  Serial << "SCA3300 h\n\n";
  displayMat( acc.getCalibBias() );
}

void serialPrintAlignData()
{
}

void runAlignment()
{
  sh.align();
}

void collectCalibData()
{
  Serial << "Get calibration data...\n";
  calibrated = sh.collectCalibration();
  if (calibrated == -1)
  {
    Serial << "Calibration sample failed! Please keep the device steady.\n";
  } else if (calibrated == 0){
    Serial << "Calibration sample succesful.\n";
  } else {
    Serial << "Calibration finished\n";
  }
}

void collectAlignData()
{
  Serial << "Get alignment data...\n";
  sh.collectAlignment();
}

void takeShot()
{
  Serial << "Taking shot...\n";
  sh.takeShot();
  Serial << "Shot data: ";
  displayRowVec(sh.getShotData());
  sh.disableLaser();
  Serial << "\n";
}


String strcmd;
const char* cmd;
void serialEvent()
{
  // cmd = Serial.parseInt();
  strcmd = Serial.readStringUntil('\n');
}

void test_main(void * parameter) {
  while(true){
    cmd = strcmd.c_str();
    Serial.println(cmd);
    if (!strcmp(cmd, "getCalibData"))
      serialPrintCalibData();
    else if (!strcmp(cmd, "getAlignData"))
      serialPrintAlignData();
    else if (!strcmp(cmd, "collectCalibData"))
      collectCalibData();
    else if (!strcmp(cmd, "collectAlignData"))
      collectAlignData();
    else if (!strcmp(cmd, "calibrate"))
      sh.calibrate();
    else if (!strcmp(cmd, "align"))
      sh.align();
    else if (!strcmp(cmd, "takeShot"))
      takeShot();
    else if (!strcmp(cmd, "laserOn"))
      las.toggleLaser(true);
    else if (!strcmp(cmd, "laserOff"))
      las.toggleLaser(false);
    else if (!strcmp(cmd, "saveCalibData"))
      sh.saveCalibration();
    else if (!strcmp(cmd, "loadCalibData"))
      sh.loadCalibration();
    else if (!strcmp(cmd, "saveAlignData"))
      sh.saveAlignment();
    else if (!strcmp(cmd, "loadAlignData"))
      sh.loadAlignment();
    else if (!strcmp(cmd, "updateInertial"))
      data = sh.update();
    else if (!strcmp(cmd, "eraseFlash"))
      erase_flash();
    else {}

    data = sh.update();
    oled.Battery(batt_percentage);
    oled.Compass(data(0));
    oled.Clino(data(1));
    oled.Distance(data(2));
    oled.Blutooth(ble_status);
    strcmd = "";
  }
}

void blank_main(void * parameter)
{
  while(true)
  {
    Serial.print("blank_main\n");
    delay(2000);
  }
}

void setup() {
  Serial.begin(115200);
  
  // Initialise sensors
  rm3100.begin();
  las.begin();

  #if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  #define Serial SERIAL_PORT_USBVIRTUAL
  #endif

  while (sca3300.begin() == false) {
      Serial.println("Murata SCL3300 inclinometer not connected.");;
  }

  // Initialise peripherals
  Serial << "Init OLED...\n";
  oled.Initialise();
  delay(250); // wait for the OLED to power up
  oled.clearDisplay();
  oled.Battery(batt_percentage);
  oled.Distance(distance);
  oled.Compass(compass);
  oled.Clino(clino);
  oled.Blutooth(ble_status);

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

  sh.resetCalibration();
  sh.resetAlignment();
}

void loop(){}
