#include <SensorHandler.h>
#include <OLED.h>
#include "RM3100SensorConnection.h"
#include "SCA3300SensorConnection.h"
#include "LDK2MSensorConnection.h"
#include "utils.h"
#include "FreeRTOS.h"
#include "BLE.h"
#include <ArduinoJson.h>

static RM3100 rm3100;
static SCA3300 sca3300;
static LDK_2M ldk2m;

static SCA3300SensorConnection sc_accelerometer(sca3300);
static RM3100SensorConnection sc_magnetometer(rm3100);
static LDK2MSensorConnection sc_laser(ldk2m);

static OLED::DisplayHandler oled;

static SensorHandler sh(sc_accelerometer, sc_magnetometer, sc_laser);
// static BLEHandler bh;

typedef Matrix<float, Dynamic, Dynamic, RowMajor> RowMatrixXf;
static StaticJsonDocument<60480> root;
static float eig_arr[3][N_ALIGN_MAG_ACC];
static float las_arr[3][N_LASER_CAL];

static unsigned int current_shot_ID = 0;
static unsigned int current_file_ID = 0;
static ShotData current_shot_data;

int begin()
{
    Serial.begin(115200);
    sh.init();
    oled.Initialise();
    return 0;
}

int takeShot()
{
    Serial.print("Taking shot...\n");
    if (!sh.takeShot())
    {
        Serial.print("Saving shot...\n");
        current_shot_data = sh.getShotData();
        saveShotData(current_shot_data,current_file_ID);
        // Read data after saving to get the ID assigned to it
        readShotData(current_shot_data,current_file_ID);
        // Write data to BLE
        // bh.shared_ble_data.write_data(&current_shot_data);
        return 0;
    }
    return 1;
}

int removeRecentCalib()
{
    if (sh.getCalibProgress(true) < N_ORIENTATIONS)
    {
        sh.removePrevCalib(true);
    } else {
        sh.removePrevCalib(false);
    }
    return 0;
}

int nextCalib(){
    // Check static calib progress, if > N_ORIENTATIONS, return N_ORIENTATIONS + laser calib progress
    // Otherwise return static calib progress
    debug(DEBUG_ALWAYS, "nextCalb()...");
    if (sh.getCalibProgress(true) < N_ORIENTATIONS)
    {
        debug(DEBUG_ALWAYS, "static calib mode");
        return sh.collectStaticCalibData();
    } else {
        debug(DEBUG_ALWAYS, "laser calib mode");
        return N_ORIENTATIONS + sh.collectLaserCalibData();
    }
    return 0;
}

int prevData()
{
    if (current_shot_ID - 1 >= 0) { current_shot_ID -= 1; }
    readShotData(current_shot_data,current_file_ID,current_shot_ID);
    return 0;
}

int nextData()
{
    static unsigned int counter;
    static char fname[FNAME_LENGTH];
    getFileName(current_file_ID,fname);
    getCounter(current_file_ID, counter);    

    if (current_shot_ID + 1 <= counter ) { current_shot_ID += 1; }
    readShotData(current_shot_data,current_file_ID,current_shot_ID);
    return 0;
}

int laserOn()
{
    sc_laser.toggleLaser(true);
    return 0;
}

int laserOff()
{
    sc_laser.toggleLaser(false);
    return 0;
}

int historyScrollUp(){
    return 0;
}

int historyScrollDown(){
    return 0;
}

int saveCalib()
{
    sh.saveCalibration();
    return 0;
}

void init()
{
    Serial.begin(115200);
    Serial << "Begin beaning...\n\n";
    sh.init();

    Serial << "SensorHandler initialised, initialising OLED...\n\n";
    oled.Initialise();

    Serial << "OLED initialised, testing magnetometer and accelerometer...\n";
    Serial.printf("Mag data: %f %f %f\n",sc_magnetometer.getMeasurement()(0),sc_magnetometer.getMeasurement()(1),sc_magnetometer.getMeasurement()(2));
    Serial.printf("Aacc data: %f %f %f\n",sc_accelerometer.getMeasurement()(0),sc_accelerometer.getMeasurement()(1),sc_accelerometer.getMeasurement()(2));

    Serial << "Initialisation succesful, starting tasks...\n\n";
}


int displayLoading()
{
    oled.clearDisplay();
    static int count = 0;
    count += 1;
    oled.displayLoading("Collecting","data...",count);
    oled.update();
    return 0;
}

int displayOrientation()
{
    static ShotData sd;
    sh.update();
    sd = sh.getShotData();

    oled.clearDisplay();
    oled.Distance(sd.d);
    oled.Clino(sd.HIR(1));
    oled.Compass(sd.HIR(0));
    oled.update();
    return 0;
}

int displayCalib()
{
    oled.clearDisplay();
    switch(sh.getCalibProgress(true))
    {
        case 0:
        oled.drawStaticCalib(OLED::CompassDirection::SOUTH,OLED::CompassDirection::UP,"1/12");
        break;

        case 1:
        oled.drawStaticCalib(OLED::CompassDirection::WEST,OLED::CompassDirection::UP,"2/12");
        break;

        case 2:
        oled.drawStaticCalib(OLED::CompassDirection::WEST,OLED::CompassDirection::DOWN,"3/12");
        break;

        case 3:
        oled.drawStaticCalib(OLED::CompassDirection::NORTH_WEST,OLED::CompassDirection::DOWN,"4/12");
        break;

        case 4:
        oled.drawStaticCalib(OLED::CompassDirection::EAST,OLED::CompassDirection::SOUTH,"5/12");
        break;

        case 5:
        oled.drawStaticCalib(OLED::CompassDirection::SOUTH,OLED::CompassDirection::WEST,"6/12");
        break;

        case 6:
        oled.drawStaticCalib(OLED::CompassDirection::NORTH,OLED::CompassDirection::WEST,"7/12");
        break;

        case 7:
        oled.drawStaticCalib(OLED::CompassDirection::NORTH_EAST,OLED::CompassDirection::NORTH_WEST,"8/12");
        break;

        case 8:
        oled.drawStaticCalib(OLED::CompassDirection::UP,OLED::CompassDirection::EAST,"9/12");
        break;

        case 9:
        oled.drawStaticCalib(OLED::CompassDirection::UP,OLED::CompassDirection::SOUTH,"10/12");
        break;

        case 10:
        oled.drawStaticCalib(OLED::CompassDirection::DOWN,OLED::CompassDirection::NORTH,"11/12");
        break;

        case 11:
        oled.drawStaticCalib(OLED::CompassDirection::DOWN,OLED::CompassDirection::NORTH_EAST,"12/12");
        break;
    }
    oled.update();    
    return 0;
}

int displayRecentShot()
{
    return 0;
}

int displayRemCalibYN(bool YN)
{
    oled.clearDisplay();
    oled.displayYN("Remove last","data?", YN);
    oled.update();
    return 0;
}

int displaySaveCalibYN(bool YN)
{
    oled.clearDisplay();
    oled.displayYN("Save calib","-ration?", YN);
    oled.update();
    return 0;
}

/**
 * @brief Setup task for Arduino framework
 * 
 */
void setup()
{
    init();
    delay(1000);
    xTaskCreatePinnedToCore(
        eventhandler, /* Function to implement the task */
        "eventhandler", /* Name of the task */
        10000,  /* Stack size in words */
        NULL,  /* Task input parameter */
        3 ,  /* Priority of the task */
        &eventhandler_task,  /* Task handle. */
        0); /* Core where the task should run */

    delay(500);
    xTaskCreatePinnedToCore(
        computefunc, /* Function to implement the task */
        "computefunc", /* Name of the task */
        50000,  /* Stack size in words */
        NULL,  /* Task input parameter */
        2 ,  /* Priority of the task */
        &computefunc_task,  /* Task handle. */
        0); /* Core where the task should run */

    delay(500);
    xTaskCreatePinnedToCore(
        displayhandler, /* Function to implement the task */
        "passivehandler", /* Name of the task */
        10000,  /* Stack size in words */
        NULL,  /* Task input parameter */
        4 ,  /* Priority of the task */
        &displayhandler_task,  /* Task handle. */
        0); /* Core where the task should run */


    // delay(500);
    // xTaskCreatePinnedToCore(
    //     blehandler, /* Function to implement the task */
    //     "blehandler", /* Name of the task */
    //     10000,  /* Stack size in words */
    //     NULL,  /* Task input parameter */
    //     1 ,  /* Priority of the task */
    //     &BLEhandler_task,  /* Task handle. */
    //     0); /* Core where the task should run */



        init_interrupts();
}

OLED::CompassDirection cd = OLED::NORTH;
int count = 0;
void loop(){
    // cd = (OLED::CompassDirection) ((int)cd + 1);
    // if ((int)cd > 7)
    // {
    //     cd = (OLED::CompassDirection)0;
    // }
    // oled.clearDisplay();
    // // oled.drawCompass();
    // // oled.drawCompassDirection(cd);
    // oled.drawCalib(cd,cd);
    // oled.update();
    // delay(500);

    // oled.clearDisplay();
    // oled.displayYN("Save data?",false);
    // oled.update();

    // delay(1500);

    // oled.clearDisplay();
    // oled.displayYN("Save data?",true);
    // oled.update();

    // delay(1500);

    // oled.clearDisplay();
    // oled.displayYN("Save calib","-ration?",false);
    // oled.update();

    // delay(1500);

    // oled.clearDisplay();
    // oled.displayYN("Save calib","-ration?",true);
    // oled.update();

    // delay(500);

    // oled.clearDisplay();
    // oled.displayLoading("Collecting", "data...", count);
    // Serial.println(count);
    // count += 1;
    // oled.update();
    // delay(250);


}