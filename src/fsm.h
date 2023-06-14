#include <ArduinoEigenDense.h>

#include <NumericalMethods_csd.h>
#include <config_csd.h>
#include <utility_csd.h>

#include <sensorhandler.h>

#include <SCA3300SensorConnection.h>
#include <RM3100SensorConnection.h>

#include <accelerometer_csd.h>
#include <magnetometer_csd.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <OLED.h>


#define RESTART_NOTIFICATION 0xFFEEFFEE
#define NOTIFY_B1_SHORT_PRESS 1
#define NOTIFY_B1_LONG_PRESS 2
#define NOTIFY_B2_SHORT_PRESS 3
#define NOTIFY_B2_LONG_PRESS 4
#define NOTIFY_B1B2_SHORT_PRESS 5
#define NOTIFY_B1B2_LONG_PRESS 6


enum mode_enum {
    MODE_MENU,
    MODE_IDLE,
    MODE_LASER_ENA,
    MODE_ALIGN_LASER,
    MODE_CALIBRATE_INERTIAL,
    MODE_CALIBRATE_MAG,
    MODE_SAVE_LASER_ALIGN,
    MODE_SAVE_MAG_CALIBRATE,
    MODE_SAVE_INERTIAL_CALIB
};
static const char* mode_names[] = {
    "MODE_MENU",
    "MODE_IDLE",
    "MODE_LASER_ENA",
    "MODE_ALIGN_LASER",
    "MODE_CALIBRATE_INERTIAL",
    "MODE_CALIBRATE_MAG",
    "MODE_SAVE_LASER_ALIGN",
    "MODE_SAVE_MAG_CALIBRATE",
    "MODE_SAVE_INERTIAL_CALIB"
};

enum menu_enum {
    MENU_CALIBRATE_INERTIAL,
    MENU_CALIBRATE_MAG,
    MENU_ALIGN_LASER,
    MENU_SETTINGS
};
static const char* menu_names[] = {
    "MENU_CALIBRATE_INERTIAL",
    "MENU_CALIBRATE_MAG",
    "MENU_ALIGN_LASER",
    "MENU_SETTINGS"
};
const static int MENU_ENUM_MAX = 3;
inline menu_enum& operator++(menu_enum& menu)
{
    if (static_cast<int>(menu) + 1 > MENU_ENUM_MAX)
    {
        return menu = static_cast<menu_enum>(MENU_ENUM_MAX);
    } else {
        return menu = static_cast<menu_enum>( static_cast<int>(menu)+ 1 ); 
    }
}
menu_enum operator++(menu_enum& menu, int)
{
  menu_enum tmp(menu);
  ++menu;
  return tmp;
}
inline  menu_enum& operator--(menu_enum& menu)
{
    
    if (static_cast<int>(menu) - 1 < 0)
    {
        return menu = static_cast<menu_enum>(MENU_ENUM_MAX);
    } else {
        return menu = static_cast<menu_enum>( static_cast<int>(menu) - 1 ); 
    }
}
menu_enum operator--(menu_enum& menu, int)
{
  menu_enum tmp(menu);
  --menu;
  return tmp;
}

static mode_enum current_mode = MODE_IDLE;
static mode_enum next_mode = MODE_IDLE;

static menu_enum current_menu = MENU_CALIBRATE_INERTIAL;
static menu_enum next_menu = MENU_CALIBRATE_INERTIAL;

static RM3100 rm3100;
static SCA3300 sca3300;

static RM3100SensorConnection mag_sc(&rm3100);
static SCA3300SensorConnection acc_sc(&sca3300);

static Magnetometer mag(&mag_sc);
static Accelerometer acc(&acc_sc);
static SensorHandler sh(&acc, &mag);

static OLED oled; // create a OLED object

static uint32_t notification;

void checkForNotify(uint32_t& notifiedValue)
{
    xTaskNotifyWait(    0x00,      /* Don't clear any notification bits on entry. */
                        ULONG_MAX, /* Reset the notification value to 0 on exit. */
                        &notifiedValue, /* Notified value pass out in computefuncNotifiedValue. */
                        5 );  /* Block for 5ms. */
}

void menuSelect()
{
    switch (current_menu)
    {
    case MENU_ALIGN_LASER:
    debug(DEBUG_MAIN,"MENU_ALIGN_LASER");
    next_mode = MODE_ALIGN_LASER;
    sh.resetAlignment();
    break;
    
    case MENU_CALIBRATE_INERTIAL:
    debug(DEBUG_MAIN,"MENU_CALIBRATE_INERTIAL");
    next_mode = MODE_CALIBRATE_INERTIAL;
    sh.resetCalibration();
    break;

    case MENU_CALIBRATE_MAG:
    debug(DEBUG_MAIN,"MENU_CALIBRATE_MAG");
    next_mode = MODE_CALIBRATE_MAG;
    break;

    case MENU_SETTINGS:
    debug(DEBUG_MAIN,"MENU_SETTINGS");
    break;

    default:
    break;
    }
}

void b1ShortHold()
{
    // debug(DEBUG_MAIN,"b1ShortHold()");
    switch (current_mode)
    {
    case MODE_IDLE:
    debug(DEBUG_MAIN,"MODE_IDLE");
    next_mode = MODE_LASER_ENA;
    // sh.enableLaser(); 
    break;

    case MODE_LASER_ENA:
    debug(DEBUG_MAIN,"MODE_LASER_ENA");
    // sh.takeShot();
    break;

    case MODE_CALIBRATE_MAG:
    debug(DEBUG_MAIN,"MODE_CALIBRATE_MAG");
    // mag.setCalibMode(true);
    // while (true)
    // {
    //     checkForNotify(notification);
    //     if (notification == NOTIFY_B1_LONG_PRESS)
    //     {
    //         next_mode = MODE_SAVE_MAG_CALIBRATE;
    //         mag.calibrateLinear();
    //         mag.save_tmp_calibration_data();
    //         break;
    //     }   else if (notification == NOTIFY_B2_LONG_PRESS)
    //     {
    //         mag.resetCalibration();
    //         mag.load_calibration_data();
    //         break;
    //     }

    //     mag.addCalibrationData();
    //     // Display mag.checkCalibrationProgress()
    // }
    break;

    case MODE_CALIBRATE_INERTIAL:
    debug(DEBUG_MAIN,"MODE_CALIBRATE_INERTIAL");
    // if (sh.collectInertialAlignmentData())
    // {
    //     next_mode = MODE_SAVE_INERTIAL_CALIB;
    //     sh.alignInertial();
    // }
    // else
    // {
    //     next_mode = MODE_CALIBRATE_INERTIAL;
    // }
    // break;

    // case MODE_ALIGN_LASER:
    // debug(DEBUG_MAIN,"MODE_ALIGN_LASER");
    // next_mode = MODE_ALIGN_LASER;
    // if (sh.collectLaserAlignmentData())
    // {
    //     sh.alignLaser();
    //     next_mode = MODE_SAVE_LASER_ALIGN;
    // }
    // else
    // {
    //     next_mode = MODE_CALIBRATE_INERTIAL;
    // }
    break;

    case MODE_MENU:
    debug(DEBUG_MAIN,"MODE_MENU");
    // next_mode = MODE_MENU;
    // current_menu++;
    break;

    default:
    break;
    }
}

void b2ShortHold()
{
    // debug(DEBUG_MAIN,"b2ShortHold()");
    switch (current_mode)
    {
        case MODE_LASER_ENA:
        debug(DEBUG_MAIN,"MODE_LASER_ENA");
        // sh.disableLaser();
        break;

        case MODE_MENU:
        debug(DEBUG_MAIN,"MODE_MENU");
        next_mode = MODE_MENU;
        current_menu--;
        break;
    }
}

void b1LongHold()
{
    debug(DEBUG_MAIN,"b1LongHold()");
    switch (current_mode)
    {
        case MODE_SAVE_INERTIAL_CALIB:
        debug(DEBUG_MAIN,"MODE_SAVE_INERTIAL_CALIB");

        // sh.save_inertial_align_data();
        break;

        case MODE_SAVE_LASER_ALIGN:
        debug(DEBUG_MAIN,"MODE_SAVE_INERTIAL_CALIB");
        // sh.save_laser_align_data();
        break;

        case MODE_SAVE_MAG_CALIBRATE:
        debug(DEBUG_MAIN,"MODE_SAVE_MAG_CALIBRATE");
        // mag.save_calibration_data();
        break;

        case MODE_MENU:
        debug(DEBUG_MAIN,"MODE_MENU");
        // menuSelect();
        break;

        default:
        break;
    }
}

void b2LongHold()
{
    // debug(DEBUG_MAIN,"b2LongHold()");
    switch (current_mode)
    {
        case MODE_SAVE_INERTIAL_CALIB:
        debug(DEBUG_MAIN,"MODE_SAVE_INERTIAL_CALIB");
        // sh.resetCalibration();
        break;

        case MODE_SAVE_LASER_ALIGN:
        debug(DEBUG_MAIN,"MODE_SAVE_LASER_ALIGN");
        // sh.resetAlignment();
        break;

        case MODE_SAVE_MAG_CALIBRATE:
        debug(DEBUG_MAIN,"MODE_SAVE_MAG_CALIBRATE");
        // mag.resetCalibration();
        break;

        case MODE_MENU:
        debug(DEBUG_MAIN,"MODE_MENU");
        current_menu = static_cast<menu_enum>(0);
        break;

        default:
        break;
    }
}

void b1b2ShortHold()
{
    // debug(DEBUG_MAIN,"b1b2ShortHold()");
    switch (current_mode)
    {
    case MODE_IDLE:
    debug(DEBUG_MAIN,"MODE_IDLE");
    next_mode = MODE_MENU;
    // Draw menu
    break;
    
    default:
    break;
    }
}

void b1b2LongHold()
{
    // debug(DEBUG_MAIN,"b1b2LongHold()");
    next_mode = MODE_IDLE;
}

void evaluateFSM(uint16_t action)
{
    current_mode = next_mode;
    next_mode = MODE_IDLE;

    switch (action)
    {
    case NOTIFY_B1_SHORT_PRESS:     b1ShortHold();      break;
    case NOTIFY_B2_SHORT_PRESS:     b2ShortHold();      break;
    case NOTIFY_B1_LONG_PRESS:      b1LongHold();       break;
    case NOTIFY_B2_LONG_PRESS:      b2LongHold();       break;
    case NOTIFY_B1B2_SHORT_PRESS:   b1b2ShortHold();    break;
    case NOTIFY_B1B2_LONG_PRESS:    b1b2LongHold();     break;
    default:    break;
    }
}