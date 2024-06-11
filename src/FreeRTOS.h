#ifndef HEADER_FREERTOS
#define HEADER_FREERTOS

#include <Arduino.h>
#include "soc/rtc_wdt.h"
#include "utils.h"
#include <debug.h>
#include <inttypes.h>



using namespace Debug;

TaskHandle_t eventhandler_task;
TaskHandle_t computefunc_task;

#define long_hold_time_ms 2000
#define NOTIFY_NONE       (uint32_t)0x0000
#define NOTIFY_B1_PRESSED (uint32_t)0x0001
#define NOTIFY_B2_PRESSED (uint32_t)0x0002
#define NOTIFY_B3_PRESSED (uint32_t)0x0003
#define NOTIFY_B4_PRESSED (uint32_t)0x0004
#define NOTIFY_B5_PRESSED (uint32_t)0x0005

#define NOTIFY_B1_RELEASED (uint32_t)0x0011
#define NOTIFY_B2_RELEASED (uint32_t)0x0012
#define NOTIFY_B3_RELEASED (uint32_t)0x0013
#define NOTIFY_B4_RELEASED (uint32_t)0x0014
#define NOTIFY_B5_RELEASED (uint32_t)0x0015

// Centre
#define PIN_BUTTON1 25
// Left
#define PIN_BUTTON2 36
// Right
#define PIN_BUTTON3 4
// Up
#define PIN_BUTTON4 34
// Down
#define PIN_BUTTON5 39

#define RESTART_NOTIFICATION 0xFFEEFFEE
#define ACTION_NONE             (uint32_t)0x0100
#define ACTION_B1_SHORT_PRESS   (uint32_t)0x0101
#define ACTION_B1_LONG_PRESS    (uint32_t)0x0102
#define ACTION_B2_SHORT_PRESS   (uint32_t)0x0103
#define ACTION_B2_LONG_PRESS    (uint32_t)0x0104
#define ACTION_B3_SHORT_PRESS   (uint32_t)0x0105
#define ACTION_B3_LONG_PRESS    (uint32_t)0x0106
#define ACTION_B4_SHORT_PRESS   (uint32_t)0x0107
#define ACTION_B4_LONG_PRESS    (uint32_t)0x0108

static uint32_t eventhandlerNotifiedValue = 0x00;
static uint32_t computefuncNotifiedValue = 0x00;
static uint32_t compute_notification = 0x00;
static uint32_t event_notification = 0x00;
static uint32_t action = 0x00;
static bool timedout;

static unsigned int current_timestamp;
static unsigned int previous_debounce_timestamp;
static unsigned int debounce_lockout_ms = 10;

enum ModeEnum
{
    LASER_OFF,
    LASER_ON,
    SHOT,
    CALIB,
    CALIB_REM_YN,
    CALIB_SAVE_YN,
    HISTORY,
    BLUETOOTH,
    FILES,
    CONFIG
};
static ModeEnum current_mode;
static ModeEnum next_mode;

void initFreeRTOS(){
    current_mode = LASER_OFF;
    next_mode = LASER_OFF;
}

void IRAM_ATTR ISR_b1_interrupt()
{
    // Set task tag
    event_notification = NOTIFY_B1_RELEASED;
    if (!digitalRead(PIN_BUTTON1))
    {
        event_notification = NOTIFY_B1_PRESSED;
    }

    // Defines whether task has a higher priority task
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Sends notify to the event handler
    xTaskNotifyFromISR( eventhandler_task,
                        event_notification,
                        eSetValueWithoutOverwrite,
                        &xHigherPriorityTaskWoken );
}

void IRAM_ATTR ISR_b2_interrupt()
{
    event_notification = NOTIFY_B2_RELEASED;
    if (!digitalRead(PIN_BUTTON2))
    {
        event_notification = NOTIFY_B2_PRESSED;
    }

    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR( eventhandler_task,
                        event_notification,
                        eSetValueWithoutOverwrite,
                        &xHigherPriorityTaskWoken );
}

void IRAM_ATTR ISR_b3_interrupt()
{
    event_notification = NOTIFY_B3_RELEASED;
    if (!digitalRead(PIN_BUTTON3))
    {
        event_notification = NOTIFY_B3_PRESSED;
    }

    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR( eventhandler_task,
                        event_notification,
                        eSetValueWithoutOverwrite,
                        &xHigherPriorityTaskWoken );
}

void IRAM_ATTR ISR_b4_interrupt()
{
    event_notification = NOTIFY_B4_RELEASED;
    if (!digitalRead(PIN_BUTTON4))
    {
        event_notification = NOTIFY_B4_PRESSED;
    }

    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR( eventhandler_task,
                        event_notification,
                        eSetValueWithoutOverwrite,
                        &xHigherPriorityTaskWoken );
}

void IRAM_ATTR ISR_b5_interrupt()
{
    event_notification = NOTIFY_B5_RELEASED;
    if (!digitalRead(PIN_BUTTON5))
    {
        event_notification = NOTIFY_B5_PRESSED;
    }

    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR( eventhandler_task,
                        event_notification,
                        eSetValueWithoutOverwrite,
                        &xHigherPriorityTaskWoken );
}

/**
 * @brief Handler function for b1 pressed
 * 
 * @return uint32_t 
 */
uint32_t b1Waiting()
{
    // Look for another button interrupt
    debug(DEBUG_ALWAYS,"b1Waiting");
    timedout = !xTaskNotifyWait(0x00, ULONG_MAX, &eventhandlerNotifiedValue, pdMS_TO_TICKS(long_hold_time_ms));
    if (timedout)
    {
        debug(DEBUG_ALWAYS,"NOTIFY_B1_LONG_PRESS");
        return ACTION_B1_LONG_PRESS;
    }
    else if (eventhandlerNotifiedValue == NOTIFY_B1_RELEASED)
    {
        debug(DEBUG_ALWAYS,"ACTION_B1_SHORT_PRESS");
        return ACTION_B1_SHORT_PRESS;
    }
    debug(DEBUG_ALWAYS,"---- NO NOTIFICATION FOUND ----");
    return ACTION_NONE;
}

/**
 * @brief Handler function for b2 pressed
 * 
 * @return uint32_t 
 */
uint32_t b2Waiting()
{
    debug(DEBUG_ALWAYS,"b2Waiting");
    // Look for another button interrupt
    timedout = !xTaskNotifyWait(0x00, ULONG_MAX, &eventhandlerNotifiedValue, pdMS_TO_TICKS(long_hold_time_ms));
    if (timedout)
    {
        debug(DEBUG_ALWAYS,"ACTION_B2_LONG_PRESS");
        return ACTION_B2_LONG_PRESS;
    }
    else if (eventhandlerNotifiedValue == NOTIFY_B2_RELEASED)
    {
        debug(DEBUG_ALWAYS,"ACTION_B2_SHORT_PRESS");
        return ACTION_B2_SHORT_PRESS;
    }
    debug(DEBUG_ALWAYS,"---- NO NOTIFICATION FOUND ----");
    return ACTION_NONE;
}

/**
 * @brief Handler function for b3 pressed
 * 
 * @return uint32_t 
 */
uint32_t b3Waiting()
{
    debug(DEBUG_ALWAYS,"b3Waiting");
    // Look for another button interrupt
    timedout = !xTaskNotifyWait(0x00, ULONG_MAX, &eventhandlerNotifiedValue, pdMS_TO_TICKS(long_hold_time_ms));
    if (timedout)
    {
        debug(DEBUG_ALWAYS,"ACTION_B3_LONG_PRESS");
        return ACTION_B3_LONG_PRESS;
    }
    else if (eventhandlerNotifiedValue == NOTIFY_B3_RELEASED)
    {
        debug(DEBUG_ALWAYS,"ACTION_B3_SHORT_PRESS");
        return ACTION_B3_SHORT_PRESS;
    }
    debug(DEBUG_ALWAYS,"---- NO NOTIFICATION FOUND ----");
    return ACTION_NONE;
}

/**
 * @brief Handler function for b4 pressed
 * 
 * @return uint32_t 
 */
uint32_t b4Waiting()
{
    debug(DEBUG_ALWAYS,"b4Waiting");
    // Look for another button interrupt
    timedout = !xTaskNotifyWait(0x00, ULONG_MAX, &eventhandlerNotifiedValue, pdMS_TO_TICKS(long_hold_time_ms));
    if (timedout)
    {
        debug(DEBUG_ALWAYS,"ACTION_B4_LONG_PRESS");
        return ACTION_B4_LONG_PRESS;
    }
    else if (eventhandlerNotifiedValue == NOTIFY_B1_RELEASED)
    {
        debug(DEBUG_ALWAYS,"ACTION_B4_SHORT_PRESS");
        return ACTION_B4_SHORT_PRESS;
    }
    debug(DEBUG_ALWAYS,"---- NO NOTIFICATION FOUND ----");
    return ACTION_NONE;
}

void enableInterrupts()
{
    attachInterrupt(PIN_BUTTON1,ISR_b1_interrupt,CHANGE);
    attachInterrupt(PIN_BUTTON2,ISR_b2_interrupt,CHANGE);
    attachInterrupt(PIN_BUTTON3,ISR_b3_interrupt,CHANGE);
    attachInterrupt(PIN_BUTTON4,ISR_b4_interrupt,CHANGE);
    attachInterrupt(PIN_BUTTON5,ISR_b5_interrupt,CHANGE);
}
void disableInterrupts()
{
    attachInterrupt(PIN_BUTTON1,ISR_b1_interrupt,CHANGE);
    attachInterrupt(PIN_BUTTON2,ISR_b2_interrupt,CHANGE);
    attachInterrupt(PIN_BUTTON3,ISR_b3_interrupt,CHANGE);
    attachInterrupt(PIN_BUTTON4,ISR_b4_interrupt,CHANGE);
    attachInterrupt(PIN_BUTTON5,ISR_b5_interrupt,CHANGE);
}

/**
 * @brief Initialise interrupts
 * 
 */
void init_interrupts()
{
    pinMode(PIN_BUTTON1,INPUT_PULLUP);
    pinMode(PIN_BUTTON2,INPUT_PULLUP);
    pinMode(PIN_BUTTON3,INPUT_PULLUP);
    pinMode(PIN_BUTTON4,INPUT_PULLUP);
    pinMode(PIN_BUTTON5,INPUT_PULLUP);

    enableInterrupts();
}


extern int begin();
extern int takeShot(); // Collect data for a shot
extern int removeRecentCalib(); // Remove the most recent calibration data point
extern int nextCalib(); // Collect a calibration data point / progress to the next step
extern int historyScrollUp(); // Collect a calibration data point / progress to the next step
extern int historyScrollDown(); // Collect a calibration data point / progress to the next step
extern int laserOn(); // Collect a calibration data point / progress to the next step
extern int laserOff(); // Collect a calibration data point / progress to the next step

extern int displayCalib();
extern int displayRemCalibYN();
extern int displaySaveCalibYN();
extern int displayOrientation();
extern int displayRecentShot();

void passiveAction(ModeEnum mode)
{
    switch(mode)
    {
        case LASER_OFF:
        displayOrientation();
        break;

        case LASER_ON:
        displayOrientation();
        break;

        case SHOT:
        displayRecentShot();
        break;

        case CALIB:
        displayCalib();
        break;

        case CALIB_REM_YN:
        displayRemCalibYN();
        break;

        case CALIB_SAVE_YN:
        displaySaveCalibYN();
        break;

        case HISTORY:
        displaySaveCalibYN();
        break;

        case BLUETOOTH:
        break;

        case FILES:
        break;

        case CONFIG:
        break;
    }
}

void evaluateAction(const uint32_t action)
{
    switch (current_mode)
    {
        case LASER_OFF:
        if (action == ACTION_B1_LONG_PRESS)
        {
            debug(DEBUG_ALWAYS,"Turn laser on...");
            laserOn();
            next_mode = LASER_ON;
        } else if (action == ACTION_B4_SHORT_PRESS) {
            debug(DEBUG_ALWAYS,"Enter calib mode...");
            laserOff();
            next_mode = CALIB;
        }
        break;

        case LASER_ON:
        if (action == ACTION_B1_SHORT_PRESS)
        {
            debug(DEBUG_ALWAYS,"Take Shot...");
            takeShot();
        } else if (action == ACTION_B1_LONG_PRESS) {
            debug(DEBUG_ALWAYS,"Turn laser off...");
            laserOff();
            next_mode = LASER_OFF;
        } else if (action == ACTION_B4_SHORT_PRESS) {
            debug(DEBUG_ALWAYS,"Enter calib mode...");
            laserOff();
            next_mode = CALIB;
        }
        break;

        case SHOT:

        break;

        case CALIB:
        if (action == ACTION_B1_SHORT_PRESS) {
            if (nextCalib() == 1)
            {
                next_mode = CALIB_SAVE_YN;
            };
        } else if (action == ACTION_B3_SHORT_PRESS) {
            next_mode = CALIB_REM_YN;
        } else if (action == ACTION_B4_SHORT_PRESS) {
            next_mode = HISTORY;
        }
        break;

        case CALIB_REM_YN:
        if (action == ACTION_B2_SHORT_PRESS) {
            removeRecentCalib();
            next_mode = CALIB;
        } else if (action == ACTION_B3_SHORT_PRESS) {
            next_mode = CALIB;
        }
        break;

        case HISTORY:
        if (action == ACTION_B2_SHORT_PRESS) {
            historyScrollUp();
        } else if (action == ACTION_B3_SHORT_PRESS) {
            historyScrollDown();
        } else if (action == ACTION_B4_SHORT_PRESS) {
                next_mode = BLUETOOTH;
        }
        break;

        case BLUETOOTH:
        // Requires button 5
        break;

        case FILES:
        break;

        case CONFIG:
        break;
    }
}


/**
 * @brief Handler function for all external inputs
 * Using FreeRTOS, wait until a npotification is received. After receiving a notification,
 * enter b1 waiting, b2 waiting.
 * 
 * After executing the desired waiting function, notify computeFunc with a desired action. Fincally clear the notification queue and values.
 * 
 * @param parameter 
 */
void eventhandler(void* parameter)
{
    Debug::debug(Debug::DEBUG_ALWAYS,"Start eventhandler");
    static bool received_command;
    while(true)
    {
        debug(DEBUG_ALWAYS,"Eventhandler: Waiting for notify...\n");
        xTaskNotifyWait(    0x00,      /* Don't clear any notification bits on entry. */
                            ULONG_MAX, /* Reset the notification value to 0 on exit. */
                            &eventhandlerNotifiedValue, /* Notified value pass out in computefuncNotifiedValue. */
                            portMAX_DELAY );  /* Block indefinitely. */
        
        debug(DEBUG_ALWAYS,"Received interrupt...");
        received_command = false;
        action = ACTION_NONE;
        switch (eventhandlerNotifiedValue)
        {
            case NOTIFY_B1_PRESSED:
                debug(DEBUG_ALWAYS,"button1 pressed");
                action = b1Waiting();
                received_command = true;
            break;

            case NOTIFY_B2_PRESSED:
                debug(DEBUG_ALWAYS,"button2 pressed");
                action = b2Waiting();
                received_command = true;
            break;

            case NOTIFY_B3_PRESSED:
                debug(DEBUG_ALWAYS,"button3 pressed");
                action = b3Waiting();
                received_command = true;
            break;

            case NOTIFY_B4_PRESSED:
                debug(DEBUG_ALWAYS,"button4 pressed");
                action = b4Waiting();
                received_command = true;
            break;

            case NOTIFY_NONE:
                received_command = false;
                debug(DEBUG_ALWAYS,"Notify with no body!");
            break;

            default:
                received_command = false;
                debug(DEBUG_ALWAYS,"Notify with no body!");
            break;
        }

        if ((received_command) & (action != 0) & (action != ACTION_NONE))
        {
            debug(DEBUG_ALWAYS,"Command received!");
            // Notifies computefunc_task with the value "notification", overwriting the current notificaiton value
            xTaskNotify( computefunc_task, action, eSetValueWithOverwrite );
        }
        // Clear any notifications received during running, this prevents chained inputs
        xTaskNotifyStateClear(eventhandler_task);
        ulTaskNotifyValueClear(eventhandler_task,0);
        eventhandlerNotifiedValue = 0x00;
    }
}


/**
 * @brief Handler for executing complex tasts. This handler responds to reuqested actions from eventandler.
 * 
 * @param parameter 
 */
void computefunc(void* parameter)
{
    debug(DEBUG_ALWAYS,"start computefunc");
    while (true)
    {
        /*Bits in this RTOS task's notification value are set by the notifying
        tasks and interrupts to indicate which events have occurred. */
        xTaskNotifyWait(    0x00,      /* Don't clear any notification bits on entry. */
                            ULONG_MAX, /* Reset the notification value to 0 on exit. */
                            &computefuncNotifiedValue, /* Notified value pass out in computefuncNotifiedValue. */
                            portMAX_DELAY );  /* Block indefinitely. */

        /* Process any events that have been latched in the notified value. */
        uint32_t val = computefuncNotifiedValue;
        debug(DEBUG_ALWAYS,"Compute task woken up...");
        evaluateAction(val);
        current_mode = next_mode;
    }
}

#endif