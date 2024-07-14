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
TaskHandle_t displayhandler_task;

#define long_hold_time_ms 2000

#define NOTIFY_NONE (uint32_t)0
#define NOTIFY_B1   (uint32_t)1
#define NOTIFY_B2   (uint32_t)2
#define NOTIFY_B3   (uint32_t)3
#define NOTIFY_B4   (uint32_t)4
#define NOTIFY_B5   (uint32_t)5

// #define NOTIFY_NONE       (uint32_t)0
// #define NOTIFY_B1_PRESSED (uint32_t)1
// #define NOTIFY_B2_PRESSED (uint32_t)2
// #define NOTIFY_B3_PRESSED (uint32_t)3
// #define NOTIFY_B4_PRESSED (uint32_t)4
// #define NOTIFY_B5_PRESSED (uint32_t)5

// #define NOTIFY_B1_RELEASED (uint32_t)11
// #define NOTIFY_B2_RELEASED (uint32_t)12
// #define NOTIFY_B3_RELEASED (uint32_t)13
// #define NOTIFY_B4_RELEASED (uint32_t)14
// #define NOTIFY_B5_RELEASED (uint32_t)15

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
static uint32_t displayhandlerNotifiedValue = 0x00;

static uint32_t event_notification = 0x00;
static uint32_t compute_notification = 0x00;

static uint32_t action = 0x00;
static uint32_t wait_id = 0x00;
static bool timedout;

static unsigned int current_timestamp;
static unsigned int previous_debounce_timestamp;
static unsigned int debounce_lockout_ms = 10;

enum DisplayModeEnum
{
    DISP_CALIB,
    DISP_LOADING,
    DISP_ORIENTATION,
    DISP_CALIB_SAVE_YN,
    DISP_CALIB_REM_YN
};

enum ModeEnum
{
    MODE_LASER_OFF,
    MODE_LASER_ON,
    MODE_SHOT,
    MODE_CALIB,
    MODE_CALIB_REM_YN,
    MODE_CALIB_SAVE_YN,
    MODE_HISTORY,
    MODE_BLUETOOTH,
    MODE_FILES,
    MODE_CONFIG,
};
static ModeEnum current_mode;
static ModeEnum next_mode;
static DisplayModeEnum display_mode;

void initFreeRTOS(){
    current_mode = MODE_LASER_OFF;
    next_mode = MODE_LASER_OFF;
}


void IRAM_ATTR ISR_b1_interrupt()
{

    event_notification = NOTIFY_B1;
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
    event_notification = NOTIFY_B2;
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR( eventhandler_task,
                        event_notification,
                        eSetValueWithoutOverwrite,
                        &xHigherPriorityTaskWoken );
}

void IRAM_ATTR ISR_b3_interrupt()
{
    event_notification = NOTIFY_B3;
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR( eventhandler_task,
                        event_notification,
                        eSetValueWithoutOverwrite,
                        &xHigherPriorityTaskWoken );
}

void IRAM_ATTR ISR_b4_interrupt()
{
    event_notification = NOTIFY_B4;
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR( eventhandler_task,
                        event_notification,
                        eSetValueWithoutOverwrite,
                        &xHigherPriorityTaskWoken );
}

void IRAM_ATTR ISR_b5_interrupt()
{
    event_notification = NOTIFY_B5;
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR( eventhandler_task,
                        event_notification,
                        eSetValueWithoutOverwrite,
                        &xHigherPriorityTaskWoken );
}

void disableInterrupts()
{
    detachInterrupt(PIN_BUTTON1);
    detachInterrupt(PIN_BUTTON2);
    detachInterrupt(PIN_BUTTON3);
    detachInterrupt(PIN_BUTTON4);
    detachInterrupt(PIN_BUTTON5);
}
void enableInterrupts()
{
    disableInterrupts();
    attachInterrupt(PIN_BUTTON1,ISR_b1_interrupt,CHANGE);
    attachInterrupt(PIN_BUTTON2,ISR_b2_interrupt,CHANGE);
    attachInterrupt(PIN_BUTTON3,ISR_b3_interrupt,CHANGE);
    attachInterrupt(PIN_BUTTON4,ISR_b4_interrupt,CHANGE);
    attachInterrupt(PIN_BUTTON5,ISR_b5_interrupt,CHANGE);
}
void enableRisingInterrupts()
{
    disableInterrupts();
    attachInterrupt(PIN_BUTTON1,ISR_b1_interrupt,RISING);
    attachInterrupt(PIN_BUTTON2,ISR_b2_interrupt,RISING);
    attachInterrupt(PIN_BUTTON3,ISR_b3_interrupt,RISING);
    attachInterrupt(PIN_BUTTON4,ISR_b4_interrupt,RISING);
    attachInterrupt(PIN_BUTTON5,ISR_b5_interrupt,RISING);
}
void enableFallingInterrupts()
{
    disableInterrupts();
    attachInterrupt(PIN_BUTTON1,ISR_b1_interrupt,FALLING);
    attachInterrupt(PIN_BUTTON2,ISR_b2_interrupt,FALLING);
    attachInterrupt(PIN_BUTTON3,ISR_b3_interrupt,FALLING);
    attachInterrupt(PIN_BUTTON4,ISR_b4_interrupt,FALLING);
    attachInterrupt(PIN_BUTTON5,ISR_b5_interrupt,FALLING);
}

uint32_t buttonWaiting()
{
    debug(DEBUG_ALWAYS,"buttonWaiting");
    // Look for another button interrupt
    timedout = !xTaskNotifyWait(ULONG_MAX, ULONG_MAX, &eventhandlerNotifiedValue, pdMS_TO_TICKS(long_hold_time_ms));
    Serial.printf("Notification received: %" PRIu32 "\n",eventhandlerNotifiedValue);

    if (timedout)
    {
        debug(DEBUG_ALWAYS,"Long Press");
        switch (wait_id)
        {
            case NOTIFY_B1:
            debug(DEBUG_ALWAYS,"ACTION_B1_LONG_PRESS");
            return ACTION_B1_LONG_PRESS;
            break;

            case NOTIFY_B2:
            debug(DEBUG_ALWAYS,"ACTION_B2_LONG_PRESS");
            return ACTION_B2_LONG_PRESS;
            break;

            case NOTIFY_B3:
            debug(DEBUG_ALWAYS,"ACTION_B3_LONG_PRESS");
            return ACTION_B3_LONG_PRESS;
            break;

            case NOTIFY_B4:
            debug(DEBUG_ALWAYS,"ACTION_B4_LONG_PRESS");
            return ACTION_B4_LONG_PRESS;
            break;
        }
    }
    else
    {
        debug(DEBUG_ALWAYS,"Short press");
        switch (wait_id)
        {
            case NOTIFY_B1:
            debug(DEBUG_ALWAYS,"ACTION_B1_SHORT_PRESS");
            return ACTION_B1_SHORT_PRESS;
            break;

            case NOTIFY_B2:
            debug(DEBUG_ALWAYS,"ACTION_B2_SHORT_PRESS");
            return ACTION_B2_SHORT_PRESS;
            break;

            case NOTIFY_B3:
            debug(DEBUG_ALWAYS,"ACTION_B3_SHORT_PRESS");
            return ACTION_B3_SHORT_PRESS;
            break;

            case NOTIFY_B4:
            debug(DEBUG_ALWAYS,"ACTION_B4_SHORT_PRESS");
            return ACTION_B4_SHORT_PRESS;
            break;
        }
    }
    debug(DEBUG_ALWAYS,"---- NO NOTIFICATION FOUND ----");
    return ACTION_NONE;
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
extern int saveCalib(); // Saves the most recent calibration
extern int nextCalib(); // Collect a calibration data point / progress to the next step
extern int historyScrollUp(); // Collect a calibration data point / progress to the next step
extern int historyScrollDown(); // Collect a calibration data point / progress to the next step
extern int laserOn(); // Collect a calibration data point / progress to the next step
extern int laserOff(); // Collect a calibration data point / progress to the next step

extern int displayCalib();
extern int displayRemCalibYN(bool YN);
extern int displaySaveCalibYN(bool YN);
extern int displayOrientation();
extern int displayRecentShot();
extern int displayLoading();
bool yes_or_no = true;

void displayAction()
{
    switch(display_mode)
    {
        case DISP_ORIENTATION:
        displayOrientation();
        break;

        case DISP_CALIB:
        displayCalib();
        break;

        case DISP_LOADING:
        displayLoading();
        break;
    }
}

void evaluateAction(const uint32_t action)
{
    switch (current_mode)
    {
        case MODE_LASER_OFF:
        if (action == ACTION_B1_LONG_PRESS)
        {
            next_mode = MODE_LASER_ON;
            display_mode = DISP_ORIENTATION;
            laserOn();
        } else if (action == ACTION_B4_SHORT_PRESS) {
            next_mode = MODE_CALIB;
            display_mode = DISP_CALIB;
            laserOff();
        }
        break;

        case MODE_LASER_ON:
        if (action == ACTION_B1_SHORT_PRESS)
        {
            next_mode = MODE_LASER_ON;
            display_mode = DISP_ORIENTATION;
            takeShot();
        } else if (action == ACTION_B1_LONG_PRESS) {
            next_mode = MODE_LASER_OFF;
            display_mode = DISP_ORIENTATION;
            laserOff();
        } else if (action == ACTION_B4_SHORT_PRESS) {
            next_mode = MODE_CALIB;
            display_mode = DISP_CALIB;
            laserOff();
        }
        break;

        case MODE_CALIB:
        if (action == ACTION_B1_SHORT_PRESS) {
            display_mode = DISP_LOADING;
            xTaskNotify(displayhandler_task,0x00,eSetValueWithOverwrite); // Update display
            taskYIELD();
            
            if (nextCalib() == N_ORIENTATIONS + N_LASER_CAL)
            {
                next_mode = MODE_CALIB_SAVE_YN;
                display_mode = DISP_CALIB_SAVE_YN;
                yes_or_no = true;
                
            } else {
                next_mode = MODE_CALIB;
                display_mode = DISP_CALIB;
            }

        } else if (action == ACTION_B3_SHORT_PRESS) {
            next_mode = MODE_CALIB_REM_YN;
            display_mode = DISP_CALIB_REM_YN;
            yes_or_no = true;

        } else if (action == ACTION_B4_SHORT_PRESS) {
            next_mode = MODE_LASER_OFF;
            display_mode = DISP_ORIENTATION;
        }
        break;

        case MODE_CALIB_SAVE_YN:
        if (action == ACTION_B1_SHORT_PRESS) {
            if (yes_or_no) {    saveCalib();    }
            next_mode = MODE_CALIB;
            display_mode = DISP_CALIB;

        } else if (action == ACTION_B2_SHORT_PRESS) {
            next_mode = MODE_CALIB_SAVE_YN;
            display_mode = DISP_CALIB_SAVE_YN;
            yes_or_no = true;

        } else if (action == ACTION_B3_SHORT_PRESS) {
            next_mode = MODE_CALIB_SAVE_YN;
            display_mode = DISP_CALIB_SAVE_YN;
            yes_or_no = false;
        }
        break;

        case MODE_CALIB_REM_YN:
        if (action == ACTION_B1_SHORT_PRESS) {
            if (yes_or_no) {    removeRecentCalib();    }
            next_mode = MODE_CALIB;
            display_mode = DISP_CALIB;
        } else if (action == ACTION_B2_SHORT_PRESS) {
            next_mode = MODE_CALIB_REM_YN;
            display_mode = DISP_CALIB_REM_YN;
            yes_or_no = true;

        } else if (action == ACTION_B3_SHORT_PRESS) {
            next_mode = MODE_CALIB_REM_YN;
            display_mode = DISP_CALIB_REM_YN;
            yes_or_no = false;
        }
        break;

        // case HISTORY:
        // if (action == ACTION_B2_SHORT_PRESS) {
        //     historyScrollUp();
        // } else if (action == ACTION_B3_SHORT_PRESS) {
        //     historyScrollDown();
        // } else if (action == ACTION_B4_SHORT_PRESS) {
        // }
        // break;

        // case BLUETOOTH:
        // // Requires button 5
        // break;

        // case FILES:
        // break;

        // case CONFIG:
        // break;
    }
    xTaskNotify(displayhandler_task,0x00,eSetValueWithOverwrite); // Update display
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
        vTaskDelay(pdMS_TO_TICKS(50)); // Delay 10ms to allow for bounce
        enableFallingInterrupts(); // Enable button press interrupts
        debug(DEBUG_ALWAYS,"Eventhandler: Waiting for notify...\n");
        xTaskNotifyWait(    0x00,      /* Don't clear any notification bits on entry. */
                            ULONG_MAX, /* Reset the notification value to 0 on exit. */
                            &eventhandlerNotifiedValue, /* Notified value pass out in computefuncNotifiedValue. */
                            portMAX_DELAY );  /* Block indefinitely. */
        
        debug(DEBUG_ALWAYS,"Received interrupt...");

        // Clear data...
        received_command = false;
        action = ACTION_NONE;
        wait_id = eventhandlerNotifiedValue;

        // Disable interrupts such that no new actions are processed until current task complete
        disableInterrupts();
        switch (eventhandlerNotifiedValue)
        {
            case NOTIFY_B1:
                debug(DEBUG_ALWAYS,"button1 pressed");
                attachInterrupt(PIN_BUTTON1,ISR_b1_interrupt,RISING);
                action = buttonWaiting();                
                received_command = true;
            break;

            case NOTIFY_B2:
                debug(DEBUG_ALWAYS,"button2 pressed");
                attachInterrupt(PIN_BUTTON2,ISR_b2_interrupt,RISING);
                action = buttonWaiting();
                received_command = true;
            break;

            case NOTIFY_B3:
                debug(DEBUG_ALWAYS,"button3 pressed");
                attachInterrupt(PIN_BUTTON3,ISR_b3_interrupt,RISING);
                action = buttonWaiting();
                received_command = true;
            break;

            case NOTIFY_B4:
                debug(DEBUG_ALWAYS,"button4 pressed");
                attachInterrupt(PIN_BUTTON4,ISR_b4_interrupt,RISING);
                action = buttonWaiting();
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

        // If a valid command is received
        if ((received_command) & (action != 0) & (action != ACTION_NONE))
        {
            debug(DEBUG_ALWAYS,"Command received!");

            // Notifies computefunc_task with the value "notification", overwriting the current notificaiton value
            disableInterrupts();
            xTaskNotify( computefunc_task, action, eSetValueWithOverwrite );
            
            // Await notification from compute function before proceeding
            disableInterrupts();
            xTaskNotifyWait(    0x00,      /* Don't clear any notification bits on entry. */
                                ULONG_MAX, /* Reset the notification value to 0 on exit. */
                                &eventhandlerNotifiedValue, /* Notified value pass out in computefuncNotifiedValue. */
                                portMAX_DELAY );  /* Block indefinitely. */
        }

        // Clear any notifications received during running, this prevents chained inputs
        xTaskNotifyStateClear(eventhandler_task);
        ulTaskNotifyValueClear(eventhandler_task,0);
        eventhandlerNotifiedValue = 0x00;


    }
}

void displayhandler(void* parameter)
{
    Debug::debug(Debug::DEBUG_ALWAYS,"Start passivehandler");
    static const TickType_t xFrequency = pdMS_TO_TICKS(250);
    while(true)
    {
        xTaskNotifyWait(    0x00,      /* Don't clear any notification bits on entry. */
                            ULONG_MAX, /* Reset the notification value to 0 on exit. */
                            &displayhandlerNotifiedValue, /* Notified value pass out in computefuncNotifiedValue. */
                            xFrequency );  /* Block indefinitely. */
        
        displayAction();
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

        // Notify eventhandler that task has finished executing and to re-enable inputs
        xTaskNotify(eventhandler_task,0x00,eSetValueWithOverwrite);
    }
}

#endif