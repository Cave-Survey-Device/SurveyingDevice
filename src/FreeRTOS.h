#include <Arduino.h>
#include "soc/rtc_wdt.h"
#include "utils.h"

TaskHandle_t eventhandler_task;
TaskHandle_t computefunc_task;

#define long_hold_time_ms 5000
#define NOTIFY_B1_PRESSED 0x01
#define NOTIFY_B2_PRESSED 0x02
#define NOTIFY_B3_PRESSED 0x03
#define NOTIFY_B4_PRESSED 0x04

#define NOTIFY_B1_RELEASED 0x05
#define NOTIFY_B2_RELEASED 0x06
#define NOTIFY_B3_RELEASED 0x07
#define NOTIFY_B4_RELEASED 0x08

#define PIN_BUTTON1 1
#define PIN_BUTTON2 1
#define PIN_BUTTON3 1
#define PIN_BUTTON4 1

#define RESTART_NOTIFICATION 0xFFEEFFEE
#define ACTION_B1_SHORT_PRESS 16
#define ACTION_B1_LONG_PRESS 17
#define ACTION_B2_SHORT_PRESS 18
#define ACTION_B2_LONG_PRESS 19
#define ACTION_B3_SHORT_PRESS 20
#define ACTION_B3_LONG_PRESS 21
#define ACTION_B4_SHORT_PRESS 22
#define ACTION_B4_LONG_PRESS 23

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

void IRAM_ATTR ISR_b1_interrupt()
{
    // Set task tag
    event_notification = NOTIFY_B1_RELEASED;
    if (digitalRead(PIN_BUTTON1))
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
    if (digitalRead(PIN_BUTTON2))
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
    event_notification = NOTIFY_B4_RELEASED;
    if (digitalRead(PIN_BUTTON4))
    {
        event_notification = NOTIFY_B2_PRESSED;
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
    if (digitalRead(PIN_BUTTON4))
    {
        event_notification = NOTIFY_B4_PRESSED;
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
    return 0;
}

/**
 * @brief Handler function for b2 pressed
 * 
 * @return uint32_t 
 */
uint32_t b2Waiting()
{
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
    return 0;
}

/**
 * @brief Handler function for b3 pressed
 * 
 * @return uint32_t 
 */
uint32_t b3Waiting()
{
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
    return 0;
}

/**
 * @brief Handler function for b4 pressed
 * 
 * @return uint32_t 
 */
uint32_t b4Waiting()
{
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
    return 0;
}

/**
 * @brief Initialise interrupts
 * 
 */
void init_interrupts()
{
    attachInterrupt(PIN_BUTTON1,ISR_b1_interrupt,CHANGE);
    attachInterrupt(PIN_BUTTON2,ISR_b2_interrupt,CHANGE);
    attachInterrupt(PIN_BUTTON3,ISR_b3_interrupt,CHANGE);
    attachInterrupt(PIN_BUTTON4,ISR_b4_interrupt,CHANGE);
}

extern int begin();
extern int takeShot(); // Collect data for a shot
extern int removeRecentCalib(); // Remove the most recent calibration data point
extern int nextCalib(); // Collect a calibration data point / progress to the next step
extern int historyScrollUp(); // Collect a calibration data point / progress to the next step
extern int historyScrollDown(); // Collect a calibration data point / progress to the next step

void evaluateAction(unsigned int action)
{
    switch (current_mode)
    {
        case SHOT:
            if (action == ACTION_B1_SHORT_PRESS)
            {
                takeShot();
            } else if (action == ACTION_B4_SHORT_PRESS) {
                next_mode = CALIB;
            }
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
    Serial.begin(115200);
    debug(DEBUG_ALWAYS,"start eventhandler");
    while(true)
    {
        
        xTaskNotifyWait(    0x00,      /* Don't clear any notification bits on entry. */
                            ULONG_MAX, /* Reset the notification value to 0 on exit. */
                            &eventhandlerNotifiedValue, /* Notified value pass out in computefuncNotifiedValue. */
                            portMAX_DELAY );  /* Block indefinitely. */
        
        current_timestamp = millis();
        if (current_timestamp > debounce_lockout_ms + previous_debounce_timestamp)
        {
            previous_debounce_timestamp = current_timestamp;
            switch (eventhandlerNotifiedValue)
            {
                case NOTIFY_B1_PRESSED:
                    debug(DEBUG_ALWAYS,"button1 pressed");
                    action = b1Waiting();
                break;

                case NOTIFY_B2_PRESSED:
                    debug(DEBUG_ALWAYS,"button2 pressed");
                    action = b2Waiting();
                break;

                case NOTIFY_B3_PRESSED:
                    debug(DEBUG_ALWAYS,"button3 pressed");
                    action = b3Waiting();
                break;

                case NOTIFY_B4_PRESSED:
                    debug(DEBUG_ALWAYS,"button4 pressed");
                    action = b4Waiting();
                break;

                default:
                    // debug(DEBUG_ALWAYS,"Notify with no body!");
                break;
            }

            // Notifies computefunc_task with the value "notification", overwriting the current notificaiton value
            xTaskNotify( computefunc_task, action, eSetValueWithOverwrite );

            // // Clear any notifications received during running, this prevents chained inputs
            xTaskNotifyStateClear(eventhandler_task);
            ulTaskNotifyValueClear(eventhandler_task,0);
            eventhandlerNotifiedValue = 0x00;
            Serial << "\n";
        }
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
        evaluateAction(computefuncNotifiedValue);
    }
}

