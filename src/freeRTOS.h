#include <Arduino.h>
#include "fsm.h"
#include "functional_main.h"


TaskHandle_t eventhandler_task;
TaskHandle_t computefunc_task;

#define NOTIFY_B1_PRESSED 0x01
#define NOTIFY_B1_RELEASED 0x02
#define NOTIFY_B2_PRESSED 0x03
#define NOTIFY_B2_RELEASED 0x04


void IRAM_ATTR b1_interrupt()
{
    static bool notification = NOTIFY_B1_PRESSED;
    if (digitalRead(PIN_BUTTON1))
    {
        notification = NOTIFY_B1_RELEASED;
    }

    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR( eventhandler_task,
                            notification,
                            eSetValueWithOverwrite,
                            &xHigherPriorityTaskWoken );
}
void IRAM_ATTR b2_interrupt()
{
    static bool notification = NOTIFY_B2_PRESSED;
    if (digitalRead(PIN_BUTTON2))
    {
        notification = NOTIFY_B2_RELEASED;
    }

    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR( eventhandler_task,
                            notification,
                            eSetValueWithOverwrite,
                            &xHigherPriorityTaskWoken );
}

static uint32_t eventhandlerNotifiedValue;

uint8_t b1Waiting()
{
    // Look for another button interrupt
    static bool timedout = xTaskNotifyWait(0x00, ULONG_MAX, &eventhandlerNotifiedValue, long_hold_time_ms/portTICK_RATE_MS);
    if (timedout)
    {
        debug(DEBUG_MAIN,"NOTIFY_B1_LONG_PRESS");
        return NOTIFY_B1_LONG_PRESS;
    }
    else if (eventhandlerNotifiedValue == NOTIFY_B2_PRESSED)
    {
        debug(DEBUG_MAIN,"b1b2Waiting()");
        return b1b2Waiting();
    } 
    else if (eventhandlerNotifiedValue == NOTIFY_B1_RELEASED)
    {
        debug(DEBUG_MAIN,"NOTIFY_B1_SHORT_PRESS");
        return NOTIFY_B1_SHORT_PRESS;
    }
}

uint8_t b2Waiting()
{
    static bool timedout = xTaskNotifyWait(0x00, ULONG_MAX, &eventhandlerNotifiedValue, long_hold_time_ms/portTICK_RATE_MS);
    if (timedout)
    {
        debug(DEBUG_MAIN,"NOTIFY_B2_LONG_PRESS");
        return NOTIFY_B2_LONG_PRESS;
    }
    else if (eventhandlerNotifiedValue == NOTIFY_B1_PRESSED)
    {
        debug(DEBUG_MAIN,"b1b2Waiting()");
        return b1b2Waiting();
    } 
    else if (eventhandlerNotifiedValue == NOTIFY_B2_RELEASED)
    {
        debug(DEBUG_MAIN,"NOTIFY_B2_SHORT_PRESS");
        return NOTIFY_B2_SHORT_PRESS;
    }
}

uint8_t b1b2Waiting()
{
    static bool timedout = xTaskNotifyWait(0x00, ULONG_MAX, &eventhandlerNotifiedValue, long_hold_time_ms/portTICK_RATE_MS);
    if (timedout)
    {
        debug(DEBUG_MAIN,"NOTIFY_B1B2_LONG_PRESS");
        return NOTIFY_B1B2_LONG_PRESS;
    }
    else if (eventhandlerNotifiedValue == NOTIFY_B1_RELEASED || eventhandlerNotifiedValue == NOTIFY_B2_RELEASED)
    {
        debug(DEBUG_MAIN,"NOTIFY_B1B2_SHORT_PRESS");
        return NOTIFY_B1B2_SHORT_PRESS;
    }
}


void eventhandler(void* parameter)
{
    static uint32_t compute_notification; // Notification to pass to computefunc_task
    while(true)
    {
        xTaskNotifyWait(    0x00,      /* Don't clear any notification bits on entry. */
                            ULONG_MAX, /* Reset the notification value to 0 on exit. */
                            &eventhandlerNotifiedValue, /* Notified value pass out in computefuncNotifiedValue. */
                            portMAX_DELAY );  /* Block indefinitely. */


        static uint8_t action;
        switch (eventhandlerNotifiedValue)
        {
            case NOTIFY_B1_PRESSED:
                action = b1Waiting();
            break;

            case NOTIFY_B2_PRESSED:
                action = b2Waiting();
            break;
        }

        // Notifies computefunc_task with the value "notification", overwriting the current notificaiton value
        xTaskNotify( computefunc_task, action, eSetValueWithOverwrite );

        // Clear any notifications received during running, this prevents chained inputs
        xTaskNotifyStateClear(eventhandler_task);
        ulTaskNotifyValueClear(eventhandler_task,0);
    }
}

void computefunc(void* parameter)
{
    static uint32_t computefuncNotifiedValue;
    while (true)
    {
    /*Bits in this RTOS task's notification value are set by the notifying
    tasks and interrupts to indicate which events have occurred. */
    xTaskNotifyWait(    0x00,      /* Don't clear any notification bits on entry. */
                        ULONG_MAX, /* Reset the notification value to 0 on exit. */
                        &computefuncNotifiedValue, /* Notified value pass out in computefuncNotifiedValue. */
                        portMAX_DELAY );  /* Block indefinitely. */

    /* Process any events that have been latched in the notified value. */
    evaluateFSM(computefuncNotifiedValue);
    }
}

void setup()
{
    xTaskCreatePinnedToCore(
        eventhandler, /* Function to implement the task */
        "main", /* Name of the task */
        100,  /* Stack size in words */
        NULL,  /* Task input parameter */
        2 ,  /* Priority of the task */
        &eventhandler_task,  /* Task handle. */
        0); /* Core where the task should run */

    xTaskCreatePinnedToCore(
        eventhandler, /* Function to implement the task */
        "main", /* Name of the task */
        50000,  /* Stack size in words */
        NULL,  /* Task input parameter */
        1 ,  /* Priority of the task */
        &computefunc_task,  /* Task handle. */
        0); /* Core where the task should run */


}