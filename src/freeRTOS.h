#include <Arduino.h>
#include "fsm.h"
#include "soc/rtc_wdt.h"

TaskHandle_t eventhandler_task;
TaskHandle_t computefunc_task;

#define long_hold_time_ms 2000
#define NOTIFY_B1_PRESSED 0x02
#define NOTIFY_B1_RELEASED 0x03
#define NOTIFY_B2_PRESSED 0x04
#define NOTIFY_B2_RELEASED 0x05

static uint32_t eventhandlerNotifiedValue = 0x00;
static uint32_t computefuncNotifiedValue = 0x00;
static uint32_t compute_notification = 0x00;
static uint32_t event_notification = 0x00;
static uint32_t action = 0x00;

void IRAM_ATTR ISR_b1_interrupt()
{
    
    event_notification = NOTIFY_B1_RELEASED;
    if (digitalRead(PIN_BUTTON1))
    {
        event_notification = NOTIFY_B1_PRESSED;
    }

    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    xTaskNotifyFromISR( eventhandler_task,
                            event_notification,
                            eSetValueWithOverwrite,
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
                            eSetValueWithOverwrite,
                            &xHigherPriorityTaskWoken );
}



static bool timedout;


uint32_t b1b2Waiting()
{
    timedout = !xTaskNotifyWait(0x00, ULONG_MAX, &eventhandlerNotifiedValue, pdMS_TO_TICKS(long_hold_time_ms));
    if (timedout)
    {
        debug(DEBUG_ALWAYS,"NOTIFY_B1B2_LONG_PRESS");
        return NOTIFY_B1B2_LONG_PRESS;
    }
    else if (eventhandlerNotifiedValue == NOTIFY_B1_RELEASED || eventhandlerNotifiedValue == 5000)
    {
        debug(DEBUG_ALWAYS,"NOTIFY_B1B2_SHORT_PRESS");
        return NOTIFY_B1B2_SHORT_PRESS;
    }
    debug(DEBUG_ALWAYS,"---- NO NOTIFICATION FOUND ----");
    return 0;
}

uint32_t b1Waiting()
{
    // Look for another button interrupt
    timedout = !xTaskNotifyWait(0x00, ULONG_MAX, &eventhandlerNotifiedValue, pdMS_TO_TICKS(long_hold_time_ms));
    if (timedout)
    {
        debug(DEBUG_ALWAYS,"NOTIFY_B1_LONG_PRESS");
        return NOTIFY_B1_LONG_PRESS;
    }
    else if (eventhandlerNotifiedValue == NOTIFY_B2_PRESSED)
    {
        debug(DEBUG_ALWAYS,"b1b2Waiting()");
        return b1b2Waiting();
    } 
    else if (eventhandlerNotifiedValue == NOTIFY_B1_RELEASED)
    {
        debug(DEBUG_ALWAYS,"NOTIFY_B1_SHORT_PRESS");
        return NOTIFY_B1_SHORT_PRESS;
    }
    debug(DEBUG_ALWAYS,"---- NO NOTIFICATION FOUND ----");
    return 0;
}

uint32_t b2Waiting()
{
    timedout = !xTaskNotifyWait(0x00, ULONG_MAX, &eventhandlerNotifiedValue, pdMS_TO_TICKS(long_hold_time_ms));
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
    debug(DEBUG_ALWAYS,"---- NO NOTIFICATION FOUND ----");
    return 0;
}

void init_interrupts()
{
    attachInterrupt(PIN_BUTTON1,ISR_b1_interrupt,CHANGE);
    attachInterrupt(PIN_BUTTON2,ISR_b2_interrupt,CHANGE);
}

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

            case NOTIFY_B1_RELEASED:
                debug(DEBUG_ALWAYS,"button1 released");
            break;

            case NOTIFY_B2_RELEASED:
                debug(DEBUG_ALWAYS,"button2 released");
            break;

            default:
                debug(DEBUG_ALWAYS,"Notify with no body!");
            break;
        }

        // // Notifies computefunc_task with the value "notification", overwriting the current notificaiton value
        // xTaskNotify( computefunc_task, action, eSetValueWithOverwrite );

        // // Clear any notifications received during running, this prevents chained inputs
        xTaskNotifyStateClear(eventhandler_task);
        ulTaskNotifyValueClear(eventhandler_task,0);
        Serial << "\n";
    }
}

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
        evaluateFSM(computefuncNotifiedValue);
    }
}

void setup()
{
    xTaskCreatePinnedToCore(
        eventhandler, /* Function to implement the task */
        "eventhandler", /* Name of the task */
        10000,  /* Stack size in words */
        NULL,  /* Task input parameter */
        2 ,  /* Priority of the task */
        &eventhandler_task,  /* Task handle. */
        0); /* Core where the task should run */

    delay(500);
    xTaskCreatePinnedToCore(
        computefunc, /* Function to implement the task */
        "computefunc", /* Name of the task */
        50000,  /* Stack size in words */
        NULL,  /* Task input parameter */
        1 ,  /* Priority of the task */
        &computefunc_task,  /* Task handle. */
        0); /* Core where the task should run */

        init_interrupts();
}

void loop(){}