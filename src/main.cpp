#define DEBUG_MODE
#ifdef DEBUG_MODE
    #include "debug_main.h"
#else
    #include "freeRTOS.h"
#endif