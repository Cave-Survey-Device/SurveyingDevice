#include "debug.h"

namespace Debug
{

void debug(unsigned int mode, const char* str)
{
    if ((int)mode == 0 || (DEBUG_BOOL_ARR[(int)mode] && sizeof(str) < 250*sizeof(char)))
    {
        char buffer[250+6];
        sprintf(buffer, "%s: %s\n",DEBUG_STR_ARR[(int)mode],str);
        Serial.print(buffer);
    }
}

void debugf(unsigned int mode, const char *format, ...)
{
    if ((int)mode == 0 || (DEBUG_BOOL_ARR[(int)mode] && sizeof(format) < 250*sizeof(char)))
    {
        va_list args;
        Serial.printf("%s: ", DEBUG_STR_ARR[(int)mode]);
        Serial.printf(format, args);
        Serial.print("\n");

    }
}

}