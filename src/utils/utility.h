#ifndef HEADER_UTILITY
#define HEADER_UTILITY

#include <ArduinoEigenDense.h>

#include "config.h"

using namespace Eigen;

struct node
{
    int id;
    float inclination;
    float heading;
    float distance;
};

template<class T> inline Print &operator <<(Print &obj, T arg)  // no-cost stream operator as described at http://arduiniana.org/libraries/streaming/
{
    obj.print(arg);
    return obj;
}

// Generate vector from current node back to base
Vector3f generate_vector(float distance, float heading, float inclination);

void debug(unsigned int mode, const char* str);
void debugf(unsigned int mode, const char *format, ...);

Vector3f Cartesian(Vector3f spherical);

Vector3f Spherical(Vector3f cartesian);

// heading, inclination, roll
Vector3f Orientation(Vector3f g, Vector3f m);

#endif