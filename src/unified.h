#ifndef HEADER_UNIFIED
#define HEADER_UNIFIED

#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
#include <ArduinoEigenSparse.h>
#include "config.h"

using namespace Eigen;

struct node
{
    unsigned int id;
    unsigned int previous;
    Vector3d vector_to_prev;
};

// Generate vector from current node back to base
Vector3d generate_vector(double distance, double heading, double inclination);

void debug(unsigned int mode, const char* str);

#endif