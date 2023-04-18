#ifndef HEADER_UTILITY
#define HEADER_UTILITY

#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
#include <ArduinoEigenSparse.h>
#include "config.h"

using namespace Eigen;

struct node
{
    int id;
    float inclination;
    float heading;
    float distance;
};

// Generate vector from current node back to base
Vector3f generate_vector(float distance, float heading, float inclination);

void debug(unsigned int mode, const char* str);

Vector2d get_inclination_heading(Vector3f true_vec);


#endif