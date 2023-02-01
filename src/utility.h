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
    double inclination;
    double heading;
    double distance;
};

// Generate vector from current node back to base
Vector3d generate_vector(double distance, double heading, double inclination);

void debug(unsigned int mode, const char* str);

Vector2d get_inclination_heading(Vector3d true_vec);


#endif