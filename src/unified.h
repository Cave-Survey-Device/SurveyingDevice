#ifndef HEADER_UNIFIED
#define HEADER_UNIFIED

#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
#include <ArduinoEigenSparse.h>

using namespace Eigen;

Vector3d generate_vector(double distance, double heading, double inclination);

#endif