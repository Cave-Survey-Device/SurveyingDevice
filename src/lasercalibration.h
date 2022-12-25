#ifndef HEADER_lasercalibration
#define HEADER_lasercalibration

#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
#include <ArduinoEigenSparse.h>
#include <iostream>
#include "config.h"


using namespace Eigen;
using std::cout;

// Calculates the best fit plane to a set of points in 3d adn returns the vector normal to this plane
Vector3d calc_SVD(MatrixXd g_vec, bool debug = false);

// Calculates the actual vector of the ToF sensor, given a normal vector, set of measured distances, and length of disto from fornt to back.
Matrix<double,3,1> calc_true_vec(Vector3d normal_vec, VectorXd laser_distances, bool debug = false);

#endif