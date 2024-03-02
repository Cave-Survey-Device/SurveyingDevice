#ifndef NUMERICAL_METHODS_LASER_ALIGNMENT_H
#define NUMERICAL_METHODS_LASER_ALIGNMENT_H

#include "utils.h"
#include "FittingFuncs.h"
namespace NumericalMethods{
    
/**
 * @brief Aligns a laser with a set of magnetometer and accelerometer readings
 * 
 * @param g gravitation data
 * @param m magnetic data
 * @param Racc Accelerometer alignment matrix
 * @param Rmag Magnetometer alignment matrix
 */
void alignLaser(const MatrixXf &g, const MatrixXf &m, Matrix3f &Racc, Matrix3f &Rmag);

/**
 * @brief Aligns the normal vector of a point cloud to the principal axis (x-axis) of the device
 * 
 * @param point_cloud Inertial data
 * @param R Correction matrix
 */
void alignToNorm(const Matrix<float,3,N_LASER_CAL> &point_cloud, Matrix3f &R);


}

#endif