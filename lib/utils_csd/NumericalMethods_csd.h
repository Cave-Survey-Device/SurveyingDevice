#ifndef HEADER_NUMERICAL_METHODS
#define HEADER_NUMERICAL_METHODS
// Uncomment if using this library outside of an embedded system with defined data sizes
//#define NUMERICAL_MDETHODS_STANDALONE

#include "utility_csd.h"

#include <ArduinoEigenDense.h>
#include <queue>

using namespace Eigen;

/**
 * @brief Finds the Kronecker product of two matrices
 * 
 * @param m1 
 * @param m2 
 * @return MatrixXf 
 */
MatrixXf kron(MatrixXf m1, MatrixXf m2);

int sign(float f);

/**
 * @brief Calculates the rotation matrix correcsponding to a rotation of 'deg' about the x axis
 * 
 * @param deg Angle in degrees
 * @return Matrix3f - Rotation matrix
 */
Matrix3f x_rotation(float deg);

/**
 * @brief Calculates the rotation matrix correcsponding to a rotation of 'deg' about the y axis
 * 
 * @param deg Angle in degrees
 * @return Matrix3f - Rotation matrix
 */
Matrix3f y_rotation(float deg);

/**
 * @brief Calculates the rotation matrix correcsponding to a rotation of 'deg' about the z axis
 * 
 * @param deg Angle in degrees
 * @return Matrix3f - Rotation matrix
 */
Matrix3f z_rotation(float deg);

/**
 * @brief Given a point cloud, calculate the best fit ellipsoid (linear least squares), returning the quadratic ellipsoid parameters
 * 
 * @param samples Samples to
 * @return RowVector<float,10> Ellipsoid parameters
 */
RowVector<float,10> fit_ellipsoid(const MatrixXf &samples, int n_samples = -1);

/**
 * @brief Calculates the transformation from a ellpipsoid to a sphere given the ellipsoid fitting parameters.
 * 
 * @param M 
 * @param n 
 * @param d 
 * @return Vector<float,12> - A transformation matrix [0:9], an offset vector [9:12]
 */
Vector<float,12> calculate_ellipsoid_transformation(Matrix3f &M, Vector3f &n, float d);

/**
 * @brief Given a point cloud, find a plane of best fit and return the vector normal to this plane
 * 
 * @param point_cloud
 * @return Vector3f 
 */
Vector3f NormalVec(const MatrixXf &point_cloud);


/**
 * @brief Calculates the standard deviation of a matrix
 * 
 * @param m 
 * @return float 
 */
float StdDev(MatrixXf m);

/**
 * @brief Given a set of calibrated magnetometer and accelerometer data, this function
     * finds the least squares best fit for the alignment of the sensor axis and outputs
     * a rotation matrix for correcting the magnetometer and the magnetic inclination at
     * the location of measurement.
 * 
 * @param g Accelerometer data
 * @param m Magnetometer data
 * @return Vector<float,10> 
 */
Vector<float,10> AlignMagAcc(const MatrixXf &g, const MatrixXf &m);
#endif