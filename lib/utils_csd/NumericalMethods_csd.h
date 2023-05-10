#ifndef HEADER_NUMERICAL_METHODS
#define HEADER_NUMERICAL_METHODS

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
RowVector<float,10> fit_ellipsoid(const MatrixXf &samples);

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
 * @brief Cost function: https://ieeexplore.ieee.org/document/8723161 Equation (15)
 * 
 * @param f - Accelerometer samples
 * @param m - Magnetometer samples
 * @param X - Vector of unknowns
 * @return float - Cost
 */
float J(const MatrixXf &f, const MatrixXf &m, const Vector<float, 10> &X);

/**
 * @brief https://ieeexplore.ieee.org/document/8723161 Equation (16)
 * 
 * @param f - Accelerometer samples
 * @param m - Magnetometer samples
 * @param X - Vector of unknowns
 * @return Vector<float, 9> dJ/dR
 */
Vector<float, 9> dJ_dR (const MatrixXf &f, const MatrixXf &m, const Vector<float, 10> &X);

/**
 * @brief https://ieeexplore.ieee.org/document/8723161 Equation (16)
 * 
 * @param f - Accelerometer samples
 * @param m - Magnetometer samples
 * @param X - Vector of unknowns
 * @return float  dJ/dd
 */
float dJ_dd (const MatrixXf &f, const MatrixXf &m, const Vector<float, 10> &X);

/**
 * @brief Calculate the gradient of the cost function
 * 
 * @param f - Accelerometer samples
 * @param m - Magnetometer samples
 * @param X - Vector of unknowns
 * @return Vector<float,10> dJ/dX
 */
Vector<float,10> GradJ(const MatrixXf &f, const MatrixXf &m, const Vector<float, 10> &X);

/**
 * @brief Perform alignment calculations
 * 
 * @param f - Accelerometer samples
 * @param m - Magnetometer samples
 * @return Vector<float,10> R, d
 */
Vector<float,10> Align(const MatrixXf &f, const MatrixXf &m);

#endif