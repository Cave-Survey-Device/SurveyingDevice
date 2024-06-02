#ifndef NUMERICAL_METHODS_UTILS_H
#define NUMERICAL_METHODS_UTILS_H


#define NUMERICAL_METHODS_ARDUINO_EIGEN
// #define NUMERICAL_METHODS_EIGEN

#ifdef NUMERICAL_METHODS_ARDUINO_EIGEN
  #include <ArduinoEigenDense.h>
  using namespace Eigen;
#else
#ifdef NUMERICAL_METHODS_EIGEN
  #include <Eigen/Dense>
  using namespace Eigen;
#endif
#endif

#include "config.h"

namespace NumericalMethods {

/**
 * @brief Find the angle between two vectors
 * 
 * @param u 
 * @param v 
 * @return float 
 */
float angle(Vector3f u, Vector3f v);

/**
 * @brief Finds the rotation matrix correcponding to a quaternion rotation about a normalised axis by an angle of rads
 * 
 * @param ax Axis to rotate about
 * @param rads Angle to rotate by (accoring to the Right Hand Rule)
 * @return Matrix3f 
 */
 Matrix3f quatRot(const Vector3f &ax, float rads);
/**
 * @brief Right-Handed Rotation about the x-axis by an angle or rad
 * 
 * @param rad 
 * @return Matrix3f 
 */
 Matrix3f xRotation(const float rad);
/**
 * @brief Right-Handed Rotation about the y-axis by an angle or rad
 * 
 * @param rad 
 * @return Matrix3f 
 */
 Matrix3f yRotation(const float rad);
/**
 * @brief Right-Handed Rotation about the z-axis by an angle or rad
 * 
 * @param rad 
 * @return Matrix3f 
 */
 Matrix3f zRotation(const float rad);
/**
 * @brief Calculate the Kronecker product of matrices m1 and m2
 * 
 * @param m1 
 * @param m2 
 * @return MatrixXf 
 */
MatrixXf kron(const MatrixXf &m1, const MatrixXf &m2);
/**
 * @brief Given a set of inetial data, return a matrix of the East, North, and Up axis with respect to the body frame
 * 
 * @param m Magnetometer data
 * @param g Gravitational data
 * @param ENU East-Noth-UP matrix
 * @return Matrix3f East-Noth-UP matrix
 */
Matrix3f inertialToENU(const Vector3f &m, const Vector3f &g);
/**
 * @brief Given a set of inetial data, return a vector representing the x-axis in the world frame
 * 
 * @param m Magnetometer data
 * @param g Gravitational data
 * @param V Output vector
 * @return Vector3f Output vector
 */
Vector3f inertialToVector(const Vector3f &m, const Vector3f &g);
/**
 * @brief Given a set of inetial data, return a vector containing the device angles in teh form of Heading-Inclination-Roll
 * 
 * @param m Magnetometer data
 * @param g Gravitational data
 * @param HIR Output Vector Heading-Inclination-Roll
 * @return Vector3f Output Vector Heading-Inclination-Roll
 */
Vector3f inertialToCardan(const Vector3f &m, const Vector3f &g);

/**
 * @brief Returns the sign of the given float
 * 
 * @param f 
 * @return int 
 */
 int sign(const float &f);
 float stDev(const VectorXf &vec);
/**
 * @brief Iterates through a matrix shifting all non-zero columns to the start of the matrix.
 * This has the effect of moving all zero values to the end of the matrix.
 * Returns the number of zeros in the matrix.
 * 
 * @tparam Derived 
 * @param mat 
 * @return int 
 */
int removeNullData(Ref<MatrixXf> mat);
}

#endif