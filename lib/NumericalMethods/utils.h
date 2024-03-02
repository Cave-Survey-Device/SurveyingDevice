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
 * @brief Convert inertial data to cardan angles
 * 
 * @param g Gravitation reading of device
 * @param m Magnetic reading of device
 * @return Vector3f 
 */
 Vector3f inertialToCardan(const Vector3f &g, const Vector3f &m);
/**
 * @brief Converts cardan angles into cartesian data
 * 
 * @param cardan 
 * @return Vector3f 
 */
 Vector3f cardanToCartesian(Vector3f cardan);
 Vector2f cartesianToCardan(const Vector3f &XYZ);
/**
 * @brief Convert inertial readings into a cartesian direction of the device
 * 
 * @param g 
 * @param m 
 * @return Vector3f 
 */
 Vector3f inertialToCartesian(const Vector3f &g, const Vector3f &m);
/**
 * @brief Convert cartesian data with an associated roll into corresponding g and m vectors to reproduce such data
 * 
 * @param XYZ Cartesian value of direction
 * @param roll Roll of the frame about the given direction vector
 * @return Matrix<float,3,2> 
 */
 Matrix<float,3,2> toInertial(const Vector3f &XYZ, const float &roll);
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