#ifndef NUMERICAL_METHODS_FITTINGFUNCS_H
#define NUMERICAL_METHODS_FITTINGFUNCS_H

#include "utils.h"
namespace NumericalMethods {

Vector3f normalVec(const Ref<const MatrixXf> &point_cloud);

RowVector<float,10> fitEllipsoid(const Ref<const MatrixXf> &samples);

void fitEllipsoid(const Ref<const MatrixXf> &samples, Matrix3f &M_out, Vector3f &n_out, float &d_out);

Vector<float,12> calculateEllipsoidTransformation(const Matrix3f &M, const Vector3f &n, const float &d);
/**
 * @brief Given a set of unformatted ellipsoid parameters, find the 3x3 rotation matrix and 3x1 offset matrix which transforms the ellipsoid into a sphere
 * @todo Add sources for calculations
 * 
 * @param U Unformatted ellipsoid parameters
 * @return Vector<float,12> 
 */
 Vector<float,12> calculateEllipsoidTransformation(const RowVector<float,10> &U);
 void calculateEllipsoidTransformation(const RowVector<float,10> &U, Matrix3f &R_out, Vector3f &b_out);
 void calculateEllipsoidTransformation(const Matrix3f &M, const Vector3f &n, const float &d, Matrix3f &R_out, Vector3f &b_out);
}
#endif