#include "LaserAlignment.h"

namespace NumericalMethods{
   
void alignLaser(const MatrixXf &g, const MatrixXf &m, Matrix3f &Racc, Matrix3f &Rmag)
{
    alignToNorm(g, Racc);
    alignToNorm(m, Rmag);
}

void alignToNorm(const Matrix<float,3,N_LASER_CAL> &point_cloud, Matrix3f &R)
{
    // Calculate normal to plane
    Vector3f target_vector;
    target_vector = normalVec(point_cloud);
    target_vector = target_vector/target_vector.norm();
    if (target_vector.dot(point_cloud.col(0)) < 0)
    {
        target_vector = -target_vector;
    }

    Vector3f vector_x = target_vector;
    Vector3f vector_z = vector_x.cross(Vector3f(0,-1,0));
    Vector3f vector_y = vector_z.cross(vector_x);

    vector_x.normalize();
    vector_y.normalize();
    vector_z.normalize();

    // Transpose of a rotation matrix is its inverse
    R.row(0) = vector_x;
    R.row(1) = vector_y;
    R.row(2) = vector_z;
}

}