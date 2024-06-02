#include "utils.h"
namespace NumericalMethods {

float angle(Vector3f u, Vector3f v)
{
    return acos(u.dot(v)/(u.norm()*v.norm()));
}

Matrix3f quatRot(const Vector3f &ax, float rads)
{
    Vector3f ax_normalised = ax.normalized();
    Quaternionf Q(cos(rads/2),ax_normalised(0)*sin(rads/2), ax_normalised(1)*sin(rads/2), ax_normalised(2)*sin(rads/2));
    return Q.toRotationMatrix();
}

Matrix3f xRotation(const float rad)
{
    return quatRot(Vector3f(1,0,0),rad);
}

Matrix3f yRotation(const float rad)
{
    return quatRot(Vector3f(0,1,0),rad);
}

Matrix3f zRotation(const float rad)
{
    return quatRot(Vector3f(0,0,1),rad);

}

MatrixXf kron(const MatrixXf &m1, const MatrixXf &m2)
{
    int m = m1.rows();
    int n = m1.cols();
    int p = m2.rows();
    int q = m2.cols();

    MatrixXf out(m*p, n*q);
    for(int i=0; i<m; i++)
    {
        for(int j=0; j<n; j++) {
            out.block(i*p,j*q,p,q) = m1(i,j) * m2;
        }
    }
    return out;
}

Matrix3f inertialToENU(const Vector3f &m, const Vector3f &g)
{
    Matrix3f ENU;
    
    // Use cross product to generate set of real world axis in body frame
    Vector3f E, N, U;
    E = g.cross(m);
    N = g.cross(E);
    U = N.cross(E);

    ENU << E, N, U;
    return ENU;
}

Vector3f inertialToVector(const Vector3f &m, const Vector3f &g)
{
    Vector3f V;
    Matrix3f ENU = inertialToENU(m, g);

    // Extract the x values of each axis to find the x-axis in the world frame
    V << ENU(0,0), ENU(1,0), ENU(2,0);
    return V;
}

Vector3f inertialToCardan(const Vector3f &m, const Vector3f &g)
{
    Vector3f HIR;
    Matrix3f ENU = inertialToENU(m, g);

    // atan2(Ex,Nx) -> atan2 of north and east components of  sensor x-axis in world frame
    HIR(0) = atan2(ENU(0,0),ENU(1,0));

    // atan2(Ux,sqrt(Ex^2 + Nx^2)) -> Inclination of ENU above XZ plane
    // Alternatively atan2(Ux*cos(heading), Nx) works as sqrt(Ex^2 + Nx^2) = Nx/cos(heading)
    HIR(1) = atan2(ENU(2,0), sqrt(pow(ENU(0,0),2) + pow(ENU(1,0),2)));

    // Angle between device z axis and actual g measurement when projected into theYZ plane
    HIR(2) = atan2(-ENU(2,1),ENU(2,2));

    // Bind data to 0 to 2*pi
    if (fabs(HIR(0)) > 2*M_PI){         HIR(0) = HIR(0) - 2*M_PI;           }
    if (fabs(HIR(1)) > 2*M_PI){         HIR(1) = HIR(1) - 2*M_PI;           }
    if (fabs(HIR(2)) > 2*M_PI){         HIR(2) = HIR(2) - 2*M_PI;           }
    
    return HIR;
}

int sign(const float &f)
{
    if (f>=0)
    {
        return 1;
    } else {
        return -1;
    }
}

float stDev(const VectorXf &vec)
{
    return sqrt((vec.array() - vec.mean()).square().sum()/(vec.size()-1));
}

template <typename Derived>
int removeNullData(MatrixBase<Derived> &mat)
{
    // Initialise blank cols mat to -1
    VectorXi blank_cols(mat.cols());
    blank_cols.setOnes();
    blank_cols *= -1;
    int index = 0;
    int n_zeros = 0; // Number of zeroes found

    int i;
    // Index zero values in reverse order
    for (int i=mat.cols()-1; i>-1; i--)
    {
        if (mat.col(i).norm() == 0)
        {
            blank_cols(index) = i;
            index++;
        }
    }
    n_zeros = index;

    // Push index back due to index++ happening AFTER assignment
    index--;
    if (index == -1)
    {
        // No zeroes found
        return 0;
    }

    // Iterate in reverse through matrix, replacing zero valued sections with non-zero valued elements nearest the end of the matrix, replacing those with zero
    for (int i=mat.cols()-1; i>-1; i--)
    {
        // Check if value is non-zero
        if (mat.col(i).norm() > 0)
        {
            // Replace zero value closest to start or matrix with non-zero value
            mat.col(blank_cols(index)) = mat.col(i);
            // Replace non-zero value with zero
            mat.col(i) << 0, 0, 0;
            // Decrease index
            index--;

            // If all zero-valued sections have been replaced, break
            if (index < 0)
            {
                break;
            }
        }
    }

    return n_zeros;
}

}