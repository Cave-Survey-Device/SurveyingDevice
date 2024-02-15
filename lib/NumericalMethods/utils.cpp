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

Vector3f inertialToCardan(const Vector3f &g_in, const Vector3f &m_in) {
    static Vector3f g, m;
    g = g_in.normalized();
    m = m_in.normalized();
    float heading, inclination, roll;
    Vector3f hir, m_roll_reversed;

    // Find roll of the device
    roll = atan(-g(1) / abs(g(2)));
    if (g(2) > 0)
    {
        roll = M_PI - roll;
    }

    // // Find inclination of the device
    // inclination = -atan(-g(0) / pow(pow(g(1), 2) + pow(g(2), 2), 0.5));

    // // Rotate device by the roll and scale of inclination to provide accurate heading
    // Vector3f m_roll_reversed = quatRot(Vector3f(1, 0, 0), roll) * m;
    // m_roll_reversed(0) = m_roll_reversed(0) / cos(inclination);
    // heading =  atan2(-m_roll_reversed(1),m_roll_reversed(0)); //getHeading(m_roll_reversed);

    inclination = asin(g(0)/g.norm());
    m_roll_reversed = quatRot(Vector3f(1, 0, 0), roll) * m;
    m_roll_reversed = quatRot(Vector3f(0, 1, 0),inclination) * m_roll_reversed;
    // Serial.printf("M roll reversed: %f %f %f\n", m_roll_reversed(0), m_roll_reversed(1), m_roll_reversed(2));
    // Serial.printf("M: %f %f %f          G: %f %f %f\n", m(0), m(1), m(2), g(0), g(1), g(2));

    // Serial.printf("M_roll_reversed: %f %f %f\n",m_roll_reversed(0),m_roll_reversed(1),m_roll_reversed(2));

    heading =  -atan2(m_roll_reversed(1),m_roll_reversed(0));

    hir << heading, inclination, roll;
    return hir;
}

Vector3f cardanToCartesian(Vector3f cardan)
{
    Vector3f cartesian;
    cartesian << (cos(cardan(1))*cos(cardan(0))),
            (cos(cardan(1))*sin(cardan(0))),
            (-sin(cardan(1)));
    return cartesian;
}

Vector2f cartesianToCardan(const Vector3f &XYZ)
{
    Vector2f cardan;
    float heading, inclination;;

    heading = atan2(XYZ(1),XYZ(0) / cos(inclination));
    inclination = -asin(XYZ(2)/XYZ.norm());
    cardan << heading, inclination;
    return cardan;
}
  
Vector3f inertialToCartesian(const Vector3f &g, const Vector3f &m)
{
    Vector3f hir;
    hir = cardanToCartesian(inertialToCardan(g,m));
    return hir;
}

Matrix<float,3,2> toInertial(const Vector3f &XYZ, const float &roll) {
    Matrix<float,3,2> gm;
    Vector3f x,y,z;

    x = XYZ.normalized();

    // Cross product follow Right-Hand-Rule so MUST have correct order
    y = x.cross(Vector3f(0, 0, -1));
    z = x.cross(y);

    y = quatRot(x, roll) * y;
    z = quatRot(x, roll) * z;
    y.normalize();
    z.normalize();

    // Find proportion of m and g acting upon new axis
    gm.col(0) << Vector3f(0, 0, -1).dot(x), Vector3f(0, 0, -1).dot(y), Vector3f(0, 0, -1).dot(z);
    gm.col(1) << Vector3f(1, 0, 0).dot(x), Vector3f(1, 0, 0).dot(y), Vector3f(1, 0, 0).dot(z);

    return gm;
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