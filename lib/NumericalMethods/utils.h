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

/**
 * @brief Finds the rotation matrix correcponding to a quaternion rotation about a normalised axis by an angle of rads
 * 
 * @param ax Axis to rotate about
 * @param rads Angle to rotate by (accoring to the Right Hand Rule)
 * @return Matrix3f 
 */
Matrix3f quatRot(const Vector3f &ax, float rads)
{
    Vector3f ax_normalised = ax.normalized();
    Quaternionf Q(cos(rads/2),ax_normalised(0)*sin(rads/2), ax_normalised(1)*sin(rads/2), ax_normalised(2)*sin(rads/2));
    return Q.toRotationMatrix();
}

/**
 * @brief Right-Handed Rotation about the x-axis by an angle or rad
 * 
 * @param rad 
 * @return Matrix3f 
 */
Matrix3f xRotation(const float rad)
{
    return quatRot(Vector3f(1,0,0),rad);
}

/**
 * @brief Right-Handed Rotation about the y-axis by an angle or rad
 * 
 * @param rad 
 * @return Matrix3f 
 */
Matrix3f yRotation(const float rad)
{
    return quatRot(Vector3f(0,1,0),rad);
}

/**
 * @brief Right-Handed Rotation about the z-axis by an angle or rad
 * 
 * @param rad 
 * @return Matrix3f 
 */
Matrix3f zRotation(const float rad)
{
    return quatRot(Vector3f(0,0,1),rad);

}

/**
 * @brief Calculate the Kronecker product of matrices m1 and m2
 * 
 * @param m1 
 * @param m2 
 * @return MatrixXf 
 */
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


/**
 * @brief Convert inertial data to cardan angles
 * 
 * @param g Gravitation reading of device
 * @param m Magnetic reading of device
 * @return Vector3f 
 */
Vector3f inertialToCardan(const Vector3f &g, const Vector3f &m) {
    float heading, inclination, roll;
    Vector3f hir;

    // Find roll of the device
    roll = atan(-g(1) / abs(g(2)));
    if (g(2) > 0)
    {
        roll = M_PI - roll;
    }

    // Find inclination of the device
    inclination = atan(-g(0) / pow(pow(g(1), 2) + pow(g(2), 2), 0.5));

    // Rotate device by the roll and scale of inclination to provide accurate heading
    Vector3f m_roll_reversed = quatRot(Vector3f(1, 0, 0), roll) * m;
    m_roll_reversed(0) = m_roll_reversed(0) / cos(inclination);
    heading =  atan2(-m_roll_reversed(1),m_roll_reversed(0)); //getHeading(m_roll_reversed);

    hir << heading, inclination, roll;
    return hir;
}

/**
 * @brief Converts cardan angles into cartesian data
 * 
 * @param cardan 
 * @return Vector3f 
 */
Vector3f cardanToCartesian(Vector3f cardan)
{
    Vector3f cartesian;
    cartesian << (cos(cardan(1))*cos(cardan(0))),
            (cos(cardan(1))*sin(cardan(0))),
            (sin(cardan(1)));
    return cartesian;
}

/**
 * @brief Convert inertial readings into a cartesian direction of the device
 * 
 * @param g 
 * @param m 
 * @return Vector3f 
 */
Vector3f inertialToCartesian(const Vector3f &g, const Vector3f &m)
{
    Vector3f hir;
    hir = cardanToCartesian(inertialToCardan(g,m));
    return hir;
}

/**
 * @brief Convert cartesian data with an associated roll into corresponding g and m vectors to reproduce such data
 * 
 * @param XYZ Cartesian value of direction
 * @param roll Roll of the frame about the given direction vector
 * @return Matrix<float,3,2> 
 */
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


#endif