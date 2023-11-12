//
// Created by chris on 21/05/2023.
//

#ifndef NUMERICALMETHODSTESTING_UTIL_H
#define NUMERICALMETHODSTESTING_UTIL_H

#include <iostream>
#include <format>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include "Eigen/Dense"

#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105


using namespace std;
using namespace Eigen;

// Right-handed rotation
Matrix3f quatRot(Vector3f ax, float rads)
{
    Quaternionf Q(cos(rads/2),ax(0)*sin(rads/2), ax(1)*sin(rads/2), ax(2)*sin(rads/2));
    return Q.toRotationMatrix();
}

Matrix3f xRotation(float rad)
{
    return quatRot(Vector3f(1,0,0),rad);
}

Matrix3f yRotation(float rad)
{
    return quatRot(Vector3f(0,1,0),rad);
}

Matrix3f zRotation(float rad)
{
    return quatRot(Vector3f(0,0,1),rad);

}

MatrixXf kron(MatrixXf m1, MatrixXf m2)
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

MatrixXf readFromFile(const char* fname)
{
    std::ifstream fin;
    std::vector<float> numbers;
    char ch;
    float num;

    fin.open(fname,ios::in);
    assert (!fin.fail( ));
    fin >> num;
    fin >> ch;
    numbers.push_back(num);

    while (!fin.eof( ))      //if not at end of file, continue reading numbers
    {
        cout<<num<<endl;    //print numbers to screen
        fin >> num;               //get next number from file
        numbers.push_back(num);
//        fin >> ch;
    }
    fin.close( );       //close file

    float* ptr_data = numbers.data();
    MatrixXf data = MatrixXf::Map(ptr_data, 3,(int)(numbers.size()/3));
    data.transposeInPlace();
//    cout << "\nData:\n" << data << "\n";
    return data;
}

Vector3f cardanToCartesian(Vector3f cardan)
{
    Vector3f cartesian;
    cartesian << (cos(cardan(1))*cos(cardan(0))),
            (cos(cardan(1))*sin(cardan(0))),
            (sin(cardan(1)));
    return cartesian;
}

float getHeading(Vector3f m)
{
    if ((m(0) > 0)) // x +ve, y -ve = 0 tp pi/2
    {
        return -atan(m(1)/m(0));

    } else if ((m(0) < 0) && (m(1) < 0)) // x -ve, y -ve = pi/2 to pi
    {
        return  atan(m(1)/m(0)) - M_PI;

    } else if ((m(0) < 0) && (m(1) > 0)) // x -ve, y+ve = -pi to -pi/2
    {
        return atan(m(1)/m(0)) + M_PI;

    } else if ((m(0) == 0) && (m(1) < 0))
    {
        return -M_PI/2;

    } else if ((m(0) == 0) && (m(1) < 0))
    {
        return M_PI_2;

    } else {
        return 0;
    }
}

Vector3f inertialToCardan(Vector3f g, Vector3f m) {
    float heading, inclination, roll;
    Vector3f hir;
    roll = atan(-g(1) / abs(g(2)));
    if (g(2) > 0)
    {
        roll = M_PI - roll;
    }

    inclination = atan(-g(0) / pow(pow(g(1), 2) + pow(g(2), 2), 0.5));

    // Rotate roll (roll on device mirrored by roll in measurements)
    Vector3f m_roll_reversed = quatRot(Vector3f(1, 0, 0), roll) * m;
    m_roll_reversed(0) = m_roll_reversed(0) / cos(inclination);
    heading =  atan2(-m_roll_reversed(1),m_roll_reversed(0)); //getHeading(m_roll_reversed);

    hir << heading, inclination, roll;
    return hir;
}

Vector3f inertialToCartesian(Vector3f g, Vector3f m)
{
    Vector3f hir;
    hir = cardanToCartesian(inertialToCardan(g,m));
    return hir;
}

Matrix<float,2,3> toInertial(Vector3f XYZ, float roll) {
    Matrix<float,2,3> gm;
    Vector3f x,y,z;

    x = XYZ.normalized();

    // Cross product follow Right-Hand-Rule so MUST have correct order
    y = x.cross(Vector3f(0, 0, -1));
    z = x.cross(y);

    y = quatRot(x, roll) * y;
    z = quatRot(x, roll) * z;
    y.normalize();
    z.normalize();

    gm.row(0) << Vector3f(0, 0, -1).dot(x), Vector3f(0, 0, -1).dot(y), Vector3f(0, 0, -1).dot(z);
    gm.row(1) << Vector3f(1, 0, 0).dot(x), Vector3f(1, 0, 0).dot(y), Vector3f(1, 0, 0).dot(z);

    return gm;
}


const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, "\t", "\n");

void writeToCSVfile(std::string name, MatrixXf matrix)
{
    std::ofstream file(name.c_str());
    file << matrix.format(CSVFormat);
}


#endif //NUMERICALMETHODSTESTING_UTIL_H
