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

//class Frame
//{
//public:
//    Vector3f update(const Vector3f &g, const Vector3f &m);
//    Vector3f getXYZ();
//    Vector3f getHIR();
//private:
//    Vector3f XYZ;
//    float HIR;
//};




float Deg2Rad(float degrees) {
    return degrees * (M_PI / 180.0);
}

float Rad2Deg(float rad) {
    return rad * (180.0/M_PI);
}

Matrix3f xRotation(float rad)
{
    Matrix3f R;
    R << 1., 0., 0.,
            0., cos(rad), -sin(rad),
            0., sin(rad), cos(rad);
    return R;
}

Matrix3f yRotation(float rad)
{
    Matrix3f R;
    R << cos(rad), 0., sin(rad),
            0., 1., 0.,
            -sin(rad), 0., cos(rad);
    return R;
}

Matrix3f zRotation(float rad)
{
    Matrix3f R;
    R << cos(rad), -sin(rad), 0.,
            sin(rad), cos(rad), 0.,
            0., 0. , 1.;
    return R;
}

// Anti-clockwise rotation about the given axis when looking aling it
Matrix3f quatRot(Vector3f ax, float rads)
{
    rads = -rads;
    Quaternionf Q(cos(rads/2),ax(0)*sin(rads/2), ax(1)*sin(rads/2), ax(2)*sin(rads/2));
    return Q.toRotationMatrix();
}

// Rotate vector a about b by theta
Vector3f arbitraryRotation(Vector3f a, Vector3f b, float theta)
{
    // https://math.stackexchange.com/a/1432182
    Vector3f aparb = (a.dot(b)/b.dot(b))*b;
    Vector3f aorthb = a - aparb;
    Vector3f w = b.cross(aorthb);

    float x1,x2;
    x1 = cos(theta)/aorthb.norm();
    x2 = sin(theta)/w.norm();

    Vector3f aorthb_rot = aorthb.norm() * (x1*aorthb + x2 * w);

    return aorthb_rot + aparb;

}
Vector3d darbitraryRotation(Vector3d a, Vector3d b, double theta)
{
    // https://math.stackexchange.com/a/1432182
    Vector3d aparb = (a.dot(b)/b.dot(b))*b;
    Vector3d aorthb = a - aparb;
    Vector3d w = b.cross(aorthb);

    double x1,x2;
    x1 = cos(theta)/aorthb.norm();
    x2 = sin(theta)/w.norm();

    Vector3d aorthb_rot = aorthb.norm() * (x1*aorthb + x2 * w);

    return aorthb_rot + aparb;

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

Vector3f inertialToCardan(Vector3f g, Vector3f m)
{
    float heading, inclination, roll;
    Vector3f hir;

    roll = -atan2(g(1),g(2));
    inclination = atan2(g(0),pow(pow(g(1),2)+pow(g(2),2),0.5));

    Vector3f m_roll_reversed = xRotation(-roll) * m;
    m_roll_reversed(0) = m_roll_reversed(0) / cos(inclination);

    // Heading is -ve of tan as m is in opposite direction to m
    heading = -atan2(m_roll_reversed(1),m_roll_reversed(0)); // asin(m(0)/m.norm());

    hir << heading, inclination, roll;
    return hir;
}

Vector3f inertialToCartesian(Vector3f g, Vector3f m)
{
    Vector3f hir;
    hir = cardanToCartesian(inertialToCardan(g,m));
    return hir;
}

const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, "\t", "\n");

void writeToCSVfile(std::string name, MatrixXf matrix)
{
    std::ofstream file(name.c_str());
    file << matrix.format(CSVFormat);
}


#endif //NUMERICALMETHODSTESTING_UTIL_H
