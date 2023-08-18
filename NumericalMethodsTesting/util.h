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

using namespace std;
using namespace Eigen;

float Deg2Rad(float degrees) {
    return degrees * (M_PI / 180.0);
}
double Deg2Rad(double degrees) {
    return degrees * (M_PI / 180.0);
}

float Rad2Deg(float rad) {
    return rad * (180.0/M_PI);
}
double Rad2Deg(double rad) {
    return rad * (180.0/M_PI);
}

Matrix3f x_rotation(float rad)
{
    Matrix3f R;
    R << 1., 0., 0.,
            0., cos(rad), -sin(rad),
            0., sin(rad), cos(rad);
    return R;
}
Matrix3d x_rotation(double rad)
{
    Matrix3d R;
    R << 1., 0., 0.,
            0., cos(rad), -sin(rad),
            0., sin(rad), cos(rad);
    return R;
}

Matrix3f y_rotation(float rad)
{
    Matrix3f R;
    R << cos(rad), 0., sin(rad),
            0., 1., 0.,
            -sin(rad), 0., cos(rad);
    return R;
}
Matrix3d y_rotation(double rad)
{
    Matrix3d R;
    R << cos(rad), 0., sin(rad),
            0., 1., 0.,
            -sin(rad), 0., cos(rad);
    return R;
}

Matrix3f z_rotation(float rad)
{
    Matrix3f R;
    R << cos(rad), -sin(rad), 0.,
            sin(rad), cos(rad), 0.,
            0., 0. , 1.;
    return R;
}
Matrix3d z_rotation(double rad)
{
    Matrix3d R;
    R << cos(rad), -sin(rad), 0.,
            sin(rad), cos(rad), 0.,
            0., 0. , 1.;
    return R;
}

Vector3f Spherical(Vector3f cartesian){
    Vector3f spherical;
    spherical << atan2(cartesian(1), cartesian(0)),
            atan2(pow( pow(cartesian(0),2) + pow(cartesian(1),2), 0.5),cartesian(2)),
            cartesian.norm();
    return spherical;
}
Vector3d dSpherical(Vector3d cartesian){
    Vector3d spherical;
    spherical << atan2(cartesian(1), cartesian(0)),
            atan2(pow( pow(cartesian(0),2) + pow(cartesian(1),2), 0.5),cartesian(2)),
            cartesian.norm();
    return spherical;
}

// Anti-clockwise rotation about the given axis when looking aling it
Matrix3f quatRot(Vector3f ax, float theta)
{
    theta = -theta;
    Quaternionf Q(cos(theta/2),ax(0)*sin(theta/2), ax(1)*sin(theta/2), ax(2)*sin(theta/2));
    return Q.toRotationMatrix();
}

// Rotate vector a about b by theta
Vector3f arbitrary_rotation(Vector3f a, Vector3f b, float theta)
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
Vector3d darbitrary_rotation(Vector3d a, Vector3d b, double theta)
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

MatrixXf read_from_file(const char* fname)
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

const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, "\t", "\n");

void writeToCSVfile(std::string name, MatrixXf matrix)
{
    std::ofstream file(name.c_str());
    file << matrix.format(CSVFormat);
}


#endif //NUMERICALMETHODSTESTING_UTIL_H
