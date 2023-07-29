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

float Rad2Deg(float rad) {
    return rad * (180.0/M_PI);
}

Matrix3f x_rotation(float deg)
{
    deg = Deg2Rad(deg);
    Matrix3f R;
    R << cos(deg), 0., sin(deg),
            0., 1., 0.,
            -sin(deg), 0., cos(deg);
    return R;
}

Matrix3f y_rotation(float deg)
{
    deg = Deg2Rad(deg);
    Matrix3f R;
    R << 1., 0., 0.,
            0., cos(deg), -sin(deg),
            0., sin(deg), cos(deg);
    return R;
}

Matrix3f z_rotation(float deg)
{
    deg = Deg2Rad(deg);
    Matrix3f R;
    R << cos(deg), -sin(deg), 0.,
            sin(deg), cos(deg), 0.,
            0., 0. , 1.;
    return R;
}

Vector3f arbitrary_rotation(float theta, Vector3f a, Vector3f b)
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
