//
// Created by chris on 13/04/2023.
//

#ifndef MAGNETOMETER_CALIBRATION_DATA_GENERATION_H
#define MAGNETOMETER_CALIBRATION_DATA_GENERATION_H

#include "Eigen/Eigen"
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include <random>
#define _USE_MATH_DEFINES
#include <math.h>

using namespace Eigen;


float Deg2Rad(float degrees) {
    return degrees * (M_PI / 180.0);
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

MatrixXf generate_true_data(Vector3f true_vec)
{
    int n_y = 10;
    int n_z = 10;
    int y;
    int z;
    int n = 0;

    MatrixXf samples = MatrixXf::Zero(3,n_y*n_z);
    Matrix3f R;

    for(y=0;y<n_y;y++)
    {
        for(z=0;z<n_z;z++)
        {
            R = y_rotation(360.* y/n_y) * z_rotation(360.* z/n_z);
            samples.col(n) = R * true_vec;
            n++;
        }
    }

    return samples;
}

MatrixXf generate_samples(MatrixXf true_data, Matrix3f Tm, Vector3f hm)
{
    int n = true_data.cols();
    MatrixXf samples = MatrixXf::Zero(3,n);
    std::default_random_engine generator;
    std::normal_distribution<float> distribution(0,0.03);
    Vector3f noise;
    for(int i=0;i<n;i++)
    {
        noise << distribution(generator),distribution(generator),distribution(generator);
        //std::cout << "i: " << i << "\n" << true_data.col(i) << "\n\n";
        samples.col(i) = Tm * true_data.col(i) + hm + noise;
    }
    return samples;
}


#endif //MAGNETOMETER_CALIBRATION_DATA_GENERATION_H
