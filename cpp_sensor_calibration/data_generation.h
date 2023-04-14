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


// #define DISPERSED_GENERATION
#ifdef DISPERSED_GENERATION
#define N_SAMPLES 100
#else
#define N_SAMPLES 12*10
#endif

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
#ifdef DISPERSED_GENERATION
    // MUST CHANGE N_SAMPLES IF THESE ARE CHANGED
    int n_y = 10;
    int n_z = 10;
    // ------------------------------------------
    int y;
    int z;
    int n = 0;

    MatrixXf samples = MatrixXf::Zero(3,N_SAMPLES);
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
#else
    VectorXf x_rots(12);
    x_rots << 0, 0 , 180, 180, 270, 270, 270, 90 , 0  , 0  , 0  , 0;
    VectorXf y_rots(12);
    y_rots << 0, 0 , 90 , 90 , 0  , 0  , 180, 0  , 270, 270, 90 , 90;
    VectorXf z_rots(12);
    z_rots << 0, 90, 0  , 45 , 270, 0  , 0  , 225, 90 , 180, 180, 225;

    Matrix3f Rx, Ry, Rz, R;
    MatrixXf samples = MatrixXf::Zero(3,12);

    for (int n=0; n<12; n++)
    {
        Rx = x_rotation(x_rots(n));
        Ry = y_rotation(y_rots(n));
        Rz = z_rotation(z_rots(n));
        R = Rx * Ry * Rz;
        samples.col(n) = R * true_vec;
    }
    return samples;
#endif
}

MatrixXf generate_samples(MatrixXf true_data, Matrix3f Tm, Vector3f hm)
{
    int n = true_data.cols();
    MatrixXf samples = MatrixXf::Zero(3,N_SAMPLES);
    std::default_random_engine generator;
    std::normal_distribution<float> distribution(0,0.005);
    Vector3f noise;
    for(int i=0;i<n;i++)
    {
        for(int j=0;j<10;j++)
        {
            noise << distribution(generator),distribution(generator),distribution(generator);
            //std::cout << "i: " << i << "\n" << true_data.col(i) << "\n\n";
            samples.col(i*10+j) = Tm * true_data.col(i) + hm + noise;
        }
    }
    return samples;
}


#endif //MAGNETOMETER_CALIBRATION_DATA_GENERATION_H
