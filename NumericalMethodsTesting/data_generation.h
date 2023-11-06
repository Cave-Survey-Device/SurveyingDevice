//
// Created by chris on 13/04/2023.
//

#ifndef MAGNETOMETER_CALIBRATION_DATA_GENERATION_H
#define MAGNETOMETER_CALIBRATION_DATA_GENERATION_H

#include "Eigen/Dense"
#include <random>
#define _USE_MATH_DEFINES
#include <math.h>

#include "util.h"


#define N_MAG_SAMPLES 8000
#define N_X 20
#define N_Y 20
#define N_Z N_MAG_SAMPLES/N_Y/N_X

#define N_SAMPLES 12*25

#define INCLINATION_OFFSET 0.0872664626
#define HEADING_OFFSET -0.0436332313

using namespace Eigen;

/**
 * Generates a set of perfect data from an initial sample. This initial sample is the reference vector (0,0,1 for accelerometers and 1,0,0 for magnetometers) from which a spehrical point cloud is generated.
 * If DISPERSED_GENERATION is NOT defined, data will be generated according to the MAG.I.CAL calibration schema.
 * @param true_vec
 * @return
 */
MatrixXf generateTrueInertialAlignData(Vector3f true_vec)
{
    VectorXf x_rots(12);
    x_rots << 0, 0 , 180, 180, 270, 270, 270, 90 , 0  , 0  , 0  , 0;
    VectorXf y_rots(12);
    y_rots << 0, 0 , 0,   0,   0  , 0  , 180, 0  , 270, 270, 90 , 90;
    VectorXf z_rots(12);
    z_rots << 0, 90, 90,  135, 270, 0  , 0  , 225, 90 , 180, 180, 225;

    Matrix3f Rx, Ry, Rz, R;
    MatrixXf samples = MatrixXf::Zero(3,12);

    for (int n=0; n<12; n++)
    {
        Rx = xRotation(DEG_TO_RAD*x_rots(n));
        Ry = yRotation(DEG_TO_RAD*y_rots(n));
        Rz = zRotation(DEG_TO_RAD*z_rots(n));
        R = Rz * Ry * Rx;
        samples.col(n) = R * true_vec;
    }
    return samples;
//#endif
}

/**
 * Generates a set of perfect data from an initial sample. This initial sample is the reference vector (0,0,1 for accelerometers and 1,0,0 for magnetometers) from which a spehrical point cloud is generated.
 * If DISPERSED_GENERATION is NOT defined, data will be generated according to the MAG.I.CAL calibration schema.
 * @param true_vec
 * @return
 */
MatrixXf generateTrueMagData(Vector3f true_vec)
{
    // ------------------------------------------
    int x,y,z;
    int n = 0;

    MatrixXf samples = MatrixXf::Zero(3,N_MAG_SAMPLES);
    Matrix3f R;

    for(x=0;x<N_X;x++)
    {
        for(y=0;y<N_Y;y++)
        {
            for(z=0;z<N_Z;z++) {
                R = xRotation(2 * M_PI * x / N_X) * yRotation(2 * M_PI * y / N_Y) * zRotation(2 * M_PI * z / N_Z);
                samples.col(n) = R * true_vec;
                n++;
            }
        }
    }
    return samples;
}

MatrixXf generateMagCalSamples(MatrixXf true_data, Matrix3f Tm, Vector3f hm) {
    MatrixXf samples = MatrixXf::Zero(3, N_MAG_SAMPLES);
    std::default_random_engine generator;
    std::normal_distribution<float> distribution(0, 0.005);
    Vector3f noise;
    for (int i = 0; i < N_MAG_SAMPLES; i++) {
        noise << distribution(generator), distribution(generator), distribution(generator);
        samples.col(i) = Tm * true_data.col(i) + hm + noise;
    }
    return samples;
}


/**
 * Generates data to calibrate the inertial alignment of a sensor given a set of perfect data and misalignment parameters
 * @param true_data
 * @param Tm The misalignment matrix from which to generate the data
 * @param hm The bias vector from which to generate the data
 * @return
 */

MatrixXf generateInertialAlignData(MatrixXf true_data, Matrix3f Tm, Vector3f hm)
{
    int n = true_data.cols();
    MatrixXf samples = MatrixXf::Zero(3,N_SAMPLES);
    std::default_random_engine generator;
    std::normal_distribution<float> distribution(0,0.005);
    Vector3f noise;
    for(int i=0;i<n;i++)
    {
        for(int j=0;j<(int)(N_SAMPLES/n);j++)
        {
            noise << distribution(generator),distribution(generator),distribution(generator);
            //std::cout << "i: " << i << "\n" << true_data.col(i) << "\n\n";
            samples.col(i*N_SAMPLES/n+j) = Tm * true_data.col(i) + hm + noise;
        }
    }
    return samples;
}
/**
 * Generates data used to calibrate a laser sensor. This takes into account alignment of inertial sensors too.
 * @param inclination_offset
 * @param heading_offset
 * @return
 */
MatrixXf generateLaserAlignData(float inclination_offset, float heading_offset)
{
    // ---------------------------------------- Define needed parameters ---------------------------------------
    float initial_roll, combined_error;
    Vector3f laser_vec;

    const int n_rots = 8;
    const Vector3f target{1,1,1};
    const float disto_len = 0.1;

    const Vector3f x_ax = Vector3f(1,0,0);
    const Vector3f y_ax = Vector3f(0,1,0);
    const Vector3f z_ax = Vector3f(0,0,1);

    // ---------------------------------------- Find parameters of laser ---------------------------------------
    laser_vec = Vector3f(1,0,0);
    // Angles rotated counter-clockwise so -inclination, +ve heading
    laser_vec = quatRot(y_ax, inclination_offset) * laser_vec;
    laser_vec = quatRot(z_ax, -heading_offset) * laser_vec;

    combined_error = acos(laser_vec.dot(x_ax));
    initial_roll = -M_PI_2 - atan2(laser_vec(2),laser_vec(1));

    cout << "Laser vec: " << laser_vec(0) << "  " << laser_vec(1) << "  " << laser_vec(2) << "\n";
    cout << "Combined error: " << combined_error << "\n";
    cout << "Initial roll: " <<  Rad2Deg(initial_roll) << "\n";


    // ------------------------------------ Find initial position of device ------------------------------------
    float  target_len, alpha, beta, gamma, theta;
    Vector3f initial_disto_tip, target_vec;
    target_len = target.norm();
    target_vec = target/target_len;
    theta = combined_error;
    alpha = M_PI - theta;
    beta = asin(disto_len * sin(alpha)/target_len);
    gamma = theta-beta;
    cout << "gamma: " << Rad2Deg(gamma) << "\n";

    // Generate initial disto tip by rotating ito targex X z_ax plane
    Vector3f ax;
    ax = target_vec.cross(z_ax);
    ax = ax/ax.norm();
    initial_disto_tip = quatRot(ax,-gamma) * target/target.norm() * disto_len;

//    cout << "Initial rotation: \n" << quatRot(ax,-gamma) << "\n";
    cout << "Initial disto tip: " << initial_disto_tip(0) << "  " << initial_disto_tip(1) << "  " << initial_disto_tip(2) << "\n";



    // ------------------------------------ Rotate device about target axis ------------------------------------
    float phi, heading, inclination, roll;
    Vector3f disto_tip;
    Matrix<float,4,n_rots> out_hird;

    for (int i=0; i<n_rots; i++)
    {
        phi = i * 2*M_PI/n_rots;
        roll = initial_roll + phi;
        disto_tip = quatRot(target_vec,-phi) * initial_disto_tip;
        laser_vec = target - disto_tip;
        cout << "Disto tip: " << disto_tip(0) << "  " << disto_tip(1) << " " << disto_tip(2) << "\n";


        heading = atan2(disto_tip(1), disto_tip(0));
        inclination = atan2(pow( pow(disto_tip(0),2) + pow(disto_tip(1),2), 0.5),disto_tip(2));
//        out_hird.col(i) << disto_tip, 1;
         out_hird.col(i) << heading , inclination, roll, laser_vec.norm();
    }

    return out_hird;
}
#endif //MAGNETOMETER_CALIBRATION_DATA_GENERATION_H
