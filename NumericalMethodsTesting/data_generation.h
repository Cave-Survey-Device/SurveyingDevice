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


// #define DISPERSED_GENERATION
#ifdef DISPERSED_GENERATION
#define N_SAMPLES 100
#define N_Y 10
#define N_Z N_SAMPLES/N_Y
#else
#define N_SAMPLES 12*5
#endif

#define INCLINATION_OFFSET 5 * M_PI/180
#define HEADING_OFFSET -10 * M_PI/180
#define COMPOUND_OFFSET sqrt(HEADING_OFFSET*HEADING_OFFSET + INCLINATION_OFFSET*INCLINATION_OFFSET)

using namespace Eigen;

Vector3f Spherical(Vector3f cartesian){
    Vector3f spherical;
    spherical << atan2(cartesian(1), cartesian(0)),
            atan2(pow( pow(cartesian(0),2) + pow(cartesian(1),2), 0.5),cartesian(2)),
            cartesian.norm();
    return spherical;
}

MatrixXf generateTrueInertialAlignData(Vector3f true_vec)
{
#ifdef DISPERSED_GENERATION
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
            R = y_rotation(360.* y/N_Y) * z_rotation(360.* z/N_Z);
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

MatrixXf generateInertialAlignData(MatrixXf true_data, Matrix3f Tm, Vector3f hm)
{
    int n = true_data.cols();
    MatrixXf samples = MatrixXf::Zero(3,N_SAMPLES);
    std::default_random_engine generator;
    std::normal_distribution<float> distribution(0,0.01);
    Vector3f noise;
    for(int i=0;i<n;i++)
    {
        for(int j=0;j<(int)(N_SAMPLES/12);j++)
        {
            noise << distribution(generator),distribution(generator),distribution(generator);
            //std::cout << "i: " << i << "\n" << true_data.col(i) << "\n\n";
            samples.col(i*N_SAMPLES/12+j) = Tm * true_data.col(i) + hm + noise;
        }
    }
    return samples;
}


//MatrixXf generateLaserAlignData()
//{
//    // Define n-rotations
//    const int n_rots = 8;
//
//    // Define characteristics of laser
//    const int true_distance = 10;
//    const float disto_len = 0.1;
//
//    // Rotate disto
//    Matrix<float,4,n_rots> out;
//
//    std::default_random_engine generator;
//    std::normal_distribution<float> theta_noise_dist(0,0.0001);
//    std::normal_distribution<float> distance_noise_dist(0,0.0001);
//
//    // Get g vector of the device
//    float theta = 0;
//    float alpha, beta, gamma, distance;
//
//    alpha = COMPOUND_OFFSET;
//    gamma = asin((disto_len * sin(alpha)/true_distance));
//    beta = M_PI - gamma;
//    distance = (true_distance * sin(beta))/sin(alpha);
//
//    for (int i=0;i<n_rots;i++) {
//        theta = 2*M_PI / n_rots * i + theta_noise_dist(generator);
//        out.col(i) <<  x_rotation(Rad2Deg(theta)) * y_rotation(Rad2Deg(COMPOUND_OFFSET)) * Vector3f{0, 0, 1}, distance * (1+distance_noise_dist(generator));
//    }
//    return out;
//}

MatrixXf generateLaserAlignData()
{
    // Define n-rotations
    const int n_rots = 8;
    const Vector3f target{1,1,2};
    const float disto_len = 1;


    // Get disto tip locations
    float alpha, beta, gamma, theta, distance, target_len, measured_len, initial_disto_rotation, disto_err_rotation;

    // Calculate value for planar solution e.g. target x {0,1,0} plane
    target_len = target.norm();
    theta = COMPOUND_OFFSET;
    alpha = M_PI - theta;
    beta = asin(disto_len * sin(alpha)/target_len);

    gamma = theta-beta;

    cout << "theta: " << Rad2Deg(theta) << "\n";
    cout << "alpha: " << Rad2Deg(alpha) << "\n";
    cout << "beta: " << Rad2Deg(beta) << "\n";
    cout << "gamma: " << Rad2Deg(gamma) << "\n";


    Vector3f initial_disto_tip, disto_tip, disto_error_vec, initial_disto_plane;

    // Find location of disto tip on target X yaxis plane
    disto_error_vec = y_rotation(Rad2Deg(INCLINATION_OFFSET)) * z_rotation(Rad2Deg(HEADING_OFFSET)) * Vector3f{1,0,0};
    initial_disto_plane = target.cross(Vector3f{0,0,1});
    // Rotate vector about normal to plane
    initial_disto_tip = arbitrary_rotation(gamma, target/target.norm() * disto_len ,initial_disto_plane);

    // Find projection of error on YZ plane + projection of initial plane on XZ plane
    Vector3f laser_vec{1,0,0};
    laser_vec = y_rotation(Rad2Deg(-INCLINATION_OFFSET)) * z_rotation(Rad2Deg(HEADING_OFFSET)) * laser_vec;
    // atan2(y,-z) to calculate angle from -z vector
    cout << "laser vec: " << laser_vec << "\n";

    disto_err_rotation = atan2(laser_vec(1),laser_vec(2));
    cout << "Disto err rotation: " << Rad2Deg(disto_err_rotation) << "\n";

    // Rotate disto tip
    Matrix<float,3,n_rots> out_disto;
    Matrix<float,4,n_rots> out_hir;
    for (int i=0;i<n_rots;i++) {
        theta = 2*M_PI / n_rots * i;
        disto_tip = arbitrary_rotation(theta,initial_disto_tip,target);
        out_disto.col(i) = disto_tip;

        Vector3f spherical_disto = Spherical(out_disto.col(i));
        out_hir.col(i) << spherical_disto(0), spherical_disto(1), theta + disto_err_rotation, (out_disto.col(i) - target).norm();
//        atan2(disto_tip(1),disto_tip(0)),
//        atan2(disto_tip(2),disto_tip(0)*disto_tip(0) + disto_tip(1)*disto_tip(1)),
//        theta + disto_err_rotation,
//        (out_disto.col(i) - target).norm();

//        cout << "Disto tip location   x:" << out_disto.col(i)(0) << "  y: " << out_disto.col(i)(1) << "  z: " << out_disto.col(i)(2) << "\n";
//        cout << "Disto tip orientation    h:" << out_hir.col(i)(0) << "  i: " << out_hir.col(i)(1) << "  r: " << out_hir.col(i)(2) << "\n\n";
    }

    return out_hir;
}
#endif //MAGNETOMETER_CALIBRATION_DATA_GENERATION_H
