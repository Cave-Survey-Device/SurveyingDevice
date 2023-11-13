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

#define N_SAMPLES 12*20

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

    Vector3f x_ax(1,0,0);
    Vector3f y_ax(0,1,0);
    Vector3f z_ax(0,0,1);
    Vector3f x, y, z;

    for (int n=0; n<12; n++)
    {
        Rx = xRotation(DEG_TO_RAD*-x_rots(n));
        Ry = yRotation(DEG_TO_RAD*y_rots(n));
        Rz = zRotation(DEG_TO_RAD*-z_rots(n));
        R = Rz * Ry * Rx;
//        samples.col(n) = R * true_vec;
//
        x = R * x_ax;
        y = R * y_ax;
        z = R * z_ax;

        samples.col(n) << x.dot(true_vec), y.dot(true_vec), z.dot(true_vec);
        samples.col(n).normalize();
//        samples.col(n) = R * true_vec;
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
    int nx,ny,nz;
    int n = 0;

    Vector3f x_ax(1,0,0);
    Vector3f y_ax(0,1,0);
    Vector3f z_ax(0,0,1);
    Vector3f x, y, z;

    MatrixXf samples = MatrixXf::Zero(3,N_MAG_SAMPLES);
    Matrix3f R;

    for(nx=0;nx<N_X;nx++)
    {
        for(ny=0;ny<N_Y;ny++)
        {
            for(nz=0;nz<N_Z;nz++) {
                R = xRotation(2 * M_PI * nx / N_X) * yRotation(2 * M_PI * ny / N_Y) * zRotation(2 * M_PI * nz / N_Z);
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
    std::normal_distribution<float> distribution(0, 0.01);
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
    std::normal_distribution<float> distribution(0,0.01);
    Vector3f noise;
    for(int i=0;i<n;i++)
    {
        for(int j=0;j<(int)(N_SAMPLES/n);j++)
        {
            noise << distribution(generator),distribution(generator),distribution(generator);
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
MatrixXf generateLaserAlignData(const float inclination_offset, const float heading_offset,
                                const Matrix3f &Ta, const Vector3f &ha, const Matrix3f &Tm, const Vector3f &hm,
                                const Matrix3f &Ra, const Vector3f &ba, const Matrix3f &Rm, const Vector3f &bm,
                                const Matrix3f &Ralign, const float &inclination_angle)
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

   // Rotated according to the right-hand-rule
    laser_vec = zRotation( heading_offset) * yRotation(-inclination_offset) * laser_vec;

    // Find total error angle and initial roll (roll such that laser_vec in -ve z direction and XZ plane)
    combined_error = acos(laser_vec.dot(x_ax));
    initial_roll = -M_PI_2 - atan2(laser_vec(2),laser_vec(1));


    // ------------------------------------ Find initial position of device ------------------------------------
    float  target_len, alpha, beta, gamma, theta;
    Vector3f initial_disto_tip, target_vec;
    target_len = target.norm();
    target_vec = target/target_len;
    theta = combined_error;
    alpha = M_PI - theta;
    beta = asin(disto_len * sin(alpha)/target_len);
    gamma = theta-beta;

    // Generate initial disto tip by rotating into targex X z_ax plane
    Vector3f ax;
    ax = target_vec.cross(z_ax);
    ax = ax/ax.norm();
    initial_disto_tip = quatRot(ax,-gamma) * target/target.norm() * disto_len;


    // ------------------------------------ Rotate device about target axis ------------------------------------
    float phi, heading, inclination, roll;
    Vector3f disto_tip;
    Matrix<float,4,n_rots> out_hird;
    Vector3f x;
    Vector3f g, m;
    Matrix<float,2,3> gm;

    for (int i=0; i<n_rots; i++)
    {
        phi = i * 2*M_PI/n_rots;
        roll = initial_roll + phi;
        disto_tip = quatRot(target_vec,-phi) * initial_disto_tip;
        x = disto_tip.normalized();
//        laser_vec = target - disto_tip;

        gm = toInertial(disto_tip,roll);
        g = gm.row(0);
        m = gm.row(1);

//        cout << "g:" << g.reshaped(1,3) << "\n";
        cout << "Disto tip: " << x.reshaped(1,3) << "\n";
        cout << "Cartesian: " << inertialToCartesian(g,m).reshaped(1,3) << "\n";

        cout << "Cardan_orig: " << atan2(x(1),x(0)) << " " << asin(x(2)) << " " << roll << "\n";
        cout << "Cardan_calc: " << inertialToCardan(g,m).reshaped(1,3) << "\n\n";


        m = Ralign * Rm * ((Tm * m + hm) - bm);
        g = Ra * ((Ta * g + ha) - ba);
    }

    return out_hird;
}
#endif //MAGNETOMETER_CALIBRATION_DATA_GENERATION_H
