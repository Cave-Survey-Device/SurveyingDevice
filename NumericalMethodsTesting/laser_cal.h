//
// Created by chris on 26/07/2023.
//

#ifndef NUMERICALMETHODSTESTING_LASER_CAL_H
#define NUMERICALMETHODSTESTING_LASER_CAL_H
#include "util.h"

#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

Vector3f Cartesian(Vector3f spherical)
{
    Vector3f cartesian;
    // Serial.printf("Heading: %f   Inclination: %f\n", spherical(0), spherical(1));
    cartesian << spherical(2)*(sin(spherical(1))*cos(spherical(0))),
            spherical(2)*(sin(spherical(1))*sin(spherical(0))),
            spherical(2)*(cos(spherical(1)));
    return cartesian;
}

Vector3f Spherical(Vector3f cartesian){
    Vector3f spherical;
    spherical << atan2(cartesian(1), cartesian(0)),
            atan2(pow( pow(cartesian(0),2) + pow(cartesian(1),2), 0.5),cartesian(2)),
            cartesian.norm();
    return spherical;
}


Vector3f NormalVec(const MatrixXf &point_cloud){
    Vector3f normal;
    MatrixXf left_singular_mat;
    // int U_cols;

    // Subtract mean from each point otherwise its wrong XD
    // https://www.ltu.se/cms_fs/1.51590!/svd-fitting.pdf
    Vector3f point_vec = point_vec.colwise()-point_vec.rowwise().mean();

    JacobiSVD<MatrixXf> svd(point_vec, ComputeThinU | ComputeThinV);
    left_singular_mat = svd.matrixU();
    // U_cols = left_singular_mat.cols();
    // 3rd col of U contains normal vec
    normal << left_singular_mat(0,2), left_singular_mat(1,2), left_singular_mat(2,2);

    return normal;
};


// Takes Heading, Incination, Distance
const int N_LASER_CAL = 8;
const float DISTO_LEN = 0.1;
Vector2f align(MatrixXf laser_alignment_data)
{
    int calib_num = 0;
    Vector3f mean_calibration_data; // Heading, Inclination, roll, distance
    Vector3f misalignement_mean; // Mean misalignement vector
    Vector3f conversion_dummy; // Dummy variable for cartesian <=> spherical conversions
    Vector3f target_vector; // Vector direction to target
    Matrix<float,3,N_LASER_CAL> misalignement_mat; // Matrix to hold each calculated misalignement vector
    Matrix<float,3,N_LASER_CAL> cartesian_calibration_data; // Matrix to hold cartesian versions of calibration data
    Matrix3f rotation_matrix; // Rotation matrix for reversing the effect of tilt
    float target_distance; // Distance of target from origin
    float x, y, z; // Cartesian coordinates for target
    float xi, yi, zi; // Cartesian coordinated for tip of disto

    // Mean of all calibration data collected
    mean_calibration_data = laser_alignment_data.rowwise().mean();


    /*************************************************************
     * 1. Find the locations of the tip of the disto
     * 2. Find the plane which best fits those points
     * 3. Find the normal vector to this
     * 4. Multiply by target distance to get location of target
     * 5. Convert to cartesian coordinates of target point
     *************************************************************/
    // Calculate cartesian coordinates for disto tip in each calibration shot
    for (calib_num=0; calib_num<N_LASER_CAL; calib_num++)
    {
        conversion_dummy << laser_alignment_data(0,calib_num), laser_alignment_data(1,calib_num), DISTO_LEN;
        cartesian_calibration_data.col(calib_num) << Cartesian(conversion_dummy);
    }

    // Calculate target distance
    target_distance = pow(pow(DISTO_LEN,2) + pow(mean_calibration_data(3),2) ,0.5); // Pythagoras on disto len and splay len
    // get normal to best fit plane and multiply by distance
    target_vector = NormalVec(cartesian_calibration_data) * target_distance;

    // Not working :( IDK why
    if (cartesian_calibration_data(0,0) * target_vector(0) < 0)
    {
        target_vector = -target_vector;
    }

    // Convert to easier variables
    x = target_vector(0);
    y = target_vector(1);
    z = target_vector(2);


    /*************************************************************************************
     * 1. Get the cartesian location of the tip of the disto for each shot
     * 2. Find the rotation matrix corresponding to the roll
     * 3. Calculate the vector between the target and disto tip
     * 4. Rotate this vector by the matrix to reverse the effects of tilt on the result
     *************************************************************************************/
    float roll;
    for (calib_num=0; calib_num<N_LASER_CAL; calib_num++)
    {
        xi = cartesian_calibration_data(0,calib_num);
        yi = cartesian_calibration_data(1,calib_num);
        zi = cartesian_calibration_data(2,calib_num);

        // Rotation matrix about x depending on device roll
        roll = laser_alignment_data(2,calib_num);
        rotation_matrix = x_rotation(-RAD_TO_DEG * roll);

        // Calculalate vector between tip of disto and actual target
        misalignement_mat.col(calib_num) << x-xi, y-yi, z-zi;
        // Reverse the effects of tilt on the misalignement vector
        misalignement_mat.col(calib_num) = rotation_matrix * misalignement_mat.col(calib_num);
        //Serial.printf("Misalignment vector: X: %f, Y: %f, Z: %f\n",misalignement_mat(0,calib_num),misalignement_mat(1,calib_num),misalignement_mat(2,calib_num));
    }


    /****************************************************
     * 1. Find the mean misalignment matrix
     * 2. Convert this to spherical coordinates and save
     ****************************************************/
    // TODO: Maybe search for and remove erroneous values?
    misalignement_mean = misalignement_mat.rowwise().mean();

    // Repurposing x,y,z
    x = misalignement_mean(0);
    y = misalignement_mat(1);
    z = misalignement_mat(2);


    // get heading and inclination corrections by converting back to polar coordinates
    float laser_inclination_alignment = atan2(y,x);
    float laser_heading_alignment = asin(z/pow(pow(x,2) + pow(y,2) + pow(z,2),0.5));

    Vector2f out;
    out << laser_inclination_alignment, laser_heading_alignment;
    return out;
}


#endif //NUMERICALMETHODSTESTING_LASER_CAL_H
