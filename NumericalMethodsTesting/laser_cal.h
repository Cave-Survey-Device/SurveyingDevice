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
    cartesian <<    spherical(2) * (sin(spherical(1)) * cos(spherical(0))),
                    spherical(2) * (sin(spherical(1)) * sin(spherical(0))),
                    spherical(2) * (cos(spherical(1)));
    return cartesian;
}

Vector3f NormalVec(const MatrixXf &point_cloud){
    Vector3f normal;
    MatrixXf left_singular_mat;
    // int U_cols;

    // Subtract mean from each point otherwise its wrong XD
    // https://www.ltu.se/cms_fs/1.51590!/svd-fitting.pdf
    MatrixXf mean_adj_point_cloud = point_cloud;
    mean_adj_point_cloud = mean_adj_point_cloud.colwise()-mean_adj_point_cloud.rowwise().mean();

    JacobiSVD<MatrixXf> svd(mean_adj_point_cloud, ComputeThinU | ComputeThinV);
    left_singular_mat = svd.matrixU();
    // U_cols = left_singular_mat.cols();
    // 3rd col of U contains normal vec
    normal << left_singular_mat(0,2), left_singular_mat(1,2), left_singular_mat(2,2);

    return normal;
};


// Takes Heading, Incination, Roll, Distance
const int N_LASER_CAL = 8;
const float DISTO_LEN = 0.1;
Vector2f align_laser(Matrix<float,4,-1> laser_alignment_data)
{
    int calib_num = 0;
    Vector4f mean_calibration_data; // Heading, Inclination, roll, distance
    Vector3f misalignement_mean; // Mean misalignement vector
    Vector3f conversion_dummy; // Dummy variable for cartesian <=> spherical conversions
    Matrix<float,3,N_LASER_CAL> misalignement_mat; // Matrix to hold each calculated misalignement vector
    Matrix<float,3,N_LASER_CAL> cartesian_calibration_data; // Matrix to hold cartesian versions of calibration data
    Matrix3f rotation_matrix; // Rotation matrix for reversing the effect of tilt

    // Mean of all calibration data collected
    mean_calibration_data = laser_alignment_data.rowwise().mean();


    /*************************************************************
     * 1. Find the locations of the tip of the disto
     * 2. Find the plane which best fits those points
     * 3. Find the normal vector to this
     * 4. Calculate averge angle of disto from target
     * 5. Calculate target distance
     * 6. Find location of target
     *************************************************************/
    // Calculate cartesian coordinates for disto tip in each calibration shot
    for (calib_num=0; calib_num<N_LASER_CAL; calib_num++)
    {
        conversion_dummy << laser_alignment_data(0,calib_num), laser_alignment_data(1,calib_num), DISTO_LEN;
        cartesian_calibration_data.col(calib_num) << Cartesian(conversion_dummy);
    }

    // Calculate normal to plane
    Vector3f target_vector;
    target_vector = NormalVec(cartesian_calibration_data);
    target_vector = target_vector/target_vector.norm();
    if (target_vector.dot(cartesian_calibration_data.col(0)) < 0)
    {
        target_vector = -target_vector;
    }

    // Calculate averge angle of disto from target
    float avg_gamma = 0;
    for (calib_num=0; calib_num<N_LASER_CAL; calib_num++)
    {
        avg_gamma += acos(
                cartesian_calibration_data.col(calib_num).dot(target_vector) / cartesian_calibration_data.col(calib_num).norm()
                );
    }
    avg_gamma = avg_gamma/calib_num;

    // Calculate target distance
    float alpha, beta, gamma, l, target_distance;
    l = mean_calibration_data(3);
    cout << "l: " << l << "\n";
    gamma   = avg_gamma;
    beta    = asin(DISTO_LEN/l * sin(gamma));
    alpha   = M_PI - gamma - beta;
    target_distance = l * sin(alpha)/sin(gamma);
    cout << "target distance: " << target_distance << "\n";

    cout << "theta: "   << RAD_TO_DEG*(M_PI - alpha) << "\n";
    cout << "alpha: "   << RAD_TO_DEG*alpha << "\n";
    cout << "beta: "    << RAD_TO_DEG*beta << "\n";
    cout << "gamma: "   << RAD_TO_DEG*gamma << "\n";


    // Find location of target
    Vector3f target_location;
    target_location = target_distance * target_vector * 1/target_vector.norm();

    cout << "Target vector: " << target_location(0) << " " << target_location(1) << " " << target_location(2) << "\n\n";


    /*************************************************************************************
     * 1. Get the cartesian location of the tip of the disto for each shot
     * 2. Rotate each disto tip about the target vector by its roll
     * 3. Rotate target vec about {1,0,0} by -disto roll
     * 4. Find heading and inclination of this new vector
     *************************************************************************************/
    float roll;
    float laser_inclination_alignment = 0;
    float laser_heading_alignment = 0;
    float mean_angle = 0;
    Vector3f rotated_disto_tip;
    Vector3f current_cal;
    for (calib_num=0; calib_num<N_LASER_CAL; calib_num++)
    {
        roll = laser_alignment_data.col(calib_num)(2);

        // Rotate all disto tips is=nto same location
        rotated_disto_tip = cartesian_calibration_data.col(calib_num);
        rotated_disto_tip = quatRot(target_vector,roll) * rotated_disto_tip;
         cout << "Roll: " << Rad2Deg(roll) << "\t Disto tip: " << rotated_disto_tip(0) << "  " << rotated_disto_tip(1) << " " << rotated_disto_tip(2) << "\n";


        // Find angle between this vec and the projected z axis
        Vector3f x_ax_xfrm = target_vector;
        Vector3f y_ax_xfrm = target_vector.cross(Vector3f(0,0,1));
        Vector3f z_ax_xfrm = x_ax_xfrm.cross(y_ax_xfrm);


        Vector3f ref_point = target_vector * DISTO_LEN * cos(gamma);

        Vector3f disto_target_vec = rotated_disto_tip - ref_point;
        disto_target_vec = disto_target_vec / disto_target_vec.norm();

        Vector3f ref_vec = z_ax_xfrm / z_ax_xfrm.norm();

        float angle = disto_target_vec.dot(ref_vec);
        angle = angle/(disto_target_vec.norm() * ref_vec.norm());
        angle = acos(angle);
        mean_angle += angle;
    }
    mean_angle = mean_angle/N_LASER_CAL;
    cout << "mena angle: " << Rad2Deg(mean_angle) << "\n";
    Vector3f disto_tip = Vector3f(DISTO_LEN,0,0);
    Vector3f target = quatRot(Vector3f(0,1,0),gamma) * Vector3f(target_distance,0,0);
    Vector3f laser_vec = quatRot(Vector3f(1,0,0),-mean_angle) * (target - disto_tip);
    laser_vec = laser_vec/laser_vec.norm();
    cout << "New laser vec: " << laser_vec(0) << " " << laser_vec(1) << " " << laser_vec(2) << "\n";

    cout << "Heading and Inclination error: ";
    cout << Rad2Deg(atan2(laser_vec(2),laser_vec(0))) << "  ";
    cout << Rad2Deg(atan2(laser_vec(1),laser_vec(0))) << "\n";

    Vector2f out;
    out << laser_inclination_alignment, laser_heading_alignment;
    return out;
}


#endif //NUMERICALMETHODSTESTING_LASER_CAL_H
