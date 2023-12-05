#ifndef NUMERICAL_METHODS_LASER_ALIGNMENT_H
#define NUMERICAL_METHODS_LASER_ALIGNMENT_H

#include "utils.h"
#include "FittingFuncs.h"

namespace NumericalMethods{
    
// Takes Heading, Incination, Roll, Distance
/**
 * @brief 
 * 
 * @param laser_alignment_data 
 * @return Vector2f 
 */
Vector2f alignLaser(const Matrix<float,4,N_LASER_CAL> &laser_alignment_data)
{
    int calib_num = 0;
    Vector4f mean_calibration_data; // Heading, Inclination, roll, distance
    Vector3f misalignement_mean; // Mean misalignement vector
    static Matrix<float,3,N_LASER_CAL> cartesian_calibration_data; // Matrix to hold cartesian versions of calibration data
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
        cartesian_calibration_data.col(calib_num) << cardanToCartesian(laser_alignment_data.col(calib_num).segment(0,3));
    }

    // Calculate normal to plane
    Vector3f target_vector;
    target_vector = normalVec(cartesian_calibration_data);
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
    gamma   = avg_gamma;
    beta    = asin(DEVICE_LENGTH/l * sin(gamma));
    alpha   = M_PI - gamma - beta;
    target_distance = l * sin(alpha)/sin(gamma);



    // Find location of target
    Vector3f target_location;
    target_location = target_distance * target_vector * 1/target_vector.norm();


    /*************************************************************************************
     * 1. Get the cartesian location of the tip of the disto for each shot
     * 2. Rotate each disto tip about the target vector by its roll to find an average disto position
     * 3. Find the the Z axis of the frame where X = Vtarget, Y = Vtarget X (0,0,-1)
     * 4. Find the vector from Psaple,avg t0 Vtarget
     * 5. Find the angle between this and the new calculated Z axis
     * 6. Find new_target by yRoatation(gamma) * (target_len,0,0), new_disto by (disto_len,0,0)
     * 7. Find laser vec by xRotation(angle in 5.) * (new_target-disto_tip)
     *************************************************************************************/
    float roll;
    float angle; // Angle between target-z-axis and the vector between the disto and the target vector
    float mean_angle = 0;
    Vector3f rotated_disto_vec, disto_target_vec, ref_point;
    Vector3f x_ax_xfrm, y_ax_xfrm, z_ax_xfrm;
    Vector3f target, laser_vec, laser_cardan;

    for (calib_num=0; calib_num<N_LASER_CAL; calib_num++)
    {
        roll = laser_alignment_data.col(calib_num)(2);

        // Rotate all disto tips into the same location
        rotated_disto_vec = cartesian_calibration_data.col(calib_num);
        rotated_disto_vec = quatRot(target_vector,-roll) * rotated_disto_vec;


        // Find angle between this vec and the projected z axis
        x_ax_xfrm = target_vector;
        y_ax_xfrm = target_vector.cross(Vector3f(0,0,-1));
        z_ax_xfrm = x_ax_xfrm.cross(y_ax_xfrm);
        x_ax_xfrm.normalize();
        y_ax_xfrm.normalize();
        z_ax_xfrm.normalize();

        // Find point closest to disto_vector on target_vector
        ref_point = target_vector * cos(gamma);

        // Find vector between the above point and the disto_vector
        disto_target_vec = rotated_disto_vec - ref_point;
        disto_target_vec = disto_target_vec / disto_target_vec.norm();

        // Find the angle between the above vector and the target-z-axis
        angle = disto_target_vec.dot(z_ax_xfrm);
        angle = angle/(disto_target_vec.norm() * z_ax_xfrm.norm());
        angle = acos(angle);
        mean_angle += angle;
    }
    mean_angle = mean_angle/N_LASER_CAL;
    
    // Add directionality to the angle
    if (disto_target_vec.dot(y_ax_xfrm) > 0)
    {
        mean_angle = abs(mean_angle);
    } else {
        mean_angle = abs(mean_angle) * -1;
    }



    target = quatRot(Vector3f(0,1,0),gamma) * Vector3f(target_distance,0,0);
    laser_vec = quatRot(Vector3f(1,0,0),mean_angle - 3*M_PI_2) * (target - Vector3f(DEVICE_LENGTH,0,0));
    laser_vec = laser_vec/laser_vec.norm();


    Matrix<float,3,2> laser_inertial = toInertial(laser_vec,0);
    laser_cardan = inertialToCardan(laser_inertial.col(0),laser_inertial.col(1));

    Vector2f out;
    out << -laser_cardan(0), laser_cardan(1);
    return out;
}

}

#endif