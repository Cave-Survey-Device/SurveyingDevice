#include "accelerometer.h"

Accelerometer::Accelerometer(struct bno055_gravity *myGravityData)
{
    sensor_connection = myGravityData;
    correction_transformation << 1, 0, 0,
                                 0, 1, 0,
                                 0, 0, 1;
};

void Accelerometer::update()
{
    bno055_read_gravity_xyz(sensor_connection);
    raw_gravity_data << sensor_connection->x, sensor_connection->y, sensor_connection->z;
    corrected_gravity_data = correction_transformation * raw_gravity_data;
};

Vector3d Accelerometer::get_gravity_unit_vec()
{
    return corrected_gravity_data;
};

double Accelerometer::get_inclination()
{
    // Find rotation about y-axis
    Matrix3d I;
    Vector3d y_axis;
    Matrix3d vx;
    Matrix3d rotation_mat;
    Vector3d cross_prod;
    double c;

    y_axis << 0,1,0;
    cross_prod = corrected_gravity_data.cross(y_axis);
    c = corrected_gravity_data.dot(y_axis);

    vx <<              0, -cross_prod[2],  cross_prod[1],
           cross_prod[2],              0, -cross_prod[0],
          -cross_prod[1],  cross_prod[0],              0;

    I << 1,0,0,
        0,1,0,
        0,0,1;


    rotation_mat = I + vx + vx*vx * (1/(1+c));
	return atan2(-rotation_mat(2,0), sqrt(rotation_mat(2,1)*rotation_mat(2,1) + rotation_mat(2,2)*rotation_mat(2,2)));

}