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
    // Find angle to z-axis
    Vector3d z_axis;
    double dot_prod;
    double scaling;
    char str[50];

    z_axis << 0,0,1;
    dot_prod = corrected_gravity_data.dot(z_axis);
    // Serial.printf("dot_prod: %f\n",dot_prod);
    scaling = z_axis.norm() * corrected_gravity_data.norm();
    // Serial.printf("scaling: %f\n",scaling);

    sprintf(str,"Got inclination: %f", RAD_TO_DEG * acos(dot_prod/scaling));
    debug(DEBUG_ACCEL,str);
	return RAD_TO_DEG * acos(dot_prod/scaling);
}