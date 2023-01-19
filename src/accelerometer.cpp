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
    corrected_gravity_data = raw_gravity_data; // *correction_transformation;
};

Vector3d Accelerometer::get_grav_vec()
{
    return corrected_gravity_data;
};


void Accelerometer::calibrate()
{
    
}