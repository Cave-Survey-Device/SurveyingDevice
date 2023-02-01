#include "accelerometer.h"

Accelerometer::Accelerometer(struct bno055_gravity *myGravityData)
{
    correction_transformation << 1, 0, 0,
                                 0, 1, 0,
                                 0, 0, 1;
};

void Accelerometer::update(){
    get_raw_data();
    corrected_gravity_data = raw_gravity_data*correction_transformation;
}

Vector3d Accelerometer::get_grav_vec()
{
    return corrected_gravity_data;
};


void Accelerometer::calibrate()
{
    
}