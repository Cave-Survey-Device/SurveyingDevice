#include "sensorhandler.h"

void SensorHandler::update()
{
    Matrix<double,3,SAMPLING_SIZE> accel_samples;
    Matrix<double,3,SAMPLING_SIZE> mag_samples;
    int sample_num;

    // get an average of SAMPLING_SIZE samples for accuracy
    for (sample_num=0;sample_num<SAMPLING_SIZE;sample_num++)
    {
        accel_sensor->update();
        mag_sensor->update();
        accel_samples.col(sample_num) = accel_sensor->get_grav_vec();
        mag_samples.col(sample_num) = mag_sensor->get_mag_vec();
    }

    // Update SensorHandler's data storage
    mag_data = mag_samples.rowwise().mean();
    grav_data = accel_samples.rowwise().mean();

    // Get lidar distance measurement
    distance = lidar_sensor->get_measurement();

    // Convert data to orientation
    this->get_orientation();

    char str_buf[60];
    sprintf(str_buf,"Raw device data H: %f, I: %f, D: %f", heading, inclination, distance);
    debug(DEBUG_SENSOR,str_buf);
}

// Returns heading and inclination
// https://arduino.stackexchange.com/a/88707
void SensorHandler::get_orientation()
{    
    // https://www.analog.com/en/app-notes/an-1057.html equation (11)
	inclination =  atan2(grav_data(0),pow(pow(grav_data(1),2) + pow(grav_data(2),2),0.5));
    roll = atan2(grav_data(1),pow(pow(grav_data(0),2) + pow(grav_data(2),2),0.5));

    Vector3d vector_north = mag_data - ((mag_data.dot(grav_data) / grav_data.dot(grav_data)) * grav_data);

    inclination = RAD_TO_DEG * inclination;
    roll = RAD_TO_DEG * roll;
    heading =  RAD_TO_DEG * atan2(vector_north(1), vector_north(0));
    if (grav_data(2) < 0)
    {
        heading = heading * -1;
    }
}

// Vector3d SensorHandler::get_shot_data()
// {
//     Vector3d out;
//     Vector3d vector_north;
//     Vector3d total_vec;
//     Vector2d orientation;
    
//     // Make sure update has been called recently when getting a shot!

//     // 1. Find the vector of the device Vd
//     // Turn heading and inclination into rotation matrices
//     // Apply both rotations to [1,0,0]
//     // Multiply by DEVICE_LEN

//     // Heading rotation (rotation of theta in the anti-clockwise direction about z axis)
//     device_vec << 1, 0, 0;
//     Matrix3d heading_rotation;
//     heading_rotation << cos(heading), -sin(heading), 0,
//                         sin(heading), cos(heading) , 0,
//                         0           , 0            , 1;
//     // Inclination rotation (rotation in the anti-clockwise direction about the y axis)
//     Matrix3d inclination_rotation;
//     inclination_rotation << cos(inclination) , 0, sin(inclination),
//                             0                , 1,                0,
//                             -sin(inclination), 0, cos(inclination);
//     // Apply rotation matrices
//     device_vec = inclination_rotation * heading_rotation * device_vec;


//     // 2. Vt (total vector) = Vd (device vector) + Tv (true Vector)
//     device_vec = device_vec.array() * DISTO_LEN;
//     total_vec = device_vec.array() + (device_vec.array() + calibration_vector.array()) * distance;


//     // 3. Generate output vector heading, inclination
//     out << RAD_TO_DEG * asin(total_vec(1)/total_vec.norm()), RAD_TO_DEG * asin(total_vec(0)/total_vec.norm()), distance;
//     return out;
// }

Vector3d SensorHandler::get_shot_data()
{
    Vector3d out;
    double x;
    double y;
    double roll_correct_heading_correction = 0;
    double roll_correct_inclination_correction = 0;

    // Convert data to radians for maths to come
    heading = DEG_TO_RAD * heading;
    inclination = DEG_TO_RAD * inclination;

    if (roll > 180)
    {
        roll_correct_heading_correction = -heading_correction;
        roll_correct_inclination_correction = inclination_correction;
    }

    // heading needs to know whether its it's facing forwards or backwards. To preserve this:
    // If 90 < heading < 180: x = -x
    x = DISTO_LEN * cos(heading);
    x += distance * cos(heading+roll_correct_heading_correction);

    y = DISTO_LEN * sin(heading);
    y += distance * sin(heading+roll_correct_heading_correction);
    out(0) = RAD_TO_DEG *  atan2(y,x);

    // Can just use sin and cose as inclination is symmetric about z axis
    x = DISTO_LEN * cos(inclination);
    x += distance * cos(inclination+roll_correct_inclination_correction);
    y = DISTO_LEN * sin(inclination);
    y += distance * sin(inclination+roll_correct_inclination_correction);
    out(1) = RAD_TO_DEG * atan2(y,x);
    out(2) = pow(pow(DISTO_LEN,2) + pow(distance,2) ,0.5);

    // Convert data back to degrees
    heading = RAD_TO_DEG * heading;
    inclination = RAD_TO_DEG * inclination;

    return out;
}

SensorHandler::SensorHandler(Accelerometer* accel, Magnetometer* mag, Lidar* lidar)
{
    accel_sensor = accel;
    mag_sensor = mag;
    lidar_sensor = lidar;
    inclination_correction = 0;
    heading_correction = 0;
    calibration_num = 0;
    calibration_vector << 0, 0, 0;

    // Add code to load/save sensor calibration data from file
}

bool SensorHandler::calibrate()
{
    if (calibration_num < 7) // Do 8 times
    {
        Serial.printf("Got calibration %i!\n",calibration_num);
        calibration_num++;
        this->update();

        // Add data to the calibration data store
        device_calibration_data.col(calibration_num) << heading, inclination, roll, distance;

        return false;
    } else {
        // Once enough data is collected, attempt to align the laser
        this->align_laser();        
        return true;
    }
}

void SensorHandler::align_laser()
{
    int calib_num;
    // Heading, Inclination, roll, distance
    Vector4d mean_data;
    // Mean misalignement vector
    Vector3d misalignement_mean;
    // Matrix to hold each calculated misalignement vector
    Matrix<double,3,CALIBRATION_SIZE> misalignement_mat;
    // Rotation matrix for reversing the effect of tilt
    Matrix3d rotation_matrix;

    // Distance of target from origin
    double target_distance;
    // Cartesian coordinates for target
    double x, y, z;
    // Cartesian coordinated for tip of disto
    double xi, yi, zi;

    // Mean of all calibration data collected
    mean_data = device_calibration_data.rowwise().mean();

    // Convert to cartesian coordinates of target point
    target_distance = pow(pow(DISTO_LEN,2) + pow(mean_data(3),2) ,0.5); // Pythagoras on disto len and splay len
    x = target_distance*cos(mean_data(1))*cos(mean_data(0));
    y = target_distance*cos(mean_data(1))*sin(mean_data(0));
    z = target_distance*sin(mean_data(1));

    Serial.printf("Target distance: %f\n Disto tip coordinates: X: %f, Y: %f, Z: %f\n",target_distance,x,y,z);

    // Calculate cartesian coordinates for disto tip in each calibration shot
    for (calib_num=0; calib_num<CALIBRATION_SIZE; calib_num++)
    {
        xi = target_distance*cos(mean_data(1))*cos(mean_data(0));
        yi = target_distance*cos(mean_data(1))*sin(mean_data(0));
        zi = target_distance*sin(mean_data(1));

        // Rotation matrix about x depending on device roll
        rotation_matrix << 1,0,0,
                           0,cos(roll),sin(roll),
                           0,sin(roll),cos(roll);

        // Calculalate vector between tip of disto and actual target
        misalignement_mat.col(calib_num) << x-xi, y-yi, z-zi;
        // Reverse the effects of tilt on the misalignement vector
        misalignement_mat.col(calib_num) = rotation_matrix * misalignement_mat.col(calib_num);
        Serial.printf("Misalignment vector: X: %f, Y: %f, Z: %f\n",misalignement_mat(0),misalignement_mat(1),misalignement_mat(2));
    }

    // TODO: Maybe search for and remove erroneous values?
    misalignement_mean = misalignement_mat.rowwise().mean();

    // Repurposing x,y,z
    x = misalignement_mean(0);
    y = misalignement_mat(1);
    z = misalignement_mat(2);
    Serial.printf("Mean misalignment vector: X: %f, Y: %f, Z: %f\n",x,y,z);


    // Get heading and inclination corrections by converting back to polar coordinates
    inclination_correction = atan2(y,x);
    heading_correction = asin(z/pow(pow(x,2) + pow(y,2) + pow(z,2),0.5));
    Serial.printf("Spherical coordinates for misalignment vector: Heading: %f, Inclination: %f\n",inclination_correction,heading_correction);

}

double SensorHandler::get_inclination()
{
    return inclination + inclination_correction;
}
double SensorHandler::get_heading()
{
    return heading + heading_correction;
}
double SensorHandler::get_distance()
{
    return distance;
}