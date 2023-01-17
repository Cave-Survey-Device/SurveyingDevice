#include "sensorhandler.h"

Vector3d calc_normal_vec(MatrixXd point_vec, bool debug /*= false*/){
  char buffer[150];
  Vector3d normal;
  MatrixXd left_singular_mat;
  int U_cols;

  // Subtract mean from each point otherwise its wrong XD
  // https://www.ltu.se/cms_fs/1.51590!/svd-fitting.pdf
  point_vec = point_vec.colwise()-point_vec.rowwise().mean();

  JacobiSVD<MatrixXd> svd(point_vec, ComputeThinU | ComputeThinV);
  left_singular_mat = svd.matrixU();
  U_cols = left_singular_mat.cols();
  // 3rd col of U contains normal vec
  normal << left_singular_mat(0,2), left_singular_mat(1,2), left_singular_mat(2,2);

  if (debug){
    sprintf(buffer, "U: ", svd.matrixU(), "Sigma: ", svd.singularValues(), "\n");
    Serial.println(buffer);
    sprintf(buffer, "Normal Vector: \nX: %f \nY: %f \nZ: %f\n", normal[0], normal[1], normal[2]);
    Serial.printf(buffer);
  }

  return normal;
};

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

    Serial.print("Cartesian coordinates of disto tip: ");
    Serial.printf("X: %f", DISTO_LEN*sin(M_PI_2 - inclination)*cos(-heading));
    Serial.printf("   Y: %f", DISTO_LEN*sin(M_PI_2 - inclination)*sin(-heading));
    Serial.printf("   Z: %f\n", DISTO_LEN*cos(M_PI_2 - inclination));

    char str_buf[60];
    sprintf(str_buf,"Raw device data H: %f, I: %f, D: %f", RAD_TO_DEG*heading, RAD_TO_DEG*inclination, distance);
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

    inclination = inclination;
    roll = roll;
    heading =  atan2(vector_north(1), vector_north(0));
    if (grav_data(2) < 0)
    {
        heading = heading * -1;
    }
}

// Outputs data in degrees
Vector3d SensorHandler::get_shot_data()
{
    Vector3d out;
    double x;
    double y;
    double roll_correct_heading_correction = 0;
    double roll_correct_inclination_correction = 0;

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
    if (calibration_num < 8) // Do 8 times
    {
        Serial.printf("Got calibration %i!\n",calibration_num);
        
        // Add data to the calibration data store
        device_calibration_data.col(calibration_num) << heading, inclination, roll, distance;
        calibration_num++;

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
    Vector4d mean_calibration_data;
    // Mean misalignement vector
    Vector3d misalignement_mean;
    // Vector direction to target
    Vector3d target_vector;
    // Matrix to hold each calculated misalignement vector
    Matrix<double,3,CALIBRATION_SIZE> misalignement_mat;
    // Matrix to hold cartesian versions of calibration data
    Matrix<double,3,CALIBRATION_SIZE> cartesian_calibration_data;
    // Rotation matrix for reversing the effect of tilt
    Matrix3d rotation_matrix;

    // Distance of target from origin
    double target_distance;
    // Cartesian coordinates for target
    double x, y, z;
    // Cartesian coordinated for tip of disto
    double xi, yi, zi;

    // Mean of all calibration data collected
    mean_calibration_data = device_calibration_data.rowwise().mean();


    /*************************************************************
     * 1. Find the locations of the tip of the disto
     * 2. find the plane which best fits those points
     * 3. find the normal vector to this
     * 4. multiply by target distance to get location of target
     * 5. Convert to cartesian coordinates of target point
     *************************************************************/
    char str_buf[60];
    for (calib_num=0; calib_num<CALIBRATION_SIZE; calib_num++)
    {
        cartesian_calibration_data.col(calib_num) << 
        DISTO_LEN*sin(M_PI_2 - device_calibration_data(1,calib_num))*cos(-device_calibration_data(0,calib_num)),
        DISTO_LEN*sin(M_PI_2 - device_calibration_data(1,calib_num))*sin(-device_calibration_data(0,calib_num)),
        DISTO_LEN*cos(M_PI_2 - device_calibration_data(1,calib_num));
    }

    // Calculate target distance
    // Maybe add target vec directio nreversing if getting issues wit direction?
    target_distance = pow(pow(DISTO_LEN,2) + pow(mean_calibration_data(3),2) ,0.5); // Pythagoras on disto len and splay len
    target_vector = calc_normal_vec(cartesian_calibration_data) * target_distance;
    x = target_vector(0);
    y = target_vector(1);
    z = target_vector(2);

    if (cartesian_calibration_data.col(0)(0) * target_vector(0) < 0)
    {
        target_vector = -target_vector;
    }

    Serial.printf("Target coordinates X: %f, Y: %f, Z: %f\n",x,y,z);
    

    /*************************************************************************************
     * 1. Get the cartesian location of the tip of the disto for each shot
     * 2. Find the rotation matrix corresponding to the roll
     * 3. Calculate the vector between the target and disto tip
     * 4. Rotate this vector by the matrix to revers the effects of tilt on the result
     *************************************************************************************/
    // Calculate cartesian coordinates for disto tip in each calibration shot
    for (calib_num=0; calib_num<CALIBRATION_SIZE; calib_num++)
    {
        xi = cartesian_calibration_data(0,calib_num);
        yi = cartesian_calibration_data(1,calib_num);
        zi = cartesian_calibration_data(2,calib_num);
        Serial.printf("Disto tip coordinates X: %f, Y: %f, Z: %f\n",xi,yi,zi);

        // Rotation matrix about x depending on device roll
        roll = device_calibration_data(2,calib_num);
        rotation_matrix << 1,0,0,
                           0,cos(roll),sin(roll),
                           0,sin(roll),cos(roll);

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