#include "sensors/Sensors.h"
#include "utils/NumericalMethods.h"
#include "utils/utility.h"
#include <queue>

void InertialSensor::CalibrateLinear()
{
    RowVector<float,10> U;
    Matrix3f M;
    Vector3f n;
    float d;

    U = fit_ellipsoid(calibration_data);
    M << U[0], U[5], U[4], U[5], U[1], U[3], U[4], U[3], U[2];
    n << U[6], U[7], U[8];
    d = U[9];
    Vector<float, 12> mag_transformation = calculate_ellipsoid_transformation(M, n, d);

    calibration_matrix << mag_transformation[0], mag_transformation[1], mag_transformation[2], mag_transformation[3], mag_transformation[4], mag_transformation[5], mag_transformation[6], mag_transformation[7], mag_transformation[8];
    calibration_offset << mag_transformation[9], mag_transformation[10], mag_transformation[11];
}

Vector3f InertialSensor::GetReading()
{
    Vector3f reading;
    reading.setZero();
    for (int i=0;i<SAMPLES_PER_READING;i++)
    {
        reading += GetRawData();
    }
    reading = reading/SAMPLES_PER_READING;
    
    return calibration_matrix * (reading - calibration_offset);
}

bool InertialSensor::ColectCalibrationSample()
{
    calibration_data.col(calib_num) = GetRawData();
    calib_num++;
    if (calib_num == N_CALIB)
    {
        return 1;
    }
    return 0;
}

void InertialSensor::ResetCalibration()
{
    calib_num = 0;
    calibrated_data = VectorXf::Zero(3,N_CALIB);
}


SensorHandler::SensorHandler(InertialSensor* acc, InertialSensor* mag, LaserSensor* las)
{
    magnetometer = mag;
    accelerometer = acc;
    laser = las;
}

SensorHandler::SensorHandler(InertialSensor* acc, InertialSensor* mag)
{
    magnetometer = mag;
    accelerometer = acc;
}

SensorHandler::SensorHandler(){}

Vector3f SensorHandler::GetReading()
{
    Vector3f reading;
    Vector3f mag_data = magnetometer->GetReading();
    Vector3f accel_data = accelerometer->GetReading();
    float laser_data = laser->GetMeasurement();    

    reading << Orientation(accel_data, mag_data)(0), Orientation(accel_data, mag_data)(1), laser_data;
    reading(0) += heading_alignment;
    reading(1) += inclination_alignment;
    return reading;
}

bool SensorHandler::CollectAlignmentData()
{
    alignment_data.col(alignment_progress) = GetReading();
    alignment_progress++;
    if (alignment_progress == N_ALIGNMENT)
    {
        // TODO
        // Execute alignment maths
        // Make some kind of warning if alignment appears to be low quality. Could be checked by checking angle between each of the spays for angle to te normal. Other method would be to do a test splay between the same two points and checking whether each shot is the inverse of the other
        AlignLaser();
        return 1;
    }
    return 0;
}

void SensorHandler::AlignLaser()
{
    int calib_num;
    // Heading, Inclination, roll, distance
    Vector3f mean_calibration_data;
    // Mean misalignement vector
    Vector3f misalignement_mean;
    // Dummy variable for cartesian <=> spherical conversions
    Vector3f conversion_dummy;
    // Vector direction to target
    Vector3f target_vector;
    // Matrix to hold each calculated misalignement vector
    Matrix<float,3,N_ALIGNMENT> misalignement_mat;
    // Matrix to hold cartesian versions of calibration data
    Matrix<float,3,N_ALIGNMENT> cartesian_calibration_data;
    // Rotation matrix for reversing the effect of tilt
    Matrix3f rotation_matrix;

    // Distance of target from origin
    float target_distance;
    // Cartesian coordinates for target
    float x, y, z;
    // Cartesian coordinated for tip of disto
    float xi, yi, zi;

    // Mean of all calibration data collected
    mean_calibration_data = alignment_data.rowwise().mean();


    /*************************************************************
     * 1. Find the locations of the tip of the disto
     * 2. Find the plane which best fits those points
     * 3. Find the normal vector to this
     * 4. Multiply by target distance to get location of target
     * 5. Convert to cartesian coordinates of target point
     *************************************************************/
    // Calculate cartesian coordinates for disto tip in each calibration shot
    for (calib_num=0; calib_num<N_ALIGNMENT; calib_num++)
    {
        conversion_dummy << alignment_data(0,calib_num), alignment_data(1,calib_num), DISTO_LEN;
        cartesian_calibration_data.col(calib_num) << Cartesian(conversion_dummy);
    }

    // Calculate target distance
    target_distance = pow(pow(DISTO_LEN,2) + pow(mean_calibration_data(3),2) ,0.5); // Pythagoras on disto len and splay len
    // Get normal to best fit plane and multiply by distance
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

    debugf(DEBUG_SENSOR,"Target coordinates X: %f, Y: %f, Z: %f\n",x,y,z); 

    /*************************************************************************************
     * 1. Get the cartesian location of the tip of the disto for each shot
     * 2. Find the rotation matrix corresponding to the roll
     * 3. Calculate the vector between the target and disto tip
     * 4. Rotate this vector by the matrix to revers the effects of tilt on the result
     *************************************************************************************/
    float roll;
    for (calib_num=0; calib_num<N_ALIGNMENT; calib_num++)
    {
        xi = cartesian_calibration_data(0,calib_num);
        yi = cartesian_calibration_data(1,calib_num);
        zi = cartesian_calibration_data(2,calib_num);
        debugf(DEBUG_SENSOR,"Disto tip coordinates X: %f, Y: %f, Z: %f\n",xi,yi,zi);

        // Rotation matrix about x depending on device roll
        roll = alignment_data(2,calib_num);
        rotation_matrix = XRotation(-roll);

        // Calculalate vector between tip of disto and actual target
        misalignement_mat.col(calib_num) << x-xi, y-yi, z-zi;
        // Reverse the effects of tilt on the misalignement vector
        misalignement_mat.col(calib_num) = rotation_matrix * misalignement_mat.col(calib_num);
        //Serial.printf("Misalignment vector: X: %f, Y: %f, Z: %f\n",misalignement_mat(0,calib_num),misalignement_mat(1,calib_num),misalignement_mat(2,calib_num));
        debugf(DEBUG_SENSOR,"Misalignment mat X: %f, Y: %f, Z: %f\n",x-xi, y-yi, z-zi);
        debugf(DEBUG_SENSOR,"Rotated misalignment mat X: %f, Y: %f, Z: %f\n\n",misalignement_mat(0,calib_num), misalignement_mat(1,calib_num), misalignement_mat(2,calib_num));
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
    debugf(DEBUG_SENSOR,"Mean misalignment vector: X: %f, Y: %f, Z: %f\n",x,y,z);


    // Get heading and inclination corrections by converting back to polar coordinates
    inclination_alignment = atan2(y,x);
    heading_alignment = asin(z/pow(pow(x,2) + pow(y,2) + pow(z,2),0.5));
    debugf(DEBUG_SENSOR,"Spherical coordinates for misalignment vector: Heading: %f, Inclination: %f\n",inclination_alignment,heading_alignment);

}

void SensorHandler::ResetCalibration()
{
    magnetometer->ResetCalibration();
    accelerometer->ResetCalibration();
    alignment_progress = 0;
}

bool SensorHandler::CollectCalibrationData()
{
    /************************************************************
    * 1. Wait 100ms to allow button press perturbation to settle
    * 2. Get an initial set of accelerometer readings
    * 3. Keep taking readings until stdeviation is small enough
    * 4. Take a set of readings per the SAMPLES_PER_ORIENTATION definition
    ************************************************************/
    // Wait for 250ms to allow button-press distrubance to go
    delay(250);
    float start_time = millis();
    float calibration_timeout = 2500; //2500ms

    // Collect a set of N_CALIB_STDEV samples
    std::queue<Vector3f> samples;
    for (int i=0;i<N_CALIB_STDEV;i++)
    {
        samples.push(GetReading());
    }

    // Keep collecting samples until the device is deemed to be still enough
    while (StdDev(samples) > CALIBRATION_STDEV_MIN)
    {   
        if (millis() > start_time + calibration_timeout)
        {
            return -1;
        }
        samples.push(accelerometer->GetReading());
        samples.pop();
    }

    // Collect SAMPLES_PER_ORIENTATION samples
    for(int i=0; i<SAMPLES_PER_ORIENTATION;i++)
    {
        if (magnetometer->ColectCalibrationSample() && accelerometer->ColectCalibrationSample())
        {
            return 1;
        }
    }
    return 0;
}

void SensorHandler::CalibrateInertial()
{
    magnetometer->CalibrateLinear();
    accelerometer->CalibrateLinear();
    // TODO
    // Test and work on non-linear fitting with RBFs. Then add function void CalibrateNonLinear()
}

void SensorHandler::AlignInertial()
{

}

void SensorHandler::EnableLaser()
{
    laser->ToggleLaser(true);
}
void SensorHandler::DisableLaser()
{
    laser->ToggleLaser(false);
}