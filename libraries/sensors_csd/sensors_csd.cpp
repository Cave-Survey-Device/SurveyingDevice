#include "sensors_csd.h"

#include <NumericalMethods_csd.h>
#include <utility_csd.h>
#include <queue>



void InertialSensor::CalibrateLinear()
{
    RowVector<float,10> U;
    Matrix3f M;
    Vector3f n;
    float d;

    Serial << "Begin ellipsoid fitting...\n";
    U = fit_ellipsoid(this->calibration_data);
    M << U[0], U[5], U[4], U[5], U[1], U[3], U[4], U[3], U[2];
    n << U[6], U[7], U[8];
    d = U[9];

    Serial << "Begin ellipsoid transformation calculations\n";
    Vector<float, 12> transformation = calculate_ellipsoid_transformation(M, n, d);

    this->calibration_matrix << transformation[0], transformation[1], transformation[2], transformation[3], transformation[4], transformation[5], transformation[6], transformation[7], transformation[8];
    this->calibration_offset << transformation[9], transformation[10], transformation[11];
}

Vector3f InertialSensor::GetReading()
{
    Vector3f reading;
    reading.setZero();
    for (int i=0;i<SAMPLES_PER_READING;i++)
    {
        reading += this->sensor->GetRawData();
    }
    reading = reading/SAMPLES_PER_READING;
    
    return this->calibration_matrix * (reading - this->calibration_offset);
}

bool InertialSensor::ColectCalibrationSample()
{
    Vector3f data = this->GetReading();
    this->calibration_data.col(this->calib_num) << data;
    this->calib_num++;
    if (this->calib_num == N_CALIB)
    {
        return 1;
    }
    return 0;
}

void InertialSensor::ResetCalibration()
{
    this->calib_num = 0;
    this->calibration_data.setZero(3,N_CALIB);
    this-> calibration_matrix = Matrix3f::Identity();
    this->calibration_offset.setZero();
}


InertialSensor::InertialSensor(InertialSensorConnection* sc)
{
    this->sensor = sc;
    this->ResetCalibration();
}

Matrix3f InertialSensor::GetT()
{
    return this->calibration_matrix;
}

Vector3f InertialSensor::Geth()
{
    return this->calibration_offset;
}

Matrix<float,3,N_CALIB> InertialSensor::GetCalibData()
{
    return this->calibration_data;
}


SensorHandler::SensorHandler(InertialSensor* acc, InertialSensor* mag, LaserSensor* las)
{
    this->magnetometer = mag;
    this->accelerometer = acc;
    this->laser = las;
}

SensorHandler::SensorHandler(InertialSensor* acc, InertialSensor* mag)
{
    this->magnetometer = mag;
    this->accelerometer = acc;
}

SensorHandler::SensorHandler(){}

Vector3f SensorHandler::GetReading()
{
    Vector3f reading;
    Vector3f mag_data = this->magnetometer->GetReading();
    Vector3f accel_data = this->accelerometer->GetReading();
    float laser_data = this->laser->GetMeasurement();    

    reading << Orientation(accel_data, mag_data)(0), Orientation(accel_data, mag_data)(1), laser_data;
    reading(0) += this->heading_alignment;
    reading(1) += this->inclination_alignment;
    return reading;
}

bool SensorHandler::CollectAlignmentData()
{
    this->alignment_data.col(this->alignment_progress) = this->GetReading();
    this->alignment_progress++;
    if (this->alignment_progress == N_ALIGNMENT)
    {
        // TODO
        // Execute alignment maths
        // Make some kind of warning if alignment appears to be low quality. Could be checked by checking angle between each of the spays for angle to te normal. Other method would be to do a test splay between the same two points and checking whether each shot is the inverse of the other
        this->AlignLaser();
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
    mean_calibration_data = this->alignment_data.rowwise().mean();


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
        roll = this->alignment_data(2,calib_num);
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
    this->inclination_alignment = atan2(y,x);
    this->heading_alignment = asin(z/pow(pow(x,2) + pow(y,2) + pow(z,2),0.5));
    debugf(DEBUG_SENSOR,"Spherical coordinates for misalignment vector: Heading: %f, Inclination: %f\n",this->inclination_alignment,this->heading_alignment);

}

void SensorHandler::ResetCalibration()
{
    this->magnetometer->ResetCalibration();
    this->accelerometer->ResetCalibration();
    this->alignment_progress = 0;
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
        if (this->magnetometer->ColectCalibrationSample() && this->accelerometer->ColectCalibrationSample())
        {
            return 1;
        }
    }
    return 0;
}

void SensorHandler::CalibrateInertial()
{
    Serial << "Calibrating accelerometer...\n";
    this->accelerometer->CalibrateLinear();
    Serial << "Calibrating magnetometer...\n";
    this->magnetometer->CalibrateLinear();
    
    // TODO
    // Test and work on non-linear fitting with RBFs. Then add function void CalibrateNonLinear()
}

void SensorHandler::AlignInertial()
{

}

void SensorHandler::EnableLaser()
{
    this->laser->ToggleLaser(true);
}
void SensorHandler::DisableLaser()
{
    this->laser->ToggleLaser(false);
}

InertialSensor* SensorHandler::GetAccelPtr()
{
    return this->accelerometer;
}
InertialSensor* SensorHandler::GetMagPtr()
{
    return this->magnetometer;
}

LaserSensor* SensorHandler::GetLaserPtr()
{
    return this->laser;
}