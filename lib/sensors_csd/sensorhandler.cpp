#include "sensorhandler.h"
#include <NumericalMethods_csd.h>
#include <utility_csd.h>
#include <queue>


SensorHandler::SensorHandler(Accelerometer* acc, Magnetometer* mag, LaserSensor* las)
{
    this->magnetometer = mag;
    this->accelerometer = acc;
    this->laser = las;
    this->using_laser = true;
    this->resetCalibration();
}

SensorHandler::SensorHandler(Accelerometer* acc, Magnetometer* mag)
{
    this->magnetometer = mag;
    this->accelerometer = acc;
    this->using_laser = false;
    this->resetCalibration();
}

SensorHandler::SensorHandler(){
    this->resetCalibration();
}

Vector3f SensorHandler::getReading()
{
    Vector3f reading;
    Vector3f mag_data = inertial_alignment_mat * this->magnetometer->getReading();
    Vector3f accel_data = this->accelerometer->getReading();
    if (using_laser)
    {
        float laser_data = this->laser->getMeasurement();   
        reading << Orientation(accel_data, mag_data)(0), Orientation(accel_data, mag_data)(1), laser_data;
    }
     
    Serial << "Mag data: ";
    displayRowVec(mag_data);
    Serial << "Heading: " << Orientation(accel_data, mag_data)(0) << "\t\t" << 
    "Inclination: " << Orientation(accel_data, mag_data)(1) << "\t\t" << 
    "Roll: " << Orientation(accel_data, mag_data)(2) << "\n\n";

    reading << Orientation(accel_data, mag_data)(0), Orientation(accel_data, mag_data)(1), 0.;
    reading(0) += this->laser_heading_alignment;
    reading(1) += this->laser_inclination_alignment;
    return reading;
}

bool SensorHandler::collectLaserAlignmentData()
{
    this->laser_alignment_data.col(this->laser_alignment_progress) = this->getReading();
    this->laser_alignment_progress++;
    if (this->laser_alignment_progress == N_LASER_CAL)
    {
        // TODO
        // Execute alignment maths
        // Make some kind of warning if alignment appears to be low quality. Could be checked by checking angle between each of the spays for angle to te normal. Other method would be to do a test splay between the same two points and checking whether each shot is the inverse of the other
        this->alignLaser();
        return 1;
    }
    return 0;
}

void SensorHandler::alignLaser()
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
    Matrix<float,3,N_LASER_CAL> misalignement_mat;
    // Matrix to hold cartesian versions of calibration data
    Matrix<float,3,N_LASER_CAL> cartesian_calibration_data;
    // Rotation matrix for reversing the effect of tilt
    Matrix3f rotation_matrix;

    // Distance of target from origin
    float target_distance;
    // Cartesian coordinates for target
    float x, y, z;
    // Cartesian coordinated for tip of disto
    float xi, yi, zi;

    // Mean of all calibration data collected
    mean_calibration_data = this->laser_alignment_data.rowwise().mean();


    /*************************************************************
     * 1. Find the locations of the tip of the disto
     * 2. Find the plane which best fits those points
     * 3. Find the normal vector to this
     * 4. Multiply by target distance to get location of target
     * 5. Convert to cartesian coordinates of target point
     *************************************************************/
    // Calculate cartesian coordinates for disto tip in each calibration shot
    for (calib_num=0; calib_num<N_LASER_CAL; calib_num++)
    {
        conversion_dummy << laser_alignment_data(0,calib_num), laser_alignment_data(1,calib_num), DISTO_LEN;
        cartesian_calibration_data.col(calib_num) << Cartesian(conversion_dummy);
    }

    // Calculate target distance
    target_distance = pow(pow(DISTO_LEN,2) + pow(mean_calibration_data(3),2) ,0.5); // Pythagoras on disto len and splay len
    // get normal to best fit plane and multiply by distance
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
     * 1. get the cartesian location of the tip of the disto for each shot
     * 2. Find the rotation matrix corresponding to the roll
     * 3. Calculate the vector between the target and disto tip
     * 4. Rotate this vector by the matrix to revers the effects of tilt on the result
     *************************************************************************************/
    float roll;
    for (calib_num=0; calib_num<N_LASER_CAL; calib_num++)
    {
        xi = cartesian_calibration_data(0,calib_num);
        yi = cartesian_calibration_data(1,calib_num);
        zi = cartesian_calibration_data(2,calib_num);
        debugf(DEBUG_SENSOR,"Disto tip coordinates X: %f, Y: %f, Z: %f\n",xi,yi,zi);

        // Rotation matrix about x depending on device roll
        roll = this->laser_alignment_data(2,calib_num);
        rotation_matrix = x_rotation(-RAD_TO_DEG * roll);

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


    // get heading and inclination corrections by converting back to polar coordinates
    this->laser_inclination_alignment = atan2(y,x);
    this->laser_heading_alignment = asin(z/pow(pow(x,2) + pow(y,2) + pow(z,2),0.5));
    debugf(DEBUG_SENSOR,"Spherical coordinates for misalignment vector: Heading: %f, Inclination: %f\n",this->laser_inclination_alignment,this->laser_heading_alignment);

}

void SensorHandler::resetCalibration()
{
    this->magnetometer->resetCalibration();
    this->accelerometer->resetCalibration();
    this->inertial_alignment_mat = Matrix3f::Identity();
    this->inclination_angle = 0;
    this->laser_alignment_progress = 0;
}

int SensorHandler::collectInertialAlignmentData()
{
    /************************************************************
    * 1. Wait 100ms to allow button press perturbation to settle
    * 2. get an initial set of accelerometer readings
    * 3. Keep taking readings until stdeviation is small enough
    * 4. Take a set of readings per the SAMPLES_PER_ORIENTATION definition
    ************************************************************/
    // Wait for 250ms to allow button-press distrubance to go
    delay(250);
    float start_time = millis();
    float calibration_timeout = 2500; //2500ms
    bool calibrated = false;
    Matrix<float,3,N_CALIB_STDEV> samples;


    // Collect a set of N_CALIB_STDEV samples
    for (int i=0;i<N_CALIB_STDEV;i++)
    {
        samples.col(i) << accelerometer->getReading();
    }

    // Keep collecting samples until the device is deemed to be still enough
    int i=0;
    while (StdDev(samples) > CALIBRATION_STDEV_MIN)
    {   
        i++;
        if (millis() > start_time + calibration_timeout)
        {
            return -1;
        }
        samples.col(i%N_CALIB_STDEV) << accelerometer->getReading();
    }
    
    // Collect SAMPLES_PER_ORIENTATION samples
    for(int i=0; i<SAMPLES_PER_ORIENTATION; i++)
    {
        calibrated = this->magnetometer->collectAlignmentSample();
        calibrated = calibrated * this->accelerometer->collectAlignmentSample();
        if (calibrated)
        {
            return 1;
        }
    }
    return 0;
}

void SensorHandler::calibrateInertial()
{
    Serial << "Calibrating accelerometer...\n";
    this->accelerometer->calibrateLinear();
    Serial << "Calibrating magnetometer...\n";
    this->magnetometer->calibrateLinear();
    
    // TODO
    // Test and work on non-linear fitting with RBFs. Then add function void CalibrateNonLinear()
}

void SensorHandler::alignInertial()
{
    // Calibrate if not already calibrated. Uses alignment data
    if (magnetometer->getCalibMode() == false)
    {
        magnetometer->calibrateLinear();
    }
    if (accelerometer->getCalibMode() == false)
    {
        accelerometer->calibrateLinear();
    }

    Serial << "-----------------------------------------------------------\n\n";
    Serial << "Accelerometer sample data\n";
    displayMat(     (  accelerometer->getCalibData()  ).transpose()    );
    Serial << "Magnetometer sample data\n";
    displayMat(     (  magnetometer->getCalibData()  ).transpose()    );
    Serial << "-----------------------------------------------------------\n\n";


    // Remove zero valued data
    Serial << "Removing zero data...\n";
    // float* mag_data_ptr = &magnetometer->getCalibData()(0,0);
    // float* acc_data_ptr = &accelerometer->getCalibData()(0,0);
    // float* mag_data_ptr = magnetometer->getCalibData().data();
    // float* acc_data_ptr = accelerometer->getCalibData().data();

    // Serial.printf("mag_ptr: %p, acc_ptr %p \n", magnetometer, accelerometer);
    // Serial.printf("mag_mat_ptr: %p, acc_mat_ptr %p \n", &magnetometer->ref_calibration_data, &accelerometer->ref_calibration_data);
    // Serial.printf("mag_data_ptr: %p, acc_data_ptr %p \n", mag_data_ptr, acc_data_ptr);

    int acc_size = accelerometer->getCalibData().cols()-removeNullData(accelerometer->getCalibData());
    int mag_size = magnetometer->getCalibData().cols()-removeNullData(magnetometer->getCalibData());

    // Calculate corrections and modift calibration data
    Serial << "Calculating corrections...\n";
    accelerometer->getCalibData() = accelerometer->getT() * (accelerometer->getCalibData().colwise() - accelerometer->geth());
    magnetometer->getCalibData() = magnetometer->getT() * (magnetometer->getCalibData().colwise() - magnetometer->geth());


    Serial << "Accelerometer correction data";
    displayMat(accelerometer->getCalibData());
    Serial << "Magnetometer correction data";
    displayMat(magnetometer->getCalibData());

    // Calculate alignment
    Serial << "Calculating alignment...\n";
    Vector<float,10> X = AlignMagAcc(accelerometer->getCalibData(), magnetometer->getCalibData());
    inertial_alignment_mat = X.segment(0,9).reshaped(3,3);
    Serial << "Magnetic inclination angle: " << RAD_TO_DEG * asinf(X(9)) << "\n";
    Serial << "\n\n Output vec: ";
    displayVec(X);
    Serial << "\n";


}

void SensorHandler::enableLaser()
{
    this->laser->toggleLaser(true);
}
void SensorHandler::disableLaser()
{
    this->laser->toggleLaser(false);
}

InertialSensor* SensorHandler::getAccelPtr()
{
    return this->accelerometer;
}
InertialSensor* SensorHandler::getMagPtr()
{
    return this->magnetometer;
}

LaserSensor* SensorHandler::getLaserPtr()
{
    return this->laser;
}