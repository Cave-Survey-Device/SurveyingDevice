#include "sensorhandler.h"
#include <NumericalMethods_csd.h>
#include <utility_csd.h>
#include <queue>
#include <filesystem_csd.h>
#define SENSORHANDLER_EXTENDED_DEBUG

SensorHandler::SensorHandler(Accelerometer* acc, Magnetometer* mag, LaserSensor* las)
{
    this->magnetometer = mag;
    this->accelerometer = acc;
    this->laser = las;
    this->using_laser = true;
    this->resetCalibration();
    this->resetAlignment();
    // this->loadCalibration();
    // this->loadCalibration();
}

SensorHandler::SensorHandler(Accelerometer* acc, Magnetometer* mag)
{
    this->magnetometer = mag;
    this->accelerometer = acc;
    this->using_laser = false;
    this->resetCalibration();
    this->resetAlignment();
    // this->loadCalibration();
    // this->loadCalibration();
}

SensorHandler::SensorHandler(){
    this->resetCalibration();
    this->resetAlignment();
}

Vector3f SensorHandler::update()
{
    // debugf(DEBUG_SENSORHANDLER, "SensorHandler::getReading()");
    mag_data = this->magnetometer->getReading();
    accel_data = this->accelerometer->getReading();    
    
    device_orientation << Orientation(accel_data, mag_data);

    return device_orientation;
}

Vector3f SensorHandler::takeShot()
{
    debug(DEBUG_SENSORHANDLER, "SensorHandler::takeShot()\n");
    shot_data = update();
    float measured_len = this->laser->getMeasurement();
    
    
    // Calculate location of target
    Vector3f disto_tip = Cartesian(Vector3f(shot_data(0), shot_data(1), DISTO_LEN));
    Vector3f target = disto_tip + laser_vec * measured_len;

    shot_data = Spherical(target);
    float x, y, z;
    printf("Disto location: \t%f \t%f \t%f \n", disto_tip(0), disto_tip(1), disto_tip(2));
    printf("Measured distance: \t%f \n", measured_len);
    printf("Target location: \t%f \t%f \t%f \n", target(0), target(1), target(2));

    return shot_data;
}

Vector3f SensorHandler::getShotData()
{
    return shot_data;
}

uint16_t SensorHandler::collectAlignment()
{
    if (laser_alignment_progress < N_LASER_CAL)
    {
        debugf(DEBUG_SENSORHANDLER, "SensorHandler::collectAlignment()");
        this->laser_alignment_data.col(this->laser_alignment_progress) = takeShot();
        this->laser_alignment_progress++;
        return 0;
    } else {
        return 1;
    }

}

void SensorHandler::align()
{
    debugf(DEBUG_SENSORHANDLER, "SensorHandler::align()");

    int calib_num;
    Vector3f mean_calibration_data; // Heading, Inclination, roll, distance
    Vector3f misalignement_mean; // Mean misalignement vector
    Vector3f conversion_dummy; // Dummy variable for cartesian <=> spherical conversions
    Vector3f target_vector; // Vector direction to target
    Matrix<float,3,N_LASER_CAL> misalignement_mat; // Matrix to hold each calculated misalignement vector
    Matrix<float,3,N_LASER_CAL> cartesian_calibration_data; // Matrix to hold cartesian versions of calibration data
    Matrix3f rotation_matrix; // Rotation matrix for reversing the effect of tilt
    float target_distance; // Distance of target from origin
    float x, y, z; // Cartesian coordinates for target
    float xi, yi, zi; // Cartesian coordinated for tip of disto

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

    debugf(DEBUG_SENSORHANDLER,"Target coordinates X: %f, Y: %f, Z: %f\n",x,y,z); 

    /*************************************************************************************
     * 1. Get the cartesian location of the tip of the disto for each shot
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
        debugf(DEBUG_SENSORHANDLER,"Disto tip coordinates X: %f, Y: %f, Z: %f\n",xi,yi,zi);

        // Rotation matrix about x depending on device roll
        roll = this->laser_alignment_data(2,calib_num);
        rotation_matrix = x_rotation(-RAD_TO_DEG * roll);

        // Calculalate vector between tip of disto and actual target
        misalignement_mat.col(calib_num) << x-xi, y-yi, z-zi;
        // Reverse the effects of tilt on the misalignement vector
        misalignement_mat.col(calib_num) = rotation_matrix * misalignement_mat.col(calib_num);
        //Serial.printf("Misalignment vector: X: %f, Y: %f, Z: %f\n",misalignement_mat(0,calib_num),misalignement_mat(1,calib_num),misalignement_mat(2,calib_num));
        debugf(DEBUG_SENSORHANDLER,"Misalignment mat X: %f, Y: %f, Z: %f\n",x-xi, y-yi, z-zi);
        debugf(DEBUG_SENSORHANDLER,"Rotated misalignment mat X: %f, Y: %f, Z: %f\n\n",misalignement_mat(0,calib_num), misalignement_mat(1,calib_num), misalignement_mat(2,calib_num));
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
    debugf(DEBUG_SENSORHANDLER,"Mean misalignment vector: X: %f, Y: %f, Z: %f\n",x,y,z);


    // get heading and inclination corrections by converting back to polar coordinates
    // debugf(DEBUG_SENSORHANDLER,"Spherical coordinates for misalignment vector: Heading: %f, Inclination: %f\n",this->laser_inclination_alignment,this->laser_heading_alignment);

}

void SensorHandler::resetCalibration()
{
    debug(DEBUG_SENSORHANDLER,"SensorHandler::resetCalibration()");
    this->magnetometer->resetCalibration();
    this->accelerometer->resetCalibration();
    this->inertial_alignment_mat = Matrix3f::Identity();
    this->inclination_angle = 0;
}

void SensorHandler::resetAlignment()
{
    debug(DEBUG_SENSORHANDLER,"SensorHandler::resetAlignment()");
    this->laser_alignment_progress = 0;
    this->laser_alignment_data.setZero();
    this->laser_alignment_mat.setZero();
}

uint16_t SensorHandler::collectCalibration()
{
    debug(DEBUG_SENSORHANDLER,"SensorHandler::collectCalibration()");
    /************************************************************
    * 1. Wait 100ms to allow button press perturbation to settle
    * 2. get an initial set of accelerometer readings
    * 3. Keep taking readings until stdeviation is small enough
    * 4. Take a set of readings per the SAMPLES_PER_ORIENTATION definition
    ************************************************************/
    // 1. Wait for 250ms to allow button-press distrubance to go
    delay(250);
    float start_time = millis();
    float calibration_timeout = 2500; //2500ms
    bool calibrated = false;
    Matrix<float,3,N_CALIB_STDEV> samples;


    // 2 Collect a set of N_CALIB_STDEV samples
    for (int i=0;i<N_CALIB_STDEV;i++)
    {
        samples.col(i) << accelerometer->getSingleSample();
    }

    // 3  Keep collecting samples until the device is deemed to be still enough
    int i=0;
    while (StdDev(samples) > CALIBRATION_STDEV_MIN)
    {   
        i++;
        if (millis() > start_time + calibration_timeout)
        {
            return -1;
        }
        samples.col(i%N_CALIB_STDEV) << accelerometer->getSingleSample();
    }
    
    // 4. Collect SAMPLES_PER_ORIENTATION samples
    for(int i=0; i<SAMPLES_PER_ORIENTATION; i++)
    {
        calibrated = this->magnetometer->addCalibData();
        calibrated = calibrated * this->accelerometer->addCalibData();
        if (calibrated)
        {
            return 1;
        }
    }
    return 0;
}

void SensorHandler::calibrate()
{
    /*********************************************************************************************
     * This function calculates the matrix required to transform the magnetometer axis onto the
     *  accelerometer axis. The process works by:
     * 
     * 1. Check if accelerometer and magnetometer have already been calibratied and calibrate if not
     * 2. Sort the calibration data matrices such that zero-valued data is at the end
     * 3. Calculate the corrected calibration data and save it over the top of the old calibration
     *    data (which is already saved in a tmp file so no data loss)
     * 4. Use the calibrated data to calculate the alignment by calling AlignMacAcc(...)
     * 5. TODO: Save the generated alignment data to a tmp file
    *********************************************************************************************/
   debug(DEBUG_SENSORHANDLER,"SensorHandler::calibrate()");
    // 1. Calibrate if not already calibrated. Uses alignment data
    if (mag_combined_calib == true)
    {
        magnetometer->calibrateLinear();
    }
    if (acc_combined_calib == true)
    {
        accelerometer->calibrateLinear();
    }

    #ifdef SENSORHANDLER_EXTENDED_DEBUG
    Serial << "-----------------------------------------------------------\n\n";
    Serial << "Accelerometer sample data\n";
    displayMat(     (  accelerometer->getCalibData()  ).transpose()    );
    Serial << "Magnetometer sample data\n";
    displayMat(     (  magnetometer->getCalibData()  ).transpose()    );
    Serial << "-----------------------------------------------------------\n\n";
    #endif

    // 2. Remove zero valued data
    debug(DEBUG_SENSORHANDLER,"Removing zero data...");
    int acc_size = accelerometer->getCalibData().cols()-removeNullData(accelerometer->getCalibData());
    int mag_size = magnetometer->getCalibData().cols()-removeNullData(magnetometer->getCalibData());

    // 3. Calculate corrections and modift calibration data
    debug(DEBUG_SENSORHANDLER,"Calculating corrections...");
    accelerometer->getCalibData() = accelerometer->getCalibMat() * (accelerometer->getCalibData().colwise() - accelerometer->getCalibBias());
    magnetometer->getCalibData() = magnetometer->getCalibMat() * (magnetometer->getCalibData().colwise() - magnetometer->getCalibBias());

    #ifdef SENSORHANDLER_EXTENDED_DEBUG
    Serial << "Accelerometer correction data";
    displayMat(accelerometer->getCalibData());
    Serial << "Magnetometer correction data";
    displayMat(magnetometer->getCalibData());
    #endif

    // 4. Calculate alignment
    debug(DEBUG_SENSORHANDLER,"Calculating alignment...");
    Vector<float,10> X = AlignMagAcc(accelerometer->getCalibData(), magnetometer->getCalibData());
    inertial_alignment_mat = X.segment(0,9).reshaped(3,3);
    inclination_angle = RAD_TO_DEG * asinf(X(9));
    debugf(DEBUG_SENSORHANDLER,"Magnetic inclination angle: %f", inclination_angle);

    // 5. Save alignment in tmp
    // save_tmp_inertial_align_data();

    #ifdef SENSORHANDLER_EXTENDED_DEBUG
    Serial << "\n\n Output vec: ";
    displayVec(X);
    Serial << "\n";
    #endif

}

void SensorHandler::enableLaser()
{
    this->laser->toggleLaser(true);
}
void SensorHandler::disableLaser()
{
    this->laser->toggleLaser(false);
}

void SensorHandler::saveInertialCalib(InertialSensor *sens)
{
    debug(DEBUG_INERTIALSENSOR,"InertialSensor::save_calibration_data()");
    //write_to_file("calib_data",device_ID,&ref_calibration_data(0),ref_calibration_data.size());
    write_to_file(sens->getID(),"calib_data",sens->getCalibData());
    write_to_file(sens->getID(),"calib_mat",sens->getCalibMat());
    write_to_file(sens->getID(),"calib_off",sens->getCalibBias());
}

void SensorHandler::loadInertialCalib(InertialSensor *sens)
{
    debug(DEBUG_INERTIALSENSOR,"InertialSensor::load_calibration_data()");
    //read_from_file("calib_data",device_ID,&ref_calibration_data(0),ref_calibration_data.size());
    read_from_file(sens->getID(),"calib_data",sens->getCalibData());

    Matrix3f mat_buffer;
    Vector3f bias_buffer;
    read_from_file(sens->getID(),"calib_mat",mat_buffer);
    read_from_file(sens->getID(),"calib_off",bias_buffer);
    sens->setCalibMat(mat_buffer);
    sens->setCalibBias(bias_buffer);

}

void SensorHandler::loadCalibration()
{
    debug(DEBUG_SENSORHANDLER,"SensorHandler::loadCalibration()");
    read_from_file("senshandlr","inalign_R",inertial_alignment_mat);
    read_from_file("senshandlr","inalign_d",inclination_angle);
    loadInertialCalib(accelerometer);
    loadInertialCalib(magnetometer);
}

void SensorHandler::saveCalibration()
{
    debug(DEBUG_SENSORHANDLER,"SensorHandler::saveCalibration()");
    write_to_file("senshandlr","inalign_R",inertial_alignment_mat);
    write_to_file("senshandlr","inalign_d",inclination_angle);
    saveInertialCalib(accelerometer);
    saveInertialCalib(magnetometer);
}

void SensorHandler::saveAlignment()
{
    debug(DEBUG_SENSORHANDLER,"SensorHandler::saveAlignment()");
    write_to_file("senshandlr","lasalign_h",laser_vec);
    write_to_file("senshandlr","lasalign_i",inclination_angle);
}
void SensorHandler::loadAlignment()
{
    debug(DEBUG_SENSORHANDLER,"SensorHandler::loadAlignment()");
    read_from_file("senshandlr","lasalign_h",laser_vec);
    read_from_file("senshandlr","lasalign_i",inclination_angle);
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