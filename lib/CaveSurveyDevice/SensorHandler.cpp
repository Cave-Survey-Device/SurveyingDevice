#include "SensorHandler.h"
#include <queue>

// IMPORTANT: References are immutable and must be definied upon initialisation!
SensorHandler::SensorHandler(Accelerometer &a, Magnetometer &m, Laser &l):acc(a), mag(m), las(l){}

void SensorHandler::init()
{
    acc.init();
    mag.init();
    las.init();

    // Should be loadCalibration
    resetCalibration();
}

Vector3f SensorHandler::getAccData() { return acc_data; }
Vector3f SensorHandler::getMagData() { return mag_data; }
float SensorHandler::getLasData() { return las_data; }

void SensorHandler::resetCalibration()
{
    calib_parms.laser_inclination = 0;
    calib_parms.laser_heading = 0;
    calib_parms.Ra.setIdentity();
    calib_parms.Rm.setIdentity();
    calib_parms.ba.setZero();
    calib_parms.bm.setZero();
    calib_parms.Ralign.setIdentity();
    calib_parms.inclination_angle = 0;
}


Vector3f SensorHandler::getCardan()
{
    return NumericalMethods::inertialToCardan(acc_data, mag_data);
}
Vector3f SensorHandler::getCartesian()
{
    return NumericalMethods::inertialToCartesian(acc_data, mag_data);

}
Vector3f SensorHandler::getFinalMeasurement()
{
    static Vector3f cartesian, cardan, laser_direction, target_location, out;
     
    // Apply inertial calibration
    mag_data = calib_parms.Ralign * calib_parms.Rm * (mag_data - calib_parms.ba);
    acc_data = calib_parms.Ra * (acc_data - calib_parms.ba);

    // Apply laser calibration
    cardan = NumericalMethods ::inertialToCardan(acc_data,mag_data);
    cartesian = NumericalMethods::inertialToCartesian(acc_data,mag_data);
    laser_direction = NumericalMethods::cardanToCartesian(Vector3f(
        cardan(0) + calib_parms.laser_heading,
        cardan(1) + calib_parms.laser_inclination,
        0));

    target_location = cartesian*DEVICE_LENGTH + las_data*laser_direction;

    // Heading, Inclination, Distance
    // Use of -ve to convert from RH cardan to LH heading and inclination
    out << -NumericalMethods::cartesianToCardan(target_location), target_location.norm();

    return out;

}



int SensorHandler::takeShot(const bool laser_reading)
{
    // Wait until device is steady
    Vector<float,N_STABILISATION> norm_buffer;
    for (int i=0; i<N_STABILISATION; i++)
    {
        norm_buffer(i) = acc.getMeasurement().norm();
    }

    int i = 0;
    while (NumericalMethods::stDev(norm_buffer) > STDEV_LIMIT)
    {
        norm_buffer(i%N_STABILISATION) = acc.getMeasurement().norm();
        i++;

        if (i > 1000) { return 1; }
    }
    

    // Take samples
    mag_data << 0,0,0;
    acc_data << 0,0,0;
    for (int i=0; i<N_SHOT_SMAPLES; i++)
    {
        mag_data += mag.getMeasurement();
        acc_data += acc.getMeasurement();
    }
    mag_data /= N_SHOT_SMAPLES;
    acc_data /= N_SHOT_SMAPLES;

    if (laser_reading) { las_data = las.getMeasurement(); }

    return 0;
}

Vector2i SensorHandler::getMagCalIndex(const Vector3f &m)
{
    static Vector2f indexf;
    static Vector2i index;
    indexf << (RAD_TO_DEG * NumericalMethods::cartesianToCardan(m));
    index << floor(indexf(0)), floor(indexf(1));

    // Heading ranges from 0 to 360 and inclination from -90 to 90
    index << floor(index(0) * (N_MAG_CAL_HEADING-1)/360), floor((index(0)+90) * (N_MAG_CAL_INCLINATION-1)/180);
    return index;
}

int SensorHandler::nFilledMagCalibIndices()
{
    return mag_calib_data_filled_indices.array().sum();
}

int SensorHandler::collectMagCalibData()
{
    // Collect data
    static Vector2i index;
    mag_data = mag.getMeasurement();
    index = getMagCalIndex(mag_data);

    mag_calib_data.col(index(0)*N_MAG_CAL_INCLINATION+index(1)) = mag_data;
    mag_calib_data_filled_indices(index(0),index(1)) = 1;

    return nFilledMagCalibIndices();
}

int SensorHandler::collectMagAccAlignData()
{
    if (mag_acc_align_progress >= N_ALIGN_MAG_ACC) { return -1; }

    takeShot(false);
    mag_align_data.col(mag_acc_align_progress) = getMagData();
    acc_align_data.col(mag_acc_align_progress) = getAccData();
    mag_acc_align_progress++;
    return mag_acc_align_progress;

}

int SensorHandler::collectLaserAlignData()
{
    if (mag_acc_align_progress >= N_LASER_CAL) { return -1; }

    if (!takeShot()) {
        Serial.print("Shot timed out! Try again.");
        return las_align_progress;
    }

    laser_align_data.col(las_align_progress) << NumericalMethods::inertialToCardan(acc_data,mag_data), las_data;
    las_align_progress++;
    return las_align_progress;
}

int SensorHandler::calibrateMagnetometer()
{
    int n_zeros = NumericalMethods::removeNullData(mag_calib_data);
    Vector<float,10> U = NumericalMethods::fitEllipsoid(mag_calib_data.block(0,0,3,N_MAG_CAL - n_zeros));
    NumericalMethods::calculateEllipsoidTransformation(U, calib_parms.Rm, calib_parms.bm);
    return 0;
}

int SensorHandler::alignInertial()
{
    if (MAG_COMBINED_CAL)
    {
        Vector<float,10> Um = NumericalMethods::fitEllipsoid(mag_align_data);
        NumericalMethods::calculateEllipsoidTransformation(Um, calib_parms.Rm, calib_parms.bm);
    }

    Vector<float,10> Ua = NumericalMethods::fitEllipsoid(acc_align_data);
    NumericalMethods::calculateEllipsoidTransformation(Ua, calib_parms.Ra, calib_parms.ba);

    NumericalMethods::alignMagAcc(acc_align_data, mag_align_data,calib_parms.Ralign,calib_parms.inclination_angle);
    return 0;
}

int SensorHandler::alignLaser()
{
    Vector2f V = NumericalMethods::alignLaser(laser_align_data);
    calib_parms.laser_heading = V(0);
    calib_parms.laser_inclination = V(1);

    return 0;
}