#include "SensorHandler.h"
#include <queue>

Vector3f SensorHandler::getAccData() { return acc_data; }
Vector3f SensorHandler::getMagData() { return mag_data; }
Vector3f SensorHandler::getLasData() { return las_data; }

int SensorHandler::takeShot(const bool laser_reading)
{
    // Wait until device is steady
    Vector<float,N_STABILISATION> norm_buffer;
    for (int i=0; i<N_STABILISATION; i++)
    {
        norm_buffer(i) = acc.getReading().norm();
    }

    int i = 0;
    while (stDev(norm_buffer) > STDEV_LIMIT)
    {
        norm_buffer(i%N_STABILISATION) = acc.getReading().norm();
        i++;

        if (i > 1000) { return 1; }
    }
    

    // Take samples
    mag_data << 0,0,0;
    acc_data << 0,0,0;
    for (int i=0; i<N_SHOT_SMAPLES; i++)
    {
        mag_data += mag.getSingleSample();
        acc_data += acc.getSingleSample();
    }
    mag_data /= N_SHOT_SMAPLES;
    acc_data /= N_SHOT_SMAPLES;

    if (laser_reading) { las_data = las.getReading(); }

    return 0;
}

Vector2i SensorHandler::getMagCalIndex(const Vector3f &m)
{
    static Vector2i index;
    index << (RAD_TO_DEG * cartesianToCardan(m)).array().floor();

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
    mag_data = mag.getSingleSample();
    index = getMagCalIndex(mag_data);

    mag_calib_data.col(index(0)*N_MAG_CAL_INCLINATION+index(1)) = mag_data;
    mag_calib_data_filled_indices(index(0),index(1)) = 1;

    return nFilledMagCalibIndices();
}

int SensorHandler::collectMagAccAlignData()
{
    takeShot(false);
    mag_align_data.col(mag_acc_align_progress) = getMagData();
    acc_align_data.col(mag_acc_align_progress) = getAccData();
    mag_acc_align_progress++;
    return mag_acc_align_progress;

}
int SensorHandler::collectLaserAlignData()
{
    if (!takeShot()) {
        Serial.print("Shot timed out! Try again.");
        return las_align_progress;
    }
    
    laser_align_data.col(las_align_progress) << inertialToCardan(acc_data,mag_data), las_data;
    las_align_progress++;
    return las_align_progress;
}

