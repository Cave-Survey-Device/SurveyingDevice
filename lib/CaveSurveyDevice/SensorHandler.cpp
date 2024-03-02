#include "SensorHandler.h"
#include <queue>

// IMPORTANT: References are immutable and must be definied upon initialisation!
SensorHandler::SensorHandler(Accelerometer &a, Magnetometer &m, Laser &l):acc(a), mag(m), las(l){}

void SensorHandler::init()
{
    Serial.print("Acc init...\n");
    acc.init();
    Serial.print("Mag init...\n");
    mag.init();
    Serial.print("Las init...\n");
    las.init();

    las.toggleLaser(false);

    // Should be loadCalibration
    resetCalibration();
}

void SensorHandler::resetCalibration()
{
    calib_parms.Ra_cal.setIdentity();
    calib_parms.Rm_cal.setIdentity();
    calib_parms.Ra_las.setIdentity();
    calib_parms.Rm_las.setIdentity();
    calib_parms.Rm_align.setIdentity();
    calib_parms.ba_cal.setZero();
    calib_parms.bm_cal.setZero();
    calib_parms.inclination_angle = 0;
}

void SensorHandler::update()
{
    mag_data << 0,0,0;
    acc_data << 0,0,0;
    for (int i=0; i<N_UPDATE_SAMPLES; i++)
    {
        mag_data += mag.getMeasurement();
        acc_data += acc.getMeasurement();
    }
    mag_data /= N_UPDATE_SAMPLES;
    acc_data /= N_UPDATE_SAMPLES;

    correctData();
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
    static Vector3f cartesian, cardan, out;
    correctData();
    // cartesian = NumericalMethods ::inertialToCartesian(acc_data,mag_data);

    // Heading, Inclination, Distance
    // Use of -ve to convert from RH cardan to LH heading and inclination
    out << -NumericalMethods::inertialToCardan(acc_data,mag_data);
    out(2) = las_data+DEVICE_LENGTH;
    return out;
}


int SensorHandler::takeShot(const bool laser_reading, const bool use_stabilisation)
{
    if (use_stabilisation)
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
    if (las_data < 0) {return 1;}

    // Update stored shot data
    shot_data.m = mag_data;
    shot_data.g = acc_data;
    shot_data.d = las_data;
    
    Serial.printf("\nLaser measurement: %f \n", las_data);

    return 0;
}

void SensorHandler::correctData()
{
    mag_data = calib_parms.Rm_las * calib_parms.Rm_align * calib_parms.Rm_cal * (mag_data - calib_parms.bm_cal);
    acc_data = calib_parms.Ra_las * calib_parms.Ra_cal * (acc_data - calib_parms.ba_cal);
}

void SensorHandler::eraseFlash()
{
    FileFuncs::erase_flash();
}
void SensorHandler::getFlashStats()
{
    FileFuncs::getStatus();
}

int SensorHandler::collectStaticCalibData()
{
    if (static_calib_progress >= N_ORIENTATIONS) { return N_ORIENTATIONS; }

    int index = 0;
    int n_avg = 25;
    Vector3f g, m;
    for (int i=0; i<N_SAMPLES_PER_ORIENTATION;i++)
    {
        index = static_calib_progress*N_SAMPLES_PER_ORIENTATION+i;

        g.setZero();
        m.setZero();

        for (int j=0; j<n_avg;j++)
        {
            g += acc.getMeasurement();
            m += mag.getMeasurement();
            // Serial.println(j);
        }
        static_calib_data.acc_data.col(index) = g/n_avg;
        static_calib_data.mag_data.col(index) = m/n_avg;
    }

    // takeShot(false);
    // mag_align_data.col(mag_acc_align_progress) = getMagData();
    // acc_align_data.col(mag_acc_align_progress) = getAccData();
    static_calib_progress++;
    return static_calib_progress;

}
int SensorHandler::collectLaserCalibData()
{
    if (las_calib_progress >= N_LASER_CAL) { return N_LASER_CAL; }

    if (takeShot()) {
        Serial.print("Shot failed! Try again.\n");
        return las_calib_progress;
    }

    laser_calib_data.acc_data.col(las_calib_progress) = acc_data;
    laser_calib_data.mag_data.col(las_calib_progress) = mag_data;
    
    las_calib_progress++;
    return las_calib_progress;
}

int SensorHandler::calibrate()
{
    Vector<float,10> Um = NumericalMethods::fitEllipsoid(static_calib_data.mag_data);
    NumericalMethods::calculateEllipsoidTransformation(Um, calib_parms.Rm_cal, calib_parms.bm_cal);

    Vector<float,10> Ua = NumericalMethods::fitEllipsoid(static_calib_data.acc_data);
    NumericalMethods::calculateEllipsoidTransformation(Ua, calib_parms.Ra_cal, calib_parms.ba_cal);
    
    return 0;
}
int SensorHandler::align()
{
    //{
    Matrix<float,3,N_LASER_CAL> temp_acc_data = calib_parms.Ra_cal * (laser_calib_data.acc_data.colwise() - calib_parms.ba_cal);
    // MatrixXf temp_mag_data = calib_parms.Rm_cal * (laser_calib_data.mag_data.colwise() - calib_parms.bm_cal);
    // Using the laser data, align the sensors with the principal device axis
    // NumericalMethods::alignLaser(
    //     laser_calib_data.acc_data,laser_calib_data.mag_data,
    //     calib_parms.Ra_las,calib_parms.Rm_las);  
    // }

    NumericalMethods::alignToNorm(temp_acc_data, calib_parms.Ra_las);
    calib_parms.Rm_las = calib_parms.Ra_las;

    // {
    // MatrixXf temp_acc_data = calib_parms.Ra_las * calib_parms.Ra_cal * (static_calib_data.acc_data.colwise() - calib_parms.ba_cal);
    // MatrixXf temp_mag_data = calib_parms.Rm_las * calib_parms.Rm_cal * (static_calib_data.mag_data.colwise() - calib_parms.bm_cal);

    // // Run fine-tuning alignment algorithm on the newly aligned data
    // NumericalMethods::alignMagAcc(
    //     temp_acc_data,
    //     temp_mag_data,
    //     calib_parms.Rm_align,calib_parms.inclination_angle);
    // }


    return 0;
}
int SensorHandler::staticAlign()
{
    MatrixXf temp_acc_data = calib_parms.Ra_cal * (static_calib_data.acc_data.colwise() - calib_parms.ba_cal);
    MatrixXf temp_mag_data = calib_parms.Rm_cal * (static_calib_data.mag_data.colwise() - calib_parms.bm_cal);

    // Run fine-tuning alignment algorithm on the newly aligned data
    NumericalMethods::alignMagAcc(
        temp_acc_data,
        temp_mag_data,
        calib_parms.Rm_align,calib_parms.inclination_angle);
    return 0;
}
void SensorHandler::saveCalibration()
{
    EigenFileFuncs::writeToFile("static_calib","acc_data", static_calib_data.acc_data);
    EigenFileFuncs::writeToFile("static_calib","mag_data", static_calib_data.mag_data);
    EigenFileFuncs::writeToFile("laser_calib","acc_data", laser_calib_data.acc_data);
    EigenFileFuncs::writeToFile("laser_calib","mag_data", laser_calib_data.mag_data);

    EigenFileFuncs::writeToFile("calib_parms","Ra_cal", calib_parms.Ra_cal);
    EigenFileFuncs::writeToFile("calib_parms","ba_cal", calib_parms.ba_cal);
    EigenFileFuncs::writeToFile("calib_parms","Rm_cal", calib_parms.Rm_cal);
    EigenFileFuncs::writeToFile("calib_parms","bm_cal", calib_parms.bm_cal);

    EigenFileFuncs::writeToFile("calib_parms","Ra_las", calib_parms.Ra_las);
    EigenFileFuncs::writeToFile("calib_parms","Rm_las", calib_parms.Rm_las);
    EigenFileFuncs::writeToFile("calib_parms","Rm_align", calib_parms.Rm_align);
}
void SensorHandler::loadCalibration()
{
    EigenFileFuncs::readFromFile("static_calib","acc_data", static_calib_data.acc_data);
    EigenFileFuncs::readFromFile("static_calib","mag_data", static_calib_data.mag_data);
    EigenFileFuncs::readFromFile("laser_calib","acc_data", laser_calib_data.acc_data);
    EigenFileFuncs::readFromFile("laser_calib","mag_data", laser_calib_data.mag_data);

    EigenFileFuncs::readFromFile("calib_parms","Ra_cal", calib_parms.Ra_cal);
    EigenFileFuncs::readFromFile("calib_parms","ba_cal", calib_parms.ba_cal);
    EigenFileFuncs::readFromFile("calib_parms","Rm_cal", calib_parms.Rm_cal);
    EigenFileFuncs::readFromFile("calib_parms","bm_cal", calib_parms.bm_cal);

    EigenFileFuncs::readFromFile("calib_parms","Ra_las", calib_parms.Ra_las);
    EigenFileFuncs::readFromFile("calib_parms","Rm_las", calib_parms.Rm_las);
    EigenFileFuncs::readFromFile("calib_parms","Rm_align", calib_parms.Rm_align);
}

Vector3f SensorHandler::getMagData()
{
    return mag_data;
}
Vector3f SensorHandler::getAccData()
{
    return acc_data;
}

const StaticCalibrationData& SensorHandler::getStaticCalibData()
{
    return static_calib_data;
}
const LaserCalibrationData& SensorHandler::getLaserCalibData()
{
    return laser_calib_data;
}
const DeviceCalibrationParameters& SensorHandler::getCalibParms()
{
    return calib_parms;
}