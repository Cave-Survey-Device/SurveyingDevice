#include "SensorHandler.h"
#include <queue>

static unsigned int counter;

void getFileName(const unsigned int fileID, char (&fname)[FNAME_LENGTH])
{
    sprintf(fname,"SD%03u\0", fileID);
}
void getVarName(const unsigned int counter, char (&varname)[VARNAME_LENGTH])
{
    sprintf(varname,"%03u\0", counter+1);
}

bool getCounter(const unsigned int fileID, unsigned int &counter)
{
    char fname[FNAME_LENGTH];
    getFileName(fileID, fname);
    // If file and varname exist ...
    if (FileFuncs::readFromFile(fname,"counter",counter)){
        return true;
    }
    Debug::debug(Debug::DEBUG_SENSOR,"Counter not found...");
    return false;
}
void setCounter(const unsigned int fileID, const unsigned int &counter)
{
    char fname[FNAME_LENGTH];
    getFileName(fileID, fname);
    FileFuncs::writeToFile(fname,"counter",counter);
}

void saveShotData(const ShotData &sd, const unsigned int fileID)
{
    char fname[FNAME_LENGTH];
    char varname[VARNAME_LENGTH];
    Debug::debug(Debug::DEBUG_SENSOR,"Saving shot data to file...");
    getFileName(fileID,fname);
    Serial.printf("Namespace to write to: %s\n", fname);


    // If file exists doesn't exist, create it with counter = 0
    if (!getCounter(fileID, counter))
    {
        Debug::debug(Debug::DEBUG_SENSOR,"File doesn't exist, creating new one...");
        counter = 0;
        setCounter(fileID, counter);
    } // Get latest shot ID

    // Save shot to file with ID = counter
    Serial.printf("Counter to write to: %u\n", counter);
    getVarName(counter+1,varname);
    setCounter(fileID,counter+1);
    Serial.printf("Key to write to: %s\n", varname);

    FileFuncs::writeToFile(fname,varname,&sd,sizeof(ShotData)); // Save the shot
}
bool readShotData(ShotData &sd, unsigned int fileID, unsigned int shotID)
{
    char fname[FNAME_LENGTH];
    char varname[VARNAME_LENGTH];
    Debug::debug(Debug::DEBUG_SENSOR,"Reading shot data from file...");
    getFileName(fileID,fname);
    getVarName(shotID,varname);
    return FileFuncs::readFromFile(fname,varname,&sd,sizeof(ShotData));
}

bool readShotData(ShotData &sd, unsigned int fileID)
{
    Debug::debug(Debug::DEBUG_SENSOR,"Reading latest shot data from file...");
    getCounter(fileID, counter); // Get latest shot ID
    return readShotData(sd,fileID,counter);
}

// IMPORTANT: References are immutable and must be definied upon initialisation!
SensorHandler::SensorHandler(Accelerometer &a, Magnetometer &m, Laser &l):acc(a), mag(m), las(l){}

void SensorHandler::init()
{
    Debug::debug(Debug::DEBUG_SENSOR, "Acc init...");
    acc.init();
    Debug::debug(Debug::DEBUG_SENSOR, "Mag init...");
    mag.init();
    Debug::debug(Debug::DEBUG_SENSOR, "Las init...");
    las.init();

    Debug::debug(Debug::DEBUG_SENSOR, "Turning laser off...");
    las.toggleLaser(false);

    Debug::debug(Debug::DEBUG_SENSOR, "Loading calibration...");
    loadCalibration();
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
    static_calib_progress = 0;
    las_calib_progress = 0;
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

    // Store corrected data in corrected_acc_data and corrected_mag_data
    correctData(corrected_shot_data.m, corrected_shot_data.g);
    corrected_shot_data.HIR = NumericalMethods::inertialToCardan(corrected_shot_data.m,corrected_shot_data.g);
    corrected_shot_data.v = NumericalMethods::inertialToVector(corrected_shot_data.m,corrected_shot_data.g);
}

Vector3f SensorHandler::getCardan(bool corrected)
{
    if (corrected) 
    {
        return NumericalMethods::inertialToCardan(corrected_acc_data, corrected_mag_data);
    }   else    {
        return NumericalMethods::inertialToCardan(acc_data, mag_data);
    }
}
Vector3f SensorHandler::getCartesian(bool corrected)
{
    if (corrected) 
    {
        return NumericalMethods::inertialToVector(corrected_acc_data, corrected_mag_data);
    }   else    {
        return NumericalMethods::inertialToVector(acc_data, mag_data);
    }
}
ShotData SensorHandler::getShotData(bool corrected)
{
    if (corrected) 
    {
        return corrected_shot_data;
    }   else    {
        return shot_data;
    }
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
    if (las_data <= 0) {return 1;}

    // Update stored shot data
    shot_data.m = mag_data;
    shot_data.g = acc_data;
    shot_data.d = las_data;
    shot_data.HIR = NumericalMethods::inertialToCardan(shot_data.m,shot_data.g);
    shot_data.v = NumericalMethods::inertialToVector(shot_data.m,shot_data.g);

    // Save corrected shot data in corrected_shot_data
    correctData(corrected_shot_data.m, corrected_shot_data.g);
    corrected_shot_data.HIR = NumericalMethods::inertialToCardan(corrected_shot_data.m,corrected_shot_data.g);
    corrected_shot_data.v = NumericalMethods::inertialToVector(corrected_shot_data.m,corrected_shot_data.g);
    corrected_shot_data.d = shot_data.d + DEVICE_LENGTH;



    
    Debug::debugf(Debug::DEBUG_SENSOR,"Laser measurement: %f ", las_data);

    las.toggleLaser(true);

    return 0;
}

void SensorHandler::correctData(Vector3f &m, Vector3f &g)
{
    m = calib_parms.Rm_las * calib_parms.Rm_cal * (mag_data - calib_parms.bm_cal);
    g = calib_parms.Ra_las * calib_parms.Ra_cal * (acc_data - calib_parms.ba_cal);
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
    int n_avg = 15;
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
        Debug::debug(Debug::DEBUG_SENSOR,"Shot failed! Try again.");
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
    Matrix<float,3,N_LASER_CAL> temp_acc_data = calib_parms.Ra_cal * (laser_calib_data.acc_data.colwise() - calib_parms.ba_cal);
    Matrix<float,3,N_LASER_CAL> temp_mag_data = calib_parms.Rm_cal * (laser_calib_data.mag_data.colwise() - calib_parms.bm_cal);

    NumericalMethods::alignToNorm(temp_acc_data, calib_parms.Ra_las);
    NumericalMethods::alignToNorm(temp_acc_data, calib_parms.Ra_las);

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
}

void SensorHandler::removePrevCalib(bool static_calib)
{
    // To implement
}

int SensorHandler::getCalibProgress(bool static_calib)
{
    return static_calib_progress + las_calib_progress;
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