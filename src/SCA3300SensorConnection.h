#ifndef HEADER_SCA3300SC
#define HEADER_SCA3300SC

#include <CaveSurveyDevice.h>
#include <SCA3300.h>

class SCA3300SensorConnection: public Accelerometer
{
public:
    SCA3300SensorConnection(SCA3300 &sca3300);
    Vector3f getMeasurement();
    void init();
private:
    SCA3300& sca3300_connection;
};

SCA3300SensorConnection::SCA3300SensorConnection(SCA3300& sca3300):sca3300_connection(sca3300){}

void SCA3300SensorConnection::init()
{
    if (sca3300_connection.begin() != 0x51) {Serial.print("SCA3300 Initialisation Failed!\n"); }
}

Vector3f SCA3300SensorConnection::getMeasurement()
{
    Eigen::Vector3f data;
    bool available = false;

    while (!available)
    {
        available = sca3300_connection.available();
        if (!available) {
            sca3300_connection.reset();
        }
    }

    data <<
    (float)sca3300_connection.getCalculatedAccelerometerX(),
    -(float)sca3300_connection.getCalculatedAccelerometerY(),
    -(float)sca3300_connection.getCalculatedAccelerometerZ();


    return data;
}

#endif