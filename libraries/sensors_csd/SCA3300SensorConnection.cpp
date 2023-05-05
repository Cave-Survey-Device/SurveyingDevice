#include "SCA3300SensorConnection.h"


SCA3300SensorConnection::SCA3300SensorConnection(SCA3300* ptr_sca3300)
{
    this->sca3300_connection = ptr_sca3300;
}

Vector3f SCA3300SensorConnection::GetRawData()
{
    Eigen::Vector3f data;
    bool available = false;

    while (!available)
    {
        available = sca3300_connection->available();
        if (!available) {
            sca3300_connection->reset();
        }
    }


    data << (float)sca3300_connection->getCalculatedAccelerometerX(), (float)sca3300_connection->getCalculatedAccelerometerY(), (float)sca3300_connection->getCalculatedAccelerometerZ();
    return data;
}
