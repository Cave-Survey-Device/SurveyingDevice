#ifndef HEADER_SCA3300SC
#define HEADER_SCA3300SC

#include "sensors_csd.h"
#include <SCA3300.h>

class SCA3300SensorConnection: public InertialSensorConnection
{
public:
    SCA3300SensorConnection(SCA3300* ptr_sca3300);
    Vector3f GetRawData();
    
private:
    SCA3300* sca3300_connection;
};

#endif