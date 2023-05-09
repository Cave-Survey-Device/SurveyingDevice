#ifndef HEADER_RM3100SC
#define HEADER_RM3100SC
#include "sensors_csd.h"
#include <RM3100.h>

class RM3100SensorConnection: public InertialSensorConnection
{
public:
    RM3100SensorConnection(RM3100* ptr_rm3100);
    Vector3f GetRawData();
    
private:
    RM3100* rm3100_connection;
};

#endif