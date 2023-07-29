#ifndef HEADER_RM3100SC
#define HEADER_RM3100SC

#include "inertialsensor.h"
#include <RM3100.h>

class RM3100SensorConnection: public InertialSensorConnection
{
public:
    RM3100SensorConnection(RM3100* ptr_rm3100);
    Vector3f getRawData();
    
private:
    RM3100* rm3100_connection; /** Pointer to the rm3100_object associated with this connection*/
};

RM3100SensorConnection::RM3100SensorConnection(RM3100* ptr_rm3100)
{
    this->rm3100_connection = ptr_rm3100;
}

Vector3f RM3100SensorConnection::getRawData()
{
    rm3100_connection->update();
    Eigen::Vector3f data;
    // Div by 45 to come closer to normalised
    data << rm3100_connection->getX()/45, rm3100_connection->getY()/45, rm3100_connection->getZ()/45;
    // Serial << "RM3100 data: ";
    // displayRowVec(data);
    return data;
}

#endif