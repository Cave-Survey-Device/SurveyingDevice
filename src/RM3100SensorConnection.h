#ifndef HEADER_RM3100SC
#define HEADER_RM3100SC

#include <CaveSurveyDevice.h>
#include <RM3100.h>

class RM3100SensorConnection: public Magnetometer
{
public:
    RM3100SensorConnection(RM3100 &rm3100);
    Vector3f getMeasurement();
    void init();
private:
    RM3100 &rm3100_connection; /** Pointer to the rm3100_object associated with this connection*/
};



RM3100SensorConnection::RM3100SensorConnection(RM3100 &rm3100):rm3100_connection(rm3100){}

void RM3100SensorConnection::init()
{
    rm3100_connection.begin();
}

Vector3f RM3100SensorConnection::getMeasurement()
{
    rm3100_connection.update();
    Vector3f data;
    // Div by 45 to come closer to normalised
    // Convert from NED to ENU
    data << rm3100_connection.getX()/50, -rm3100_connection.getY()/50, -rm3100_connection.getZ()/50;
    // Serial << "RM3100 data: ";
    // displayRowVec(data);
    return data;
}

#endif