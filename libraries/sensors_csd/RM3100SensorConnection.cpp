#include "RM3100SensorConnection.h"


RM3100SensorConnection::RM3100SensorConnection(RM3100* ptr_rm3100)
{
    this->rm3100_connection = ptr_rm3100;
}

Vector3f RM3100SensorConnection::GetRawData()
{
    rm3100_connection->update();
    Eigen::Vector3f data;
    // Div by 45 to come closer to normalised
    data << rm3100_connection->getX()/45., rm3100_connection->getY()/45, rm3100_connection->getZ()/45;
    return data;
}