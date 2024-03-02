#ifndef HEADER_LDK2MSensorConnection
#define HEADER_LDK2MSensorConnection

#include <CaveSurveyDevice.h>
#include <LDK_2M.h>

class LDK2MSensorConnection : public Laser
{
public:
    LDK2MSensorConnection(LDK_2M &ptr_LDK2M);
    void init();
    float getMeasurement();
    void toggleLaser();
    void toggleLaser(bool mode);
private:
    LDK_2M &LDK2M_connection;
};

LDK2MSensorConnection::LDK2MSensorConnection(LDK_2M &ldk2m): LDK2M_connection(ldk2m){}

void LDK2MSensorConnection::init()
{
    LDK2M_connection.init();
}

float LDK2MSensorConnection::getMeasurement()
{
    return LDK2M_connection.getMeasurement();
}

void LDK2MSensorConnection::toggleLaser()
{
    LDK2M_connection.toggleLaser();
}
void LDK2MSensorConnection::toggleLaser(bool mode)
{
    LDK2M_connection.toggleLaser(mode);
}








#endif