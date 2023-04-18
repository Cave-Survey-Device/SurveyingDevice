#include "accelerometer.h"


class SCA3300: public Accelerometer
{
public:
    SCA3300();
protected:
    void get_raw_data();

};