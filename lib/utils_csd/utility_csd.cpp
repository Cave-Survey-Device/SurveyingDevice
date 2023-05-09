#include "utility_csd.h"
#include <stdarg.h>



Vector3f generate_vector(float distance, float heading, float inclination)
{
    Vector3f vector;
    Matrix3f heading_transformation;
    Matrix3f inclination_transformation;

    vector << 1, 0, 0;

    // Rotate about z axis to apply heading
    heading_transformation << cos(heading), sin(heading), 0,
                              sin(heading), cos(heading), 0,
                              0         , 0         , 1;

    // Rotate about y axis to apply inclincation
    inclination_transformation << cos(inclination), 0, sin(inclination),
                              0         , 1,          0,
                              sin(inclination), 0, cos(inclination);

    vector = inclination_transformation*heading_transformation*vector;

    return -vector;
}

void debug(unsigned int mode, const char* str)
{
    if ((int)mode == 0 || (DEBUG_BOOL_ARR[(int)mode] && sizeof(str) < 250*sizeof(char)))
    {
        char buffer[250+6];
        sprintf(buffer, "%s: %s\n",DEBUG_STR_ARR[(int)mode],str);
        Serial.print(buffer);
    }
}

void debugf(unsigned int mode, const char *format, ...)
{
    if ((int)mode == 0 || (DEBUG_BOOL_ARR[(int)mode] && sizeof(format) < 250*sizeof(char)))
    {
        char buffer[250+6];
        va_list args;

        sprintf(buffer, "%s: ",DEBUG_STR_ARR[(int)mode]);
        Serial.print(buffer);

        vsprintf(buffer, format, args);
        Serial.print(buffer);

        Serial.print("\n");

    }
}


Vector3f Cartesian(Vector3f spherical)
{
    Vector3f cartesian;
    // Serial.printf("Heading: %f   Inclination: %f\n", spherical(0), spherical(1));
    cartesian << spherical(2)*(sin(spherical(1))*cos(spherical(0))),
                 spherical(2)*(sin(spherical(1))*sin(spherical(0))),
                 spherical(2)*(cos(spherical(1)));
    return cartesian;
}

Vector3f Spherical(Vector3f cartesian){
    Vector3f spherical;
    spherical << atan2(cartesian(1), cartesian(0)),
                 atan2(pow( pow(cartesian(0),2) + pow(cartesian(1),2), 0.5),cartesian(2)),
                 cartesian.norm();
    return spherical;
}

Vector3f Orientation(Vector3f g, Vector3f m)
{    
    Vector3f hi;
    // Returns heading and inclination
    // https://arduino.stackexchange.com/a/88707
    // https://www.analog.com/en/app-notes/an-1057.html equation (11)
	float inclination =  atan2(g(0),pow(pow(g(1),2) + pow(g(2),2),0.5));
    float roll = atan2(g(1),pow(pow(g(0),2) + pow(g(2),2),0.5));

    // Project magnetic vector onto horizontal plane
    Vector3f vector_north = m - ((m.dot(g) / g.dot(g)) * g);

    float heading =  atan2(vector_north(1), vector_north(0));
    if (g(2) < 0)
    {
        heading = heading * -1;
    }
    hi << heading, inclination, roll;
    return hi;
}