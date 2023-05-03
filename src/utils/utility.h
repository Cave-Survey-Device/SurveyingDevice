#ifndef HEADER_UTILITY
#define HEADER_UTILITY

#include <ArduinoEigenDense.h>

#include "config.h"

using namespace Eigen;

/**
 * @brief A node to be saved. Id is not needed but could be used later if the device to to be used standalone.
 * 
 */
struct node
{
    int id;
    float inclination;
    float heading;
    float distance;
};

template<class T> inline Print &operator <<(Print &obj, T arg)  // no-cost stream operator as described at http://arduiniana.org/libraries/streaming/
{
    obj.print(arg);
    return obj;
}

// Generate vector from current node back to base
/**
 * @brief Generates a vector from distance, heading, inclination
 * TODO: maybe convert to HID like all other funcs, or change all funct to DHI.
 * 
 * @param distance 
 * @param heading 
 * @param inclination 
 * @return Vector3f 
 */
Vector3f generate_vector(float distance, float heading, float inclination);

/**
 * @brief Pretty debug for disto
 * 
 * @param mode 
 * @param str 
 */
void debug(unsigned int mode, const char* str);

/**
 * @brief Debug with functionality of printf. Very nice :).
 * 
 * @param mode 
 * @param format 
 * @param ... 
 */
void debugf(unsigned int mode, const char *format, ...);

Vector3f Cartesian(Vector3f spherical);

/**
 * @brief Converts cartesian data to cartesian data (X, Y, Z) to (H, I, D)
 * 
 * @param cartesian Cartesian data XYZ
 * @return Vector3f Spherical data HID
 */
Vector3f Spherical(Vector3f cartesian);

/**
 * @brief Finds the orientation of the device from gravitational and magnetic data.
 * 
 * @param g Gravitationl data
 * @param m Magnetic data
 * @return Vector3f 
 */
Vector3f Orientation(Vector3f g, Vector3f m);

#endif