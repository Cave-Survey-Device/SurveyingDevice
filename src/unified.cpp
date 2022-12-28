#include "unified.h"

Vector3d generate_vector(double distance, double heading, double inclination)
{
    Vector3d vector;
    Matrix3d heading_transformation;
    Matrix3d inclination_transformation;

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

void check_heap()
{
    Serial.print("Remaining heap: ");
    Serial.println(xPortGetFreeHeapSize());
    Serial.print("Heap integrity: ");
    Serial.println(heap_caps_check_integrity_all(true));
}