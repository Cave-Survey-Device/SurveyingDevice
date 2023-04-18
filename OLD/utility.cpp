#include "utility.h"

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


void check_heap()
{
    Serial.print("Remaining heap: ");
    Serial.println(xPortGetFreeHeapSize());
    Serial.print("Heap integrity: ");
    Serial.println(heap_caps_check_integrity_all(true));
}

void debug(unsigned int mode, const char* str)
{
    if (DEBUG_BOOL_ARR[(int)mode])
    {
        Serial.printf("%s: %s\n",DEBUG_STR_ARR[(int)mode],str);
    }
}

// Returns inclination and heading in a vector2d
Vector2d get_inclination_heading(Vector3f true_vec)
{
    Vector3f z_axis;
    Vector2d out;
    float dot_prod;
    float scaling;

    z_axis << 0,1,0;
    dot_prod = true_vec.dot(z_axis);
    scaling = z_axis.norm() * true_vec.norm();


    out[0] = RAD_TO_DEG * acos(scaling);

	out[1] = RAD_TO_DEG * atan2(true_vec(1), true_vec(0));

	return out;
}

