#include "utility_csd.h"
#include <stdarg.h>

void displayMat(const MatrixXf &m)
{
    int rows = m.rows();
    int cols = m.cols();
    for(int i=0; i<rows;i++)
    {
        Serial << "[\t";
        for(int j=0; j<cols;j++)
        {
            Serial.print(m(i,j),8);
            Serial.print("\t");
        }
        Serial << "\t]\n";
    }
    Serial << "    \n";
}

void displayVec(const VectorXf &v)
{
    int n = v.size();
    for(int i=0; i<n;i++)
    {
        Serial.print("[\t");
        Serial.print(v(i),8);
        Serial.print("\t");
        Serial.print("\t]\n");
    }
    Serial << "\n";
}

void displayRowVec(const VectorXf &v)
{
    int n = v.size();
    for(int i=0; i<n;i++)
    {
        Serial.print("[\t");
        Serial.print(v(i),8);
        Serial.print("\t");
    }
    Serial << "]\n";
}

void serialPlotVec(const VectorXf &v1, const VectorXf &v2)
{
    //assert ((v1.size() == v2.size()), "vector1.size() == vector2.size() ");

    int n = v1.size();
    for(int i=0; i<n;i++)
    {
        Serial << "Var1:";
        Serial.print(v1(i),8);
        Serial << ",Var2:";
        Serial.print(v2(i),8);
        Serial << "\n";
    }
    Serial << "\n";
    
}

void serialPlotVec(const VectorXf &v1, const VectorXf &v2, const char* v1_name , const char* v2_name)
{
    //assertm ((v1.size() == v2.size()), "vector1.size() == vector2.size() ");

    int n = v1.size();
    for(int i=0; i<n;i++)
    {
        Serial << v1_name << ":";
        Serial.print(v1(i),8);
        Serial << "," << v2_name << ":";
        Serial.print(v2(i),8);
        Serial << "\n";
    }
    Serial << "\n";
}

float Deg2Rad(float degrees) {
    return degrees * (3.14159265359 / 180.0);
}

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
    Vector3f hir;
    // Returns heading and inclination
    // https://arduino.stackexchange.com/a/88707
    // https://www.analog.com/en/app-notes/an-1057.html equation (11)
	float inclination =  atan2(g(0),pow(pow(g(1),2) + pow(g(2),2),0.5));
    float roll = atan2(g(1),pow(pow(g(0),2) + pow(g(2),2),0.5));
    float heading = atan2(m(0),pow(pow(m(1),2) + pow(m(2),2),0.5));
    // // Project magnetic vector onto horizontal plane
    // Vector3f vector_north = m - ((m.dot(g) / g.dot(g)) * g);

    // float heading =  atan2(vector_north(0), vector_north(1));
    // if (g(2) < 0)
    // {
    //     heading = heading * -1;
    // }
    hir << RAD_TO_DEG * heading, RAD_TO_DEG * inclination, RAD_TO_DEG * roll;
    return hir;
}

// Shifts all zero-valued columns to the end of the matrix and return the number of zero-valued columns
int removeNullData(Ref<MatrixXf> mat)
{
    Serial << "Remove null data\n";
    // Creates a map to the incoming data to prevent a copy being needed. A map just holds the information about how and where the data is stored. It can be interfaced with just like any other Eigen object.
    // Eigen::Map<Matrix<float,3,-1>> mat(data_ptr,3,size);

    // Initialise blank cols mat to -1
    VectorXi blank_cols(mat.cols());
    blank_cols.setOnes();
    blank_cols *= -1;
    int index = 0;
    int max_index = 0;

    int i;
    Serial << "Finding null indexes\n";
    // Index zero values in reverse order
    for (int i=mat.cols()-1; i>-1; i--)
    {
        if (mat.col(i).norm() == 0)
        {
            blank_cols(index) = i;
            index++;
        }
    }
    max_index = index;

    // Push index back due to index++ happening AFTER assignment
    index--;
    if (index == -1)
    {
        Serial << "No zeroes found\n";
        return 0;
    }
    
    Serial << "Sorting data\n";

    // Iterate in reverse through matrix, replacing zero valued sections with non-zero valued elements nearest the end of the matrix, replacing those with zero
    for (int i=mat.cols()-1; i>-1; i--)
    {
        // Check if value is non-zero
        if (mat.col(i).norm() > 0)
        {
            // Replace zero value closest to start or matrix with non-zero value
            mat.col(blank_cols(index)) = mat.col(i);
            // Replace non-zero value with zero
            mat.col(i) << 0, 0, 0;
            // Decrease index
            index--;

            // If all zero-valued sections have been replaced, break
            if (index < 0)
            {
                break;
            }
        }
    }
    Serial << "Finished removing null data\n";

    return max_index;
}