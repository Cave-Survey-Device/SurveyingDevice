#include <SensorHandler.h>
#include <OLED.h>
#include "RM3100SensorConnection.h"
#include "SCA3300SensorConnection.h"
#include "LDK2MSensorConnection.h"
#include "utils.h"

#include <ArduinoJson.h>


static RM3100 rm3100;
static SCA3300 sca3300;
static LDK_2M ldk2m;

static SCA3300SensorConnection sc_accelerometer(sca3300);
static RM3100SensorConnection sc_magnetometer(rm3100);
static LDK2MSensorConnection sc_laser(ldk2m);

static SensorHandler sh(sc_accelerometer, sc_magnetometer, sc_laser);

static OLED oled;

String strcmd;
const char* cmd;
void serialEvent()
{
  // cmd = Serial.parseInt();
  Serial << "\nserialEvent\n";
  strcmd = Serial.readStringUntil('\n');
  Serial << "strcmd: " << strcmd << "\n";
}

// void addJSONArr(const char * name, JsonArray &arr, Ref<MatrixXf> data){
//     for 
// }

typedef Matrix<float, Dynamic, Dynamic, RowMajor> RowMatrixXf;
static StaticJsonDocument<60480> root;
static float eig_arr[3][N_ALIGN_MAG_ACC];
static float las_arr[3][N_LASER_CAL];


int n;
Vector3f orientation;
void mainloop(void *){
while(true){
    cmd = strcmd.c_str();

    if (!strcmp(cmd, "1")) { // Collect static alignment data

        Serial << "Collect static calibration data...\n";
        n = sh.collectStaticCalibData();
        Serial << "Progress: " << n << "\n";
        if (n >= N_ORIENTATIONS){ sh.calibrate();}
        strcmd = "";

    } else if (!strcmp(cmd, "3")) { // Collect laser alignment data

        Serial << "Collect laser calibration data\n";
        n = sh.collectLaserCalibData();
        Serial << "Progress: " << n << "\n";
        strcmd = "";
        delay(100);
        ldk2m.toggleLaser(true);

    } else if (!strcmp(cmd, "align")) { // align the device

        Serial << "align\n";
        sh.align();
        strcmd = "";

    } else if (!strcmp(cmd, "calibrate")) { // calibrate the device

        Serial << "calibrate\n";
        sh.calibrate();
        sh.staticAlign();
        strcmd = "";

    } else if (!strcmp(cmd, "shot")) { // Collect inertial alignment data

        Serial << "shot\n";
        sh.takeShot();

        Serial.printf("Shot measurement: %f %f\n", sh.getCardan()(0), sh.getCardan()(1));
        Vector3f meas = sh.getFinalMeasurement();
        Serial.printf("Final measurement: %f %f %f\n", meas(0), meas(1), meas(2));
        strcmd = "";


    } else if (!strcmp(cmd, "erase")) { // Collect inertial alignment data

        Serial << "erase\n";
        sh.eraseFlash();
        strcmd = "";
    } else if (!strcmp(cmd, "flashstats")) { // Collect inertial alignment data

        Serial << "flashstats\n";
        sh.getFlashStats();
        strcmd = "";

    } else if (!strcmp(cmd, "las")) { // Collect inertial alignment data

        Serial << "las\n";
        ldk2m.toggleLaser();
        strcmd = "";

    } else if (!strcmp(cmd, "2")) { // Output calibration data and parameters as JSON to serial

        DeviceCalibrationParameters parm_data = sh.getCalibParms();
        
        root.clear();

        JsonArray jarr;
        MatrixXf static_acc_samples, static_mag_samples, laser_acc_samples, laser_mag_samples;

        JsonArray acc_data = root.createNestedArray("static_acc_samples");
        static_acc_samples = sh.getStaticCalibData().acc_data;
        Map<RowMatrixXf>(&eig_arr[0][0], 3, N_ALIGN_MAG_ACC) = static_acc_samples; 
        copyArray(eig_arr,acc_data);
        Serial.print("static_acc_samples\n");
        // serializeJson(acc_data,Serial);

        JsonArray mag_data = root.createNestedArray("static_mag_samples");
        static_mag_samples = sh.getStaticCalibData().mag_data;
        Map<RowMatrixXf>(&eig_arr[0][0], 3, N_ALIGN_MAG_ACC) = static_mag_samples; 
        copyArray(eig_arr,mag_data);
        Serial.print("static_mag_samples\n");
        // serializeJson(mag_data,Serial);

        JsonArray las_acc_data = root.createNestedArray("laser_acc_samples");
        laser_acc_samples = sh.getLaserCalibData().acc_data;
        Map<RowMatrixXf>(&las_arr[0][0], 3, N_LASER_CAL) = laser_acc_samples; 
        copyArray(las_arr,las_acc_data);
        Serial.print("laser_acc_samples\n");
        // serializeJson(las_acc_data,Serial);

        JsonArray las_mag_data = root.createNestedArray("laser_mag_samples");
        laser_mag_samples = sh.getLaserCalibData().mag_data;
        Map<RowMatrixXf>(&las_arr[0][0], 3, N_LASER_CAL) = laser_mag_samples; 
        copyArray(las_arr,las_mag_data);
        Serial.print("laser_mag_samples\n");
        serializeJson(las_mag_data,Serial);
        Serial << "\n\n";
        


        
        // JsonObject parms = root.createNestedObject("parms");
        JsonArray Ra_static = root.createNestedArray("Ra_static");
        JsonArray ba_static = root.createNestedArray("ba_static");
        JsonArray Rm_static = root.createNestedArray("Rm_static");
        JsonArray bm_static = root.createNestedArray("bm_static");
        JsonArray Ra_laser = root.createNestedArray("Ra_laser");
        JsonArray Rm_laser = root.createNestedArray("Rm_laser");
        JsonArray Rm_align = root.createNestedArray("Rm_align");


        float vec3fdata[3];
        float mat3fdata[3][3];

        Map<RowMatrixXf>(&mat3fdata[0][0], 3, 3) = parm_data.Ra_cal; 
        copyArray(mat3fdata,Ra_static);
        Map<Vector3f>(&vec3fdata[0], 3) = parm_data.ba_cal; 
        copyArray(vec3fdata,ba_static);
        

        Map<RowMatrixXf>(&mat3fdata[0][0], 3, 3) = parm_data.Rm_cal; 
        copyArray(mat3fdata,Rm_static);
        Map<Vector3f>(&vec3fdata[0], 3) = parm_data.bm_cal; 
        copyArray(vec3fdata,bm_static);


        Map<RowMatrixXf>(&mat3fdata[0][0], 3, 3) = parm_data.Ra_las; 
        copyArray(mat3fdata,Ra_laser);

        Map<RowMatrixXf>(&mat3fdata[0][0], 3, 3) = parm_data.Rm_las; 
        copyArray(mat3fdata,Rm_laser);


        Map<RowMatrixXf>(&mat3fdata[0][0], 3, 3) = parm_data.Rm_align; 
        copyArray(mat3fdata,Rm_align);

        serializeJson(root,Serial);

        Serial << "Inclination angle: " << parm_data.inclination_angle << "\n";
   

        Serial << "\n\n";
        strcmd = "";

    } else if (!strcmp(cmd, "save")) { // Save calibration parameters
        Serial << "Save\n";
        sh.saveCalibration();
        strcmd = "";


    } else if (!strcmp(cmd, "load")) { // Load calibration parameters
        Serial << "Load\n";
        sh.loadCalibration();
        strcmd = "";


    } else if (!strcmp(cmd, "reset")) { // Reset calibration parameters
        Serial << "Reset\n";
        sh.resetCalibration();
        strcmd = "";


    } else if (!strcmp(cmd, "mag")) { // Reset calibration parameters
        Serial << "Magnetometer: " << sh.getMagData()(0) << " " << sh.getMagData()(1) << " " << sh.getMagData()(2) << "\n";
        Serial.printf("Mag norm: %f\n",sh.getMagData().norm());
        sh.correctData();
        Serial.printf("Mag corrceted norm: %f\n", sh.getMagData().norm());

        strcmd = "";


    } else if (!strcmp(cmd, "acc")) { // Reset calibration parameters
        Serial << "Accelerometer: " << sh.getAccData()(0) << " " << sh.getAccData()(1) << " " << sh.getAccData()(2) << "\n";
        strcmd = "";

    } else if (!strcmp(cmd, "card")) { // Reset calibration parameters
        Serial << "Cardan: " << sh.getCardan()(0) << " " << sh.getCardan()(1) << " " << sh.getCardan()(2) << "\n";
        strcmd = "";


    } else {
        // Serial << "Update OLED\n";
        delay(50);
        // sh.takeShot(false,false);
        sh.update();
        orientation = sh.getCardan();
        oled.Clino(-RAD_TO_DEG * orientation(1));
        oled.Compass(-RAD_TO_DEG * orientation(0));
    }  

    cmd = "";
}
}


void setup() {
    Serial.begin(115200);
    Serial << "Begin beaning\n";
    sh.init();
    Serial << "SensorHandler initialised!\n";
    oled.Initialise();

    Serial.printf("Mag data: %f %f %f\n",sc_magnetometer.getMeasurement()(0),sc_magnetometer.getMeasurement()(1),sc_magnetometer.getMeasurement()(2));
    Serial.printf("Aacc data: %f %f %f\n",sc_accelerometer.getMeasurement()(0),sc_accelerometer.getMeasurement()(1),sc_accelerometer.getMeasurement()(2));

    TaskHandle_t hardware_handle;
    xTaskCreatePinnedToCore(
        mainloop, /* Function to implement the task */
        "main", /* Name of the task */
        50000,  /* Stack size in words */
        NULL,  /* Task input parameter */
        tskIDLE_PRIORITY ,  /* Priority of the task */
        &hardware_handle,  /* Task handle. */
        0); /* Core where the task should run */

}


void loop(){}



