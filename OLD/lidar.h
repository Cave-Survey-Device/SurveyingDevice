#ifndef HEADER_LIDAR
#define HEADER_LIDAR

// Abstract class to allow usage of different sensors
class Lidar {
    public:
        // Initialise lidar module
        virtual void init()=0;

        // Get lidar mesaurement
        virtual float get_measurement()=0;

        // Toggle laser
        virtual void toggle_laser()=0;
};

#endif