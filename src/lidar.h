#ifndef HEADER_LIDAR
#define HEADER_LIDAR

class Lidar {
    public:
        // Initialise lidar module
        virtual void init()=0;

        // Get lidar mesaurement
        virtual double get_measurement()=0;

        // Toggle laser
        virtual void toggle_laser()=0;
};

#endif