#ifndef HEADER_LASER_CSD
#define HEADER_LASER_CSD

class LaserSensor
{
public:
    /**
     * @brief Initialise laser module
     * 
     */
    virtual void begin()=0;

    /**
     * @brief Get laser mesaurement
     * 
     * @return float 
     */
    virtual float getMeasurement()=0;

    /**
     * @brief Toggle laser
     * 
     * @param mode 
     */
    virtual void toggleLaser(bool mode)=0;
};

#endif