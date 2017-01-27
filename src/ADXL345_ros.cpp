#include "imu/ADXL345_ros.h"

//!A constructor
ADXL345::ADXL345(){};

//!A destructor
ADXL345::~ADXL345(){}; 
/*!
\param Yg Acceleration data in 'g's related to Y axis.
\param Zg Acceleration data in 'g's related to Z axis.
\return Accelerometer Roll angle.
*/
double ADXL345::getRoll(double Yg, double Zg)
{
    // Roll calculation
    double roll = atan2(Yg,Zg)*180/M_PI;
    
    return roll;
}

/*!
\param Xg Acceleration data in 'g's related to X axis.
\param Yg Acceleration data in 'g's related to Y axis.
\param Zg Acceleration data in 'g's related to Z axis.
 */
double ADXL345::getPitch(double Xg, double Yg, double Zg)
{
    // Auxiliary variable to calculate the pitch angle
    double YZg = sqrt(Yg*Yg + Zg*Zg);                  


    //Pitch calculation
    double pitch = atan2(-Xg,YZg)*180/M_PI;

    return pitch;
}





