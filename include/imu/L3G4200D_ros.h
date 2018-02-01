/*! \file L3G4200D_ros.h
    \brief Gyroscope library whithin ROS header file
*/

#ifndef L3G4200D_H
#define L3G4200D_H

#include "ros/ros.h"

class L3G4200D
{
    public:

        L3G4200D();     /*!< Constructor */
        ~L3G4200D();    /*!< Destructor */

        double heading, headingDegrees, dt;

        double getHeading(double gyro_z);   /*!< Function to calculate Heading */

        void resetHeading();              /*!< Functino to reset heading data */
};

#endif