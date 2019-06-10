#include "imu/L3G4200D_ros.h"
#include <iostream>
#include <unistd.h>		/*!< Time library to use usleep() function */

using namespace std;

/*!< Constructor */
L3G4200D::L3G4200D()
{
	heading = 0;
	headingDegrees = 0;
	dt = 0.02;
}

L3G4200D::~L3G4200D(){};    /*!< Destructor */

double L3G4200D::getHeading(double gyro_z)
{
	gyro_z *= 0.061;

	/*! Noise reduction conditional structure */
	if(gyro_z > -5 && gyro_z < 5)
	{
		gyro_z = 0;
	} 
	

    heading = heading + gyro_z*dt;

	/*! To obtain heading in degrees, we have to multiply the result by a constant value according to the scale adopted in gyro setup.
		For scale = 2000deg/s and 16-bit gyro:
			-2000 -> -32768
			 2000 ->  32767
		Therefore, the constant value is 2000/32767 = 0.061
	*/

    return heading;
}

void L3G4200D::resetHeading()
{
	heading = 0;
}