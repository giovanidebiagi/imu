/*! \file ADXL345.h
    \brief Accelerometer library header file.
*/

#ifndef ADXL345_H
#define ADXL345_H

#include <cmath>

//!Class for the accelerometer ADXL345.
class ADXL345
{
    public:
     //!A constructor.
     ADXL345();                           
     //!A destructor.
     ~ADXL345();      
     //! A function which takes acceleration data in 
     double getRoll(double Yg, double Zg);
     //!A function which takes acceleration data in axis 
     double getPitch(double Xg, double Yg, double Zg);
     
};
                                            
                                            
#endif
