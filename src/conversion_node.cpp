/*! Libraries inclusion */
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "imu/ADXL345_ros.h"
#include "std_msgs/Float64.h"
#include "math.h"

double accel_x, accel_y, accel_z, mag_x, mag_y, mag_z;      /*!< Data global variables */

//-----------------------------------
float hDeg1;
//-----------------------------------

ADXL345 accel;      /*!< Accelerometer object */

/*! Accelerometer callback function */
void callback_accel(const geometry_msgs::Vector3& msg)
{
    accel_x = msg.x;
    accel_y = msg.y;
    accel_z = msg.z;
}

/*! Magnetometer callback function */
void callback_mag(const geometry_msgs::Vector3& msg)
{
    mag_x = msg.x;
    mag_y = msg.y;
    mag_z = 0;
}

/*! Main function */
int main(int argc, char** argv)
{
    /*! Begin ROS in the context of this node */
    ros::init(argc, argv, "conversion_node");
    
    /*! Declare node */
    ros::NodeHandle node;
    
    //double roll, pitch, heading, headingDegrees;        /*!< Data local variables */
    std_msgs::Float64 roll, pitch, heading, headingDegrees;  
    
    ros::Subscriber sub_accel = node.subscribe("accel", 1, callback_accel);     /*!< Accelerometer data publisher */
        
    ros::Subscriber sub_mag = node.subscribe("mag", 1, callback_mag);       /*!< Magnetometer data publisher */
    
    //-------------------
    ros::Publisher pub_roll = node.advertise<std_msgs::Float64>("roll", 1);
    
    ros::Publisher pub_pitch = node.advertise<std_msgs::Float64>("pitch", 1);
    
    ros::Publisher pub_yaw = node.advertise<std_msgs::Float64>("yaw", 1);
    //-------------------
    
    /*!
        Declination angle for Sao Carlos-SP, Brazil
        Found at http://www.magnetic-declination.com/
    */
    float declinationAngle = 0.36;
    
    ros::Rate loop_rate(50);        /*!< ROS loop rate in Hertz */
    
    /*! ROS routine */
    while(ros::ok())
    {
        /*! Calculate Roll and Pitch angles */
        roll.data = -accel.getRoll(accel_y, -accel_z);  /*!<  Negative because of the hand axis */
        pitch.data = -accel.getPitch(accel_x, accel_y, accel_z);

        /*! Calculate Yaw angle */
        heading.data = atan2(mag_y, mag_x);
        heading.data += declinationAngle;
        
        /*if(heading.data < -M_PI)
        {
            heading.data = heading.data + 2*M_PI;    
        }
        
    
        else if(heading.data >  M_PI)
        {
            heading.data = heading.data - 2*M_PI;
        }*/
        
        // Correct for when signs are reversed.
        if(heading.data < 0)
        heading.data += 2*M_PI;
    
        // Check for wrap due to addition of declination.
        if(heading.data > 2*M_PI)
        heading.data -= 2*M_PI;
        
        headingDegrees.data = heading.data * 180/ M_PI;
        
        /*!
            Print data
            Roll and pitch are inverted because the roll and pitch angles for the accelerometer are different from the ones for the hand of the user, that is, the IMU was soldered in a way that itss X axis is perpendicular to the glove X axis
        */
        std::cout << "Roll: " << pitch.data << "          Pitch: " << roll.data << "         Yaw: " << headingDegrees.data << std::endl;
        
        pub_roll.publish(pitch);
        pub_pitch.publish(roll);
        pub_yaw.publish(headingDegrees);
        
        /*! End of ROS routine */
        ros::spinOnce();
        loop_rate.sleep();
    }
}
