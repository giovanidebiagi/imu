/*! Libraries inclusion */
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "imu/ADXL345_ros.h"
#include "imu/L3G4200D_ros.h"
#include "std_msgs/Float64.h"
#include "math.h"
#include <unistd.h>

/*! Data global variables */
double accel_x, accel_y, accel_z;      
double gyro_z;
int count = 0;          /*!< Counter variable for gyro error reseting */

ADXL345 accel;      /*!< Accelerometer object */
L3G4200D gyro;      /*!< Gyroscope object */

/*! Accelerometer callback function */
void callback_accel(const geometry_msgs::Vector3& msg)
{
    accel_x = msg.x;
    accel_y = msg.y;
    accel_z = msg.z;
}

/*! Gyroscope callback function */
void callback_gyro(const geometry_msgs::Vector3& msg)
{
    gyro_z = msg.z;
}

/*! Main function */
int main(int argc, char** argv)
{
    /*! Begin ROS in the context of this node */
    ros::init(argc, argv, "conversion_node");
    
    /*! Declare node */
    ros::NodeHandle node;
    
    std_msgs::Float64 roll, pitch, heading;  
    
    ros::Subscriber sub_accel = node.subscribe("accel", 1, callback_accel);     /*!< Accelerometer data publisher */
    ros::Subscriber sub_gyro = node.subscribe("gyro", 1, callback_gyro);

    ros::Publisher pub_roll = node.advertise<std_msgs::Float64>("roll", 1);
    ros::Publisher pub_pitch = node.advertise<std_msgs::Float64>("pitch", 1);
    ros::Publisher pub_heading = node.advertise<std_msgs::Float64>("heading", 1);
    
    ros::Rate loop_rate(50);        /*!< ROS loop rate in Hertz */
    
    /*! ROS routine */
    while(ros::ok())
    {
        /*! Calculate Roll and Pitch angles */
        roll.data = -accel.getRoll(accel_y, -accel_z);  /*!<  Negative because of the hand axis */
        
        pitch.data = -accel.getPitch(accel_x, accel_y, accel_z);
        
        heading.data = gyro.getHeading(gyro_z);

        count++;

        /*! If gyro mode is activated, the orientation is reseted every 20 seconds to eliminate errors from yaw rate integration */
        if(count > 3000)
        {
            std::cout << "Reseting heading..." << std::endl;
            usleep(4000000);
            gyro.resetHeading();
            count = 0;
        }

        std::cout << "Roll: " << pitch.data << "          Pitch: " << roll.data << "         Heading: " << heading.data << std::endl;    
       
        pub_roll.publish(pitch);
        pub_pitch.publish(roll);
        pub_heading.publish(heading);
        
        /*! End of ROS routine */
        ros::spinOnce();
        loop_rate.sleep();
    }
}
