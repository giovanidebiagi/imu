/*! Libraries inclusion */
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "imu/ADXL345_ros.h"
#include "imu/L3G4200D_ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "math.h"
#include <unistd.h>

/*! Data global variables */
double accel_x = 0, accel_y = 0, accel_z = 0;      
double gyro_z = 0;
int count = 0;          /*!< Counter variable for gyro error reseting */

ADXL345 accel;      /*!< Accelerometer object */
L3G4200D gyro;      /*!< Gyroscope object */

std_msgs::Bool operation_mode, previous_operation_mode;  /*!< Used to check if the operation mode is gyro mode, and if it is, reset heading every 60s */

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

void callback_operation_mode(const std_msgs::Bool& msg)
{
    previous_operation_mode.data = operation_mode.data;
    operation_mode.data = msg.data;

    /*! Everytime the operation mode changes, count is reseted */
    if(previous_operation_mode.data != operation_mode.data)
    {
        count = 0;
    }
}

/*! Main function */
int main(int argc, char** argv)
{
    /*! Begin ROS in the context of this node */
    ros::init(argc, argv, "conversion_node");
    
    /*! Declare node */
    ros::NodeHandle node;
    
    std_msgs::Float64 roll, pitch, heading;

    /*!< Initiate variables with 0 */
    // roll.data = 0;
    // pitch.data = 0;
    // heading.data = 0;  
    
    /*!< This is used to make the attitude_decision nodes aware that the measurements have been reseted, allowing them to recalculate the relative values measurements */
    std_msgs::Bool reset_flag;
    reset_flag.data = false;
    
    ros::Subscriber sub_accel = node.subscribe("accel", 1, callback_accel);
    ros::Subscriber sub_gyro = node.subscribe("gyro", 1, callback_gyro);
    ros::Subscriber sub_operation_mode = node.subscribe("/operation_mode", 1, callback_operation_mode);

    ros::Publisher pub_roll = node.advertise<std_msgs::Float64>("roll", 1);
    ros::Publisher pub_pitch = node.advertise<std_msgs::Float64>("pitch", 1);
    ros::Publisher pub_heading = node.advertise<std_msgs::Float64>("heading", 1);
    ros::Publisher pub_reset = node.advertise<std_msgs::Bool>("reset", 1); /*!< This is used to make the attitude_decision nodes aware that the measurements have been reseted, allowing them to recalculate the relative values measurements */
    
    ros::Rate loop_rate(50);        /*!< ROS loop rate in Hertz */
    
    /*! Using launch file, it is necessary that this node waits for the imu_node to open
         the serial port and start publishing data, otherwise the conversion_node will have
         garbage values and attitude_decision nodes will abort operation */
    usleep(5000000);

    /*! ROS routine */
    while(ros::ok())
    {
        /*! Calculate Roll and Pitch angles */
        roll.data = -accel.getRoll(accel_y, -accel_z);  /*!<  Negative because of the hand axis */
        
        pitch.data = -accel.getPitch(accel_x, accel_y, accel_z);
        
        heading.data = gyro.getHeading(gyro_z);

        count++;

        /*! If gyro mode is activated, the orientation is reseted every 60 seconds to eliminate errors from yaw rate integration */
        if(operation_mode.data == true && count > 3000)
        {   
            std::cout << "Reseting heading..." << std::endl;
            // usleep(4000000);

            /*! Alternative to usleep so that its always sending data values 0, instead of
            sending nothing */
            pitch.data=0;
            roll.data=0;
            heading.data=0;

            /*! Wait for aprox 4 sec */
            for(int k=0; k<400000; k++)
            {   
                reset_flag.data = true;
                pub_reset.publish(reset_flag); /*! This is used to make the attitude_decision nodes aware that the measurements have been reseted, allowing them to recalculate the relative values measurements */
                pub_roll.publish(pitch);
                pub_pitch.publish(roll);
                pub_heading.publish(heading);
            }

            reset_flag.data = false;
            pub_reset.publish(reset_flag); /*! This is used to make the attitude_decision nodes aware that the measurements have been reseted, allowing them to recalculate the relative values measurements */

            /*--------------------*/

            gyro.resetHeading();
            count = 0;
        }

        reset_flag.data = false; /*! This is used to make the attitude_decision nodes aware that the measurements have been reseted, allowing them to recalculate the relative values measurements */

        std::cout << "Roll: " << pitch.data << "          Pitch: " << roll.data << "         Heading: " << heading.data << std::endl;    
       
        pub_roll.publish(pitch);
        pub_pitch.publish(roll);
        pub_heading.publish(heading);
        
        /*! End of ROS routine */
        ros::spinOnce();
        loop_rate.sleep();
    }
}
