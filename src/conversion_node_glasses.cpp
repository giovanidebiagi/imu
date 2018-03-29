/*! Libraries inclusion */
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "imu/ADXL345_ros.h"
#include "imu/L3G4200D_ros.h"
#include "std_msgs/Float64.h"
#include "math.h"
#include <unistd.h>

using namespace std;

/*! Data global variables */
double accel_x, accel_y, accel_z;      
double gyro_z;

geometry_msgs::Vector3 joy;     /*!< Joystick variable */

//float teste = 0;

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

void callback_joy(const geometry_msgs::Vector3& msg)
{
    joy.y = msg.y;
    joy.z = msg.z;
}

/*! Main function */
int main(int argc, char** argv)
{
    /*! Begin ROS in the context of this node */
    ros::init(argc, argv, "conversion_node_glasses");
    
    /*! Declare node */
    ros::NodeHandle node;
    
    /*! Data local variables */
    int count = 0;
    int previous_joy_state = 0;     /*!< Auxiliary variable used in gyro/joystick switch routine */

    /*! Boolean variable to choose from controlling yaw rate using either gyroscope or joystick.
    
    joystick -> operation_mode = false
    gyro -> operation_mode = true */
    bool operation_mode = true;

    joy.z = 1;         

    /*! Auxiliary variables used to prevent gyro from reseting the orientation when switched from 
    joystick mode to gyro mode.
        For example, if you are on gyro mode, have 35 degrees of orientation, switch to joystick mode,
    add more 15 degrees with it, and switch back to gyro, measuremet should go back to 35, as long as
    gyroscope measurement does not add previous heading from joystick. Therefore, using these variables,
    previous measurement from joystick is added to gyroscope measurement, so, in the case above, the
    new heading should be 35 + 15 = 50 degrees */
    float previous_heading_glasses = 0, previous_heading_gyro = 0, current_heading_gyro = 0;
    
    std_msgs::Float64 roll, pitch, heading;
    

    roll.data = 0;
    pitch.data = 0;
    heading.data = 0;
    
    ros::Subscriber sub_accel = node.subscribe("accel_glasses", 1, callback_accel);     /*!< Accelerometer data publisher */
    ros::Subscriber sub_gyro = node.subscribe("gyro_glasses", 1, callback_gyro);
    ros::Subscriber sub_joy = node.subscribe("joy_glasses", 1, callback_joy);

    ros::Publisher pub_roll = node.advertise<std_msgs::Float64>("roll_glasses", 1);
    ros::Publisher pub_pitch = node.advertise<std_msgs::Float64>("pitch_glasses", 1);
    ros::Publisher pub_heading = node.advertise<std_msgs::Float64>("heading_glasses", 1);
    
    ros::Rate loop_rate(50);        /*!< ROS loop rate in Hertz */
    
    /*! ROS routine */
    while(ros::ok())
    {
        /*! Counting for gyro heading reseting */
        count++;

        /*! Gyroscope/joystick switch routine */
        if(joy.z == 0 && previous_joy_state == 0)
        {
            if(operation_mode == false) operation_mode = true;
            else if(operation_mode == true) operation_mode = false;
            
            previous_joy_state = 1;
        }

        else if(joy.z == 1)
        {
            previous_joy_state = 0;
        }

        /*! Joystick heading routine */
        if(operation_mode == false)
        {
            ROS_INFO_STREAM("Heading control: Joystick. Press the joystick button to switch.");

            /*! The '< 50' condition refers to the range limit of the servos */
            if(joy.y > 900 && heading.data < 12)
            {
                heading.data = heading.data + 1;
            }

            /*! The '> -50' condition refers to the range limit of the servos */
            else if(joy.y < 200 && heading.data > -12)
            {
                heading.data = heading.data - 1;
            }

        }

        /*! Gyro heading routine */
        else if(operation_mode == true)
        {
	   /*! If gyro mode is activated, the orientation is reseted every 60 seconds to eliminate errors from yaw rate integration */
            if(count > 3000)
            {
                std::cout << "Reseting heading..." << std::endl;
                usleep(4000000);
                gyro.resetHeading();    /*!< Sets the return value 'heading' (the same variable returned in gyro.getHeading()) to 0 */
                count = 0;
                previous_heading_glasses = 0;   /*! Reset the auxiliary variables */
                previous_heading_gyro = 0;      /*! Reset the auxiliary variables */
            }

            ROS_INFO_STREAM("Heading control: Gyroscope. Press the joystick button to switch.");

            current_heading_gyro = gyro.getHeading(gyro_z);

            heading.data = previous_heading_glasses + current_heading_gyro - previous_heading_gyro;
        }


        /*! Keep track of the previous heading data to add it to the new one in the next loop,
        if necessary */
        previous_heading_glasses = heading.data;

        /*! Keep track of gyroscope heading */
        previous_heading_gyro = gyro.getHeading(gyro_z);

        /*! Calculate Roll and Pitch angles */
        roll.data = -accel.getRoll(accel_y, -accel_z);  /*!<  Negative because of the hand axis */
        
        pitch.data = -accel.getPitch(accel_x, accel_y, accel_z);

        std::cout << "Roll: " << pitch.data << "          Pitch: " << roll.data << "         Heading: " << heading.data << std::endl;    
       
        pub_roll.publish(pitch);
        pub_pitch.publish(roll);
        pub_heading.publish(heading);
        
        /*! End of ROS routine */
        ros::spinOnce();
        loop_rate.sleep();
    }
}
