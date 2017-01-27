/*! Libraries inclusion */
#include "ros/ros.h"
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include "geometry_msgs/Vector3.h"



/*! Main function */
int main(int argc, char **argv)
{
    /*! Begin ROS in the context of this node */
    ros::init (argc,argv,"imu_node");
	
    /*! Declare node */
    ros::NodeHandle node;
    
    int fd, n;      /*!< Auxiliary variables used in the reading function */
    std::string aux;                    /*!< Refers to the serial port */ 
    geometry_msgs::Vector3 accel;       /*!< Accelerometer raw data */
    geometry_msgs::Vector3 mag;         /*!< Magnetometer raw data */
    geometry_msgs::Vector3 joy;              /*!< Joystick raw data */
    
    float acc_x, acc_y, acc_z, mag_x, mag_y, joy_x, joy_y, push_button;     /*!< Data variables */
    
    
    ros::Publisher pub_accel = node.advertise<geometry_msgs::Vector3>("accel",1);       /*!< Accelerometer data publisher */
    ros::Publisher pub_mag = node.advertise<geometry_msgs::Vector3>("mag",1);           /*!< Magnetometer data publisher */
    ros::Publisher pub_joy = node.advertise<geometry_msgs::Vector3>("joy",1);    /*!< Joystick data publisher */           
    
    ros::Rate loop_rate(50);       /*!< ROS loop rate in Hertz */
     
    /*! Initialize serial communication with Arduino */
	struct termios toptions;
    
	/*! Open serial port */
	node.getParam("imu_node/serial_port",aux);
	ROS_INFO_STREAM(aux);
    fd = open(aux.c_str(), O_RDWR | O_NOCTTY); //opens the file (serial port)
    /*! Wait for the Arduino to reboot */
	usleep(3500000);
	/*! Get current serial port settings */
	tcgetattr(fd, &toptions);
	/*! Set 115200 baud both ways */
	cfsetispeed(&toptions, B115200);
	cfsetospeed(&toptions, B115200);
	/*! 8 bits, no parity, no stop bits */
	toptions.c_cflag &= ~PARENB;
	toptions.c_cflag &= ~CSTOPB;
	toptions.c_cflag &= ~CSIZE;
	toptions.c_cflag |= CS8;
	/*! Canonical mode */
	toptions.c_lflag |= ICANON;
	/*! Commit the serial port settings */
	tcsetattr(fd, TCSANOW, &toptions);
       
    /*! ROS routine */
	while(ros::ok())
	{    
        
        
        /*! If there is an error to open serial port. */
        if(fd < 0)
        {
            ROS_INFO_STREAM("Unable to open serial port!");
        }
        
        char buf[64]="temp text";       /*!< Buffer where the data will be written */
        write(fd, "I\n", 2);        /*!< Keep getting data */ 
		usleep(500);		
        
        
        /*! Read data from serial port */
        do
		{   
            n = read(fd, buf, 64);
        }while (n < 10);
		
            
        /*! Insert terminating zero in the string */
		buf[n] = 0;		
     
        
        /*! Set data to buffer */
		sscanf(buf, "I|%f|%f|%f|%f|%f|%f|%f|%f|*\r\n", &acc_x, &acc_y, &acc_z, &mag_x, &mag_y, &joy_x, &joy_y, &push_button);
		
    
        
        /*! Print data */
        
        ROS_INFO_STREAM(buf);
        ROS_INFO_STREAM("accel_x: " << acc_x);
        ROS_INFO_STREAM("accel_y: " << acc_y);
        ROS_INFO_STREAM("accel_z: " << acc_z);
        ROS_INFO_STREAM("mag_x: " << mag_x);
        ROS_INFO_STREAM("mag_y: " << mag_y);
        ROS_INFO_STREAM("joy_x: " << joy_x); 
        ROS_INFO_STREAM("joy_y: " << joy_y); 
        ROS_INFO_STREAM("push_button: " << push_button); 
        
        
        /*! Set data to Vector3 format to publish */
        accel.x = acc_x;
        accel.y = acc_y;
        accel.z = acc_z;
        mag.x = mag_x;
        mag.y = mag_y;
        joy.x = joy_x;
        joy.y = joy_y;
        joy.z = push_button;
                
        /*! Publish to topics */
        pub_accel.publish(accel);
        pub_mag.publish(mag);
        pub_joy.publish(joy);   
        
        /*! End of ROS routine */
		ros::spinOnce();
		loop_rate.sleep();
	}
}