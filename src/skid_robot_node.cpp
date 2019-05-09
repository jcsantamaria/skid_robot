#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

#include <sstream>
#include <iomanip>

#include "BNO080_I2C.h"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "skid_robot");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    /**
     * Publish imu sensor
     */
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu/data", 1000);

    /**
     * Frequency
     */
    ros::Rate loop_rate(50);

    /**
     * BNO080 initialization
     */
    BNO080 imu;
    imu.enableDebugging();
    imu.begin();
    imu.enableAccelerometer(18);

    while (ros::ok())
    {
        if (imu.dataAvailable())
        {
            /**
             * This is a message object. You stuff it with data, and then publish it.
             */
            sensor_msgs::Imu msg;
            msg.linear_acceleration.x = imu.getAccelX();
            msg.linear_acceleration.y = imu.getAccelY();
            msg.linear_acceleration.z = imu.getAccelZ();

            ROS_INFO_STREAM( "acc[" << std::setw(10) << imu.getTimeStamp() << "]: " << msg.linear_acceleration.x << " " << msg.linear_acceleration.y << " " << msg.linear_acceleration.z);

            /**
             * The publish() function is how you send messages. The parameter
             * is the message object. The type of this object must agree with the type
             * given as a template parameter to the advertise<>() call, as was done
             * in the constructor above.
             */
            imu_pub.publish(msg);
        }

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}