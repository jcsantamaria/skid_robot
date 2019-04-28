#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

#include <sstream>

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

    while (ros::ok())
    {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        sensor_msgs::Imu msg;

        ROS_INFO("%s", msg.data.c_str());

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        cimu_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}