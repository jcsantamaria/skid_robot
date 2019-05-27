#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

#include <sstream>
#include <iomanip>
#include <wiringPi.h>

#include "bno080_rvc.h"

// --- Forward declarations -------------------------------------------

static void callback(RvcReport_t *pReport);

// --- Private data ---------------------------------------------------

static RvcReport_t report;

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

    sensor_msgs::Imu msg;

    /**
     * BNO080 initialization
     */
    bno080_rvc_init(UART_DEVICE, RSTN_GPIO_PIN, callback);

    while (ros::ok())
    {
        uint32_t timestamp = 0;

        // fetch current status (exclusive)
        piLock(0);
        timestamp = report.timestamp;
        msg.linear_acceleration.x = report.acc_x;
        msg.linear_acceleration.y = report.acc_y;
        msg.linear_acceleration.z = report.acc_z;
        piUnlock(0);

        if (timestamp > 0)
        {
            ROS_INFO_STREAM("acc[" << std::setw(10) << timestamp << "]: " << msg.linear_acceleration.x << " " << msg.linear_acceleration.y << " " << msg.linear_acceleration.z);

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

// ----------------------------------------------------------------------------------
// Private functions
// ----------------------------------------------------------------------------------

void callback(RvcReport_t *pReport)
{
    piLock(0);
    memcpy(&report, pReport, sizeof(RvcReport_t));
    piUnlock(0);
}