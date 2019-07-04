#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sstream>
#include <iomanip>
#include <wiringPi.h>

#include "bno080_rvc.h"
#include "rotary_encoder.h"

// --- Forward declarations -------------------------------------------

static void imu_callback(RvcReport_t *pReport);

// --- Private data ---------------------------------------------------

static RvcReport_t report;

// --- Constants ------------------------------------------------------

static const double DEG2RAD = M_PI / 180.0;
static const double RAD2DEG = 180.0 / M_PI;

// -- motor 1 
#define GPIO_QUAD_A 4      
#define GPIO_QUAD_B 5

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
    msg.header.frame_id = "imu_link";

    /**
     * BNO080 initialization
     */
    bno080_rvc_init(UART_DEVICE, RSTN_GPIO_PIN, imu_callback);

    /**
     * Motor 1 encoders
     */
    struct Encoder *motor_1 = setup_encoder(GPIO_QUAD_A, GPIO_QUAD_B);

    while (ros::ok())
    {
        uint32_t timestamp = 0;

        // fetch current status (exclusive)
        piLock(0);
        timestamp = report.timestamp;

        // set linear acceleration
        msg.linear_acceleration.x = report.acc_x;
        msg.linear_acceleration.y = report.acc_y;
        msg.linear_acceleration.z = report.acc_z;

        // set orientation
        tf2::Quaternion q_tf;
        q_tf.setRPY(report.roll * DEG2RAD, report.pitch * DEG2RAD, report.yaw * DEG2RAD);
        msg.orientation = tf2::toMsg(q_tf);
        piUnlock(0);

        if (timestamp > 0)
        {
            //ROS_INFO_STREAM("acc[" << std::setw(10) << timestamp << "]: " << msg.linear_acceleration.x << " " << msg.linear_acceleration.y << " " << msg.linear_acceleration.z);

            /**
             * The publish() function is how you send messages. The parameter
             * is the message object. The type of this object must agree with the type
             * given as a template parameter to the advertise<>() call, as was done
             * in the constructor above.
             */
            imu_pub.publish(msg);
        }

        ROS_INFO_STREAM("motor_1: " << motor_1->value);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}

// ----------------------------------------------------------------------------------
// Private functions
// ----------------------------------------------------------------------------------

void imu_callback(RvcReport_t *pReport)
{
    piLock(0);
    memcpy(&report, pReport, sizeof(RvcReport_t));
    piUnlock(0);
}
