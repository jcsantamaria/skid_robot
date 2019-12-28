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
#define MOTOR1_QUAD_A 4      // GPIO 4: BCM 23 : P16
#define MOTOR1_QUAD_B 5      // GPIO 5: BCM 24 : P18

// -- motor 2 
#define MOTOR2_QUAD_A 2      // GPIO 2: BCM 2 : P13
#define MOTOR2_QUAD_B 3      // GPIO 3: BCM 3 : P15

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
    struct Encoder *motor_1 = setup_encoder(MOTOR1_QUAD_A, MOTOR1_QUAD_B);

    /**
     * Motor 2 encoders: it is mounted in the opposite direction from motor 1
     *                   therefore, the encoders are reversed to provide the same
     *                   sign as motor 1
     */
    struct Encoder *motor_2 = setup_encoder(MOTOR2_QUAD_B, MOTOR2_QUAD_A);

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

        ROS_INFO_STREAM("motor_1: " << motor_1->value << "  motor_2: " << motor_2->value);
        //ROS_INFO_STREAM("motor_1: " << motor_1->value);
        //ROS_INFO_STREAM("motor_2: " << motor_2->value);

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
