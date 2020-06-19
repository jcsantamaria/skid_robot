#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>

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
#define MOTOR_RGT_QUAD_A 4      // GPIO 4: BCM 23 : P16
#define MOTOR_RGT_QUAD_B 5      // GPIO 5: BCM 24 : P18

// -- motor 2 
#define MOTOR_LFT_QUAD_A 2      // GPIO 2: BCM 2 : P13
#define MOTOR_LFT_QUAD_B 3      // GPIO 3: BCM 3 : P15

static const double TICK_PER_REV = 45 * 48;     // gear ratio * PPR (PULSE_PER_REV)

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_base");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle nh("~");

    /*
     * Node parameters
     */
    double publishRate = 50.0;
    nh.param<double>("publish_rate", publishRate, 50.0);
    bool   publish_odom = false;
    nh.param<bool>("enable_odom_tf", publish_odom, false);

    /*
     * Robot parameters
     */
    double wheelRadius = 0.123;     // 12.3 cm
    double axelWidth   = 0.230;     // 23.0 cm
    nh.param<double>("/wheel_radius", wheelRadius, wheelRadius);
    nh.param<double>("/axel_width"  , axelWidth  , axelWidth);

    std::string base_frame_id = "base_link";
    nh.param<std::string>("base_frame_id", base_frame_id, "base_link");
    // base_frame_id = nh.resolveName(base_frame_id);

    std::cout << nh.resolveName("") << ":" << std::endl;
    std::cout << "    base_frame  : " << base_frame_id << std::endl;
    std::cout << "    wheel_radius: " << wheelRadius   << std::endl;
    std::cout << "    axel_width  : " << axelWidth     << std::endl;

    /**
     * Publish imu sensor
     */
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data", 50);

    /**
     * Publish Odometry sensor
     */
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

    /**
     * Publish tf
     */
    tf2_ros::TransformBroadcaster odom_broadcaster;

    /**
     * Node parameters
     */
    ros::Rate loop_rate(publishRate);

    /**
     * BNO080 initialization
     */
    bno080_rvc_init(UART_DEVICE, RSTN_GPIO_PIN, imu_callback);

    /**
     * Motor 1 encoders
     */
    struct Encoder *motor_rgt = setup_encoder(MOTOR_RGT_QUAD_A, MOTOR_RGT_QUAD_B);

    /**
     * Motor 2 encoders: it is mounted in the opposite direction from motor 1
     *                   therefore, the encoders are reversed to provide the same
     *                   sign as motor 1
     */
    struct Encoder *motor_lft = setup_encoder(MOTOR_LFT_QUAD_B, MOTOR_LFT_QUAD_A);

    /*
     * estimated pose of the robot according to wheel odometry
     */
    double x = 0.0, y = 0.0, yaw = 0.0;

    /*
     * State keeping
     */
    long ticks_lft_last = 0;
    long ticks_rgt_last = 0;

    /*
     * Time keeping
     */
    ros::Time     current_time = ros::Time::now();
    ros::Time     last_time    = current_time;
    ros::Duration elapsed;

    while (ros::ok())
    {
        ros::spinOnce();    // check for incoming messages

        // track time
        current_time = ros::Time::now();
        elapsed      = current_time - last_time;

        // fetch and publish imu message
        {
            sensor_msgs::Imu imu;
            imu.header.stamp    = current_time;
            imu.header.frame_id = "imu_link";

            uint32_t timestamp = 0;

            // fetch current imu status (exclusive)
            piLock(0);
            timestamp = report.timestamp;

            // set linear acceleration
            imu.linear_acceleration.x = report.acc_x;
            imu.linear_acceleration.y = report.acc_y;
            imu.linear_acceleration.z = report.acc_z;

            // set orientation
            tf2::Quaternion q_tf;
            q_tf.setRPY(report.roll * DEG2RAD, report.pitch * DEG2RAD, report.yaw * DEG2RAD);
            imu.orientation = tf2::toMsg(q_tf);
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
                imu_pub.publish(imu);
            }
        }

        // fetch wheel encoders and compute odometry
        {
            // compute speed
            double delta_t = elapsed.toSec();
            double raw_speed_rgt = (motor_rgt->value - ticks_rgt_last) / TICK_PER_REV * 2.0 * M_PI * wheelRadius / delta_t;
            double raw_speed_lft = (motor_lft->value - ticks_lft_last) / TICK_PER_REV * 2.0 * M_PI * wheelRadius / delta_t;
            ticks_rgt_last = motor_rgt->value;
            ticks_lft_last = motor_lft->value;

            double fwd_speed = (raw_speed_lft + raw_speed_rgt) / 2.0;
            double yaw_speed = (raw_speed_rgt - raw_speed_lft) / axelWidth;

            // estimate position
            x   += fwd_speed * cos(yaw) * delta_t;
            y   += fwd_speed * sin(yaw) * delta_t;
            yaw += yaw_speed * delta_t;

            // form Odometry message
            nav_msgs::Odometry odom;
            odom.header.stamp    = current_time;
            odom.header.frame_id = "odom";
            odom.child_frame_id  = base_frame_id.c_str();

            // set position
            geometry_msgs::Quaternion base_q = tf::createQuaternionMsgFromYaw(yaw);

            odom.pose.pose.position.x  = x;
            odom.pose.pose.position.y  = y;
            odom.pose.pose.position.z  = 0.0;
            odom.pose.pose.orientation = base_q;

            // set velocity
            odom.twist.twist.linear.x = fwd_speed;
            odom.twist.twist.linear.y = 0.0;
            odom.twist.twist.linear.z = 0.0;
            odom.twist.twist.angular.z = yaw_speed;

            odom_pub.publish(odom);

            //ROS_INFO_STREAM("motor_rgt: " << motor_rgt->value << "  motor_lft: " << motor_lft->value);

            if (publish_odom)
            {
                geometry_msgs::TransformStamped odom_trans;
                odom_trans.header.stamp = current_time;
                odom_trans.header.frame_id = "odom";
                odom_trans.child_frame_id = base_frame_id.c_str();
                
                odom_trans.transform.translation.x = x;
                odom_trans.transform.translation.y = y;
                odom_trans.transform.translation.z = 0.0;
                odom_trans.transform.rotation = base_q;

                //send the transform
                odom_broadcaster.sendTransform(odom_trans);
            }
        }

        last_time = current_time;

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
