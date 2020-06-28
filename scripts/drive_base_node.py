#!/usr/bin/env python
from math import pi, cos, sin

import diagnostic_msgs
import diagnostic_updater
import roboclaw_driver.roboclaw_driver as roboclaw
import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry

__author__ = "bwbazemore@uga.edu (Brad Bazemore)"


# TODO need to find some better was of handling OSerror 11 or preventing it, any ideas?

class EncoderOdom:
    def __init__(self, ticks_per_meter, base_width, base_length):
        self.TICKS_PER_METER = ticks_per_meter
        self.BASE_WIDTH = base_width
        self.BASE_LENGTH = base_length
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.cur_x = 0
        self.cur_y = 0
        self.cur_theta = 0.0
        self.last_enc_rear_left = 0
        self.last_enc_rear_right = 0
        self.last_enc_front_left = 0
        self.last_enc_front_right = 0
        self.last_enc_time = rospy.Time.now()

    @staticmethod
    def normalize_angle(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def update(self, enc_rear_left, enc_rear_right, enc_front_left, enc_front_right):
        rear_left_ticks   = enc_rear_left   - self.last_enc_rear_left
        rear_right_ticks  = enc_rear_right  - self.last_enc_rear_right
        front_left_ticks  = enc_front_left  - self.last_enc_front_left
        front_right_ticks = enc_front_right - self.last_enc_front_right
        self.last_enc_rear_left   = enc_rear_left
        self.last_enc_rear_right  = enc_rear_right
        self.last_enc_front_left  = enc_front_left
        self.last_enc_front_right = enc_front_right

        # compute wheel displacements
        rear_dist_left   = rear_left_ticks   / self.TICKS_PER_METER
        rear_dist_right  = rear_right_ticks  / self.TICKS_PER_METER
        front_dist_left  = front_left_ticks  / self.TICKS_PER_METER
        front_dist_right = front_right_ticks / self.TICKS_PER_METER

        # total displacement
        dist = (rear_dist_right + rear_dist_left + front_dist_right + front_dist_left) / 4.0

        # elapsed time
        current_time = rospy.Time.now()
        d_time = (current_time - self.last_enc_time).to_sec()
        self.last_enc_time = current_time

        # yaw change
        d_theta = (rear_dist_right + front_dist_right - rear_dist_left - front_dist_left) / 4.0 / (self.BASE_WIDTH + self.BASE_LENGTH)

        # update pose
        self.cur_x += dist * cos(self.cur_theta)
        self.cur_y += dist * sin(self.cur_theta)
        self.cur_theta = self.normalize_angle(self.cur_theta + d_theta)

        # compute speeds
        if abs(d_time) < 0.000001:
            vel_x = 0.0
            vel_theta = 0.0
        else:
            vel_x = dist / d_time
            vel_theta = d_theta / d_time

        return vel_x, vel_theta

    def update_publish(self, enc_rear_left, enc_rear_right, enc_front_left, enc_front_right):
        # 2106 per 0.1 seconds is max speed, error in the 16th bit is 32768
        # TODO lets find a better way to deal with this error
        if abs(enc_rear_left - self.last_enc_rear_left) > 20000:
            rospy.logerr("Ignoring rear left encoder jump: cur %d, last %d" % (enc_rear_left, self.last_enc_rear_left))
        elif abs(enc_rear_right - self.last_enc_rear_right) > 20000:
            rospy.logerr("Ignoring rear right encoder jump: cur %d, last %d" % (enc_rear_right, self.last_enc_rear_right))
        elif abs(enc_front_left - self.last_enc_front_left) > 20000:
            rospy.logerr("Ignoring front left encoder jump: cur %d, last %d" % (enc_front_left, self.last_enc_front_left))
        elif abs(enc_front_right - self.last_enc_front_right) > 20000:
            rospy.logerr("Ignoring front right encoder jump: cur %d, last %d" % (enc_front_right, self.last_enc_front_right))
        else:
            vel_x, vel_theta = self.update(enc_rear_left, enc_rear_right, enc_front_left, enc_front_right)
            self.publish_odom(self.cur_x, self.cur_y, self.cur_theta, vel_x, vel_theta)

    def publish_odom(self, cur_x, cur_y, cur_theta, vx, vth):
        quat = tf.transformations.quaternion_from_euler(0, 0, cur_theta)
        current_time = rospy.Time.now()

        br = tf.TransformBroadcaster()
        br.sendTransform((cur_x, cur_y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, cur_theta),
                         current_time,
                         "base_link",
                         "odom")

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'

        odom.pose.pose.position.x = cur_x
        odom.pose.pose.position.y = cur_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*quat)

        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        # odom.pose.covariance[14] = 99999
        # odom.pose.covariance[21] = 99999
        # odom.pose.covariance[28] = 99999
        odom.pose.covariance[35] = 0.01

        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = odom.pose.covariance

        self.odom_pub.publish(odom)


class Node:
    def __init__(self):

        self.ERRORS = {0x0000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
                       0x0001: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 over current"),
                       0x0002: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 over current"),
                       0x0004: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Emergency Stop"),
                       0x0008: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
                       0x0010: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
                       0x0020: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Main batt voltage high"),
                       0x0040: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage high"),
                       0x0080: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage low"),
                       0x0100: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 driver fault"),
                       0x0200: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 driver fault"),
                       0x0400: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage high"),
                       0x0800: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage low"),
                       0x1000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
                       0x2000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
                       0x4000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M1 home"),
                       0x8000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M2 home")}

        rospy.init_node("drive_base_node", anonymous = False)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Connecting to roboclaws")
        dev_name = rospy.get_param("~dev", "/dev/ttyAMA0")
        baud_rate = int(rospy.get_param("~baud_rate", "115200"))

        # rear roboclaw
        self.rear_address = int(rospy.get_param("~rear_address", "129"))
        if self.rear_address > 0x87 or self.rear_address < 0x80:
            rospy.logfatal("Rear address out of range")
            rospy.signal_shutdown("Rear address out of range")

        # front roboclaw
        self.front_address = int(rospy.get_param("~front_address", "128"))
        if self.front_address > 0x87 or self.front_address < 0x80:
            rospy.logfatal("Front address out of range")
            rospy.signal_shutdown("Front address out of range")

        # TODO need someway to check if address is correct
        try:
            roboclaw.Open(dev_name, baud_rate)
        except Exception as e:
            rospy.logfatal("Could not connect to Roboclaw")
            rospy.logdebug(e)
            rospy.signal_shutdown("Could not connect to Roboclaw")

        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("Roboclaw")
        self.updater.add(diagnostic_updater.
                         FunctionDiagnosticTask("Rear Vitals", self.check_rear_vitals))
        self.updater.add(diagnostic_updater.
                         FunctionDiagnosticTask("Front Vitals", self.check_front_vitals))

        # collect and report rear roboclaw version
        try:
            version = roboclaw.ReadVersion(self.rear_address)
        except Exception as e:
            rospy.logwarn("Problem getting rear roboclaw version")
            rospy.logdebug(e)
            pass

        if not version[0]:
            rospy.logwarn("Could not get version from rear roboclaw")
        else:
            rospy.loginfo("Rear roboclaw version: %s", repr(version[1]))

        # collect and report front roboclaw version
        try:
            version = roboclaw.ReadVersion(self.front_address)
        except Exception as e:
            rospy.logwarn("Problem getting front roboclaw version")
            rospy.logdebug(e)
            pass

        if not version[0]:
            rospy.logwarn("Could not get version from front roboclaw")
        else:
            rospy.loginfo("Front roboclaw version: %s", repr(version[1]))

        # stop all motors
        roboclaw.SpeedM1M2(self.rear_address, 0, 0)
        roboclaw.ResetEncoders(self.rear_address)
        roboclaw.SpeedM1M2(self.front_address, 0, 0)
        roboclaw.ResetEncoders(self.front_address)

        # collect parameters
        self.MAX_SPEED       = float(rospy.get_param("~max_speed"     ,    "2.0"))
        self.TICKS_PER_METER = float(rospy.get_param("~tick_per_meter", "1634.2"))
        self.BASE_WIDTH      = float(rospy.get_param("~axel_width"    ,    "0.233"))
        self.BASE_LENGTH     = float(rospy.get_param("~axel_length"   ,    "0.142"))

        # instantiate encoder
        self.encodm = EncoderOdom(self.TICKS_PER_METER, self.BASE_WIDTH, self.BASE_LENGTH)
        self.last_set_speed_time = rospy.get_rostime()

        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

        rospy.sleep(1)

        # report configuration
        rospy.loginfo("device          : %s", dev_name)
        rospy.loginfo("baud_rate       : %d", baud_rate)
        rospy.loginfo("rear_address    : %d", self.rear_address)
        rospy.loginfo("front_address   : %d", self.front_address)
        rospy.loginfo("max_speed       : %f", self.MAX_SPEED)
        rospy.loginfo("ticks_per_meter : %f", self.TICKS_PER_METER)
        rospy.loginfo("base_width      : %f", self.BASE_WIDTH)
        rospy.loginfo("base_length     : %f", self.BASE_LENGTH)

    def run(self):
        rospy.loginfo("Starting motor drive")
        r_time = rospy.Rate(10)
        while not rospy.is_shutdown():

            if (rospy.get_rostime() - self.last_set_speed_time).to_sec() > 1:
                rospy.loginfo("Did not get command for 1 second, stopping")
                try:
                    roboclaw.ForwardM1(self.rear_address, 0)
                    roboclaw.ForwardM2(self.rear_address, 0)
                    roboclaw.ForwardM1(self.front_address, 0)
                    roboclaw.ForwardM2(self.front_address, 0)
                    self.last_set_speed_time = rospy.get_rostime()
                except OSError as e:
                    rospy.logerr("Could not stop")
                    rospy.logdebug(e)

            # TODO need find solution to the OSError11 looks like sync problem with serial
            rear_status1, rear_enc1, rear_crc1 = None, None, None
            rear_status2, rear_enc2, rear_crc2 = None, None, None
            front_status1, front_enc1, front_crc1 = None, None, None
            front_status2, front_enc2, front_crc2 = None, None, None

            # rear roboclaw
            try:
                rear_status1, rear_enc1, rear_crc1 = roboclaw.ReadEncM1(self.rear_address)
            except ValueError:
                pass
            except OSError as e:
                rospy.logwarn("Rear ReadEncM1 OSError: %d", e.errno)
                rospy.logdebug(e)

            try:
                rear_status2, rear_enc2, rear_crc2 = roboclaw.ReadEncM2(self.rear_address)
            except ValueError:
                pass
            except OSError as e:
                rospy.logwarn("Rear ReadEncM2 OSError: %d", e.errno)
                rospy.logdebug(e)

            # front roboclaw
            try:
                front_status1, front_enc1, front_crc1 = roboclaw.ReadEncM1(self.front_address)
            except ValueError:
                pass
            except OSError as e:
                rospy.logwarn("Front ReadEncM1 OSError: %d", e.errno)
                rospy.logdebug(e)

            try:
                front_status2, front_enc2, front_crc2 = roboclaw.ReadEncM2(self.front_address)
            except ValueError:
                pass
            except OSError as e:
                rospy.logwarn("Front ReadEncM2 OSError: %d", e.errno)
                rospy.logdebug(e)

            if (rear_enc1 is not None and rear_enc2 is not None and front_enc1 is not None and front_enc2 is not None):
                rospy.logdebug(" Encoders rear: %s %s  front: %s %s", rear_enc1, rear_enc2, front_enc1, front_enc2)
                self.encodm.update_publish(rear_enc1, rear_enc2, front_enc1, front_enc2)

                self.updater.update()
            r_time.sleep()

    def cmd_vel_callback(self, twist):

        if (rospy.get_rostime() - self.last_set_speed_time).to_sec() > 0.1:
            self.last_set_speed_time = rospy.get_rostime()

            linear_x = twist.linear.x
            if linear_x > self.MAX_SPEED:
                linear_x = self.MAX_SPEED
            if linear_x < -self.MAX_SPEED:
                linear_x = -self.MAX_SPEED

            vr = linear_x + twist.angular.z * (self.BASE_WIDTH + self.BASE_LENGTH)  # m/s
            vl = linear_x - twist.angular.z * (self.BASE_WIDTH + self.BASE_LENGTH)

            vr_ticks = int(vr * self.TICKS_PER_METER)  # ticks/s
            vl_ticks = int(vl * self.TICKS_PER_METER)

            # rospy.logdebug("vr_ticks:%d vl_ticks: %d", vr_ticks, vl_ticks)

            try:
                # This is a hack way to keep a poorly tuned PID from making noise at speed 0
                if vr_ticks is 0 and vl_ticks is 0:
                    roboclaw.ForwardM1(self.rear_address, 0)    # left
                    roboclaw.ForwardM2(self.rear_address, 0)    # right
                    roboclaw.ForwardM1(self.front_address, 0)   # left
                    roboclaw.ForwardM2(self.front_address, 0)   # right
                else:
                    roboclaw.SpeedM1M2(self.rear_address , vl_ticks, vr_ticks)
                    roboclaw.SpeedM1M2(self.front_address, vl_ticks, vr_ticks)
            except OSError as e:
                rospy.logwarn("Rear SpeedM1M2 OSError: %d", e.errno)
                rospy.logdebug(e)

    # TODO: Need to make this work when more than one error is raised
    def check_rear_vitals(self, stat):
        try:
            status = roboclaw.ReadError(self.rear_address)[1]
        except OSError as e:
            rospy.logwarn("Rear diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
            return
        state, message = self.ERRORS[status]
        stat.summary(state, message)
        try:
            stat.add("Rear Main Batt V:", float(roboclaw.ReadMainBatteryVoltage(self.rear_address)[1] / 10))
            stat.add("Rear Logic Batt V:", float(roboclaw.ReadLogicBatteryVoltage(self.rear_address)[1] / 10))
            stat.add("Rear Temp1 C:", float(roboclaw.ReadTemp(self.rear_address)[1] / 10))
            stat.add("Rear Temp2 C:", float(roboclaw.ReadTemp2(self.rear_address)[1] / 10))
        except OSError as e:
            rospy.logwarn("Rear diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
        return stat

    # TODO: Need to make this work when more than one error is raised
    def check_front_vitals(self, stat):
        try:
            status = roboclaw.ReadError(self.front_address)[1]
        except OSError as e:
            rospy.logwarn("Front diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
            return
        state, message = self.ERRORS[status]
        stat.summary(state, message)
        try:
            stat.add("Front Main Batt V:", float(roboclaw.ReadMainBatteryVoltage(self.front_address)[1] / 10))
            stat.add("Front Logic Batt V:", float(roboclaw.ReadLogicBatteryVoltage(self.front_address)[1] / 10))
            stat.add("Front Temp1 C:", float(roboclaw.ReadTemp(self.front_address)[1] / 10))
            stat.add("Front Temp2 C:", float(roboclaw.ReadTemp2(self.front_address)[1] / 10))
        except OSError as e:
            rospy.logwarn("Front diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
        return stat

    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self):
        rospy.loginfo("Shutting down")
        try:
            roboclaw.ForwardM1(self.rear_address, 0)
            roboclaw.ForwardM2(self.rear_address, 0)
            roboclaw.ForwardM1(self.front_address, 0)
            roboclaw.ForwardM2(self.front_address, 0)
        except OSError:
            rospy.logerr("Shutdown did not work trying again")
            try:
                roboclaw.ForwardM1(self.rear_address, 0)
                roboclaw.ForwardM2(self.rear_address, 0)
                roboclaw.ForwardM1(self.front_address, 0)
                roboclaw.ForwardM2(self.front_address, 0)
            except OSError as e:
                rospy.logerr("Could not shutdown motors!!!!")
                rospy.logdebug(e)


if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")
