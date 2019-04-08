#!/usr/bin/env python
#import roslib; roslib.load_manifest('skid_robot')
import rospy
from geometry_msgs.msg import Twist
from drivetrain import DriveTrain

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)
    train.drive( data.linear.x, data.angular.z)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    # Robot drive train
    train = DriveTrain(4, 6, 5, 10, 12, 11, 9, 7, 8, 15, 14, 13)

    rospy.Subscriber('/cmd_vel', Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
