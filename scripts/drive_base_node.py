#!/usr/bin/env python
#import roslib; roslib.load_manifest('skid_robot')

import sys
import argparse
import rospy
from geometry_msgs.msg import Twist
from drivetrain import DriveTrain


# function to parse command-line arguments
def get_args(args):
    parser = argparse.ArgumentParser(description='Run the drive_base ROS node')
    parser.add_argument('-v', '--verbose', help='increase output verbosity', action='store_true')    
    args, unknown = parser.parse_known_args()
    return args


# Robot drive train: channels for each motor controller
train = DriveTrain(4, 6, 5, 10, 12, 11, 9, 7, 8, 15, 14, 13)

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)
    train.drive( data.linear.x, data.angular.z)

def drive_base_node(argv):

    # parse arguments
    args = get_args(argv[1:])

    # get parameters
    cmdTopic = rospy.resolve_name(rospy.get_param('~command', 'cmd_vel'))

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('drive_base', anonymous=False, log_level=rospy.DEBUG)

    print('configuration:')
    print('    node: {0}'.format(rospy.get_name()))
    print('    {0}: {1}'.format(rospy.resolve_name('~command'), cmdTopic))

    rospy.Subscriber(cmdTopic, Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        drive_base_node(sys.argv)
    except rospy.ROSInterruptException:
        pass
