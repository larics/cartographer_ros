#!/usr/bin/python

import sys
import math
import rospy
import argparse
from cartographer_ros_msgs.srv import StartTrajectory, StartTrajectoryRequest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import tf
import tf_conversions

"""
"""

class SlamStarter:

    def __init__(self, configuration_directory, configuration_basename):
        rospy.Subscriber('odom', Odometry, self.odomCb)
        self.start_trajectory = rospy.ServiceProxy('start_trajectory', StartTrajectory)
        self.odom_received = False

        self.configuration_basename = configuration_basename
        self.configuration_directory = configuration_directory

    def odomCb(self, data):
        self.odom_received = True

        self.odom_pose = data.pose.pose

        # Extract yaw
        q = [data.pose.pose.orientation.w,
                data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z]

        yaw = math.atan2( 2 * (q[0]*q[3] + q[1]*q[2]), 1 - 2 * (q[2]**2 + q[3]**2))
        # new_orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, yaw))

        new_orientation = data.pose.pose.orientation

        req = StartTrajectoryRequest()
        req.configuration_directory = self.configuration_directory
        req.configuration_basename = self.configuration_basename
        req.use_initial_pose = True

        req.initial_pose.position.x = 0
        req.initial_pose.position.y = 0
        req.initial_pose.position.z = 0
        req.initial_pose.orientation = new_orientation

        # req.initial_pose = self.odom_pose

        req.relative_to_trajectory_id = -1

        try:
            rospy.sleep(1)
            self.start_trajectory.call(req)
            rospy.logwarn("Starting SLAM with yaw = %f", yaw)
            rospy.signal_shutdown('SLAM started')
        except rospy.ServiceException, e:
            rospy.logerr('Calling /start_trajectory failed :%s', e)



if __name__ == "__main__":
    rospy.init_node('SLAM_starter')
    argv = rospy.myargv(argv=sys.argv)
    slam_starter = SlamStarter(argv[1], argv[2])
    rospy.spin()
