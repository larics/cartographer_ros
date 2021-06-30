#!/usr/bin/python

import sys
import math
import rospy
import argparse
from cartographer_ros_msgs.srv import StartTrajectory, StartTrajectoryRequest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, TransformStamped
import tf
import tf_conversions

"""
Script for starting a new Cartographer trajectory with an initial pose and orientation.

Please use only one of the two subscribers

Subscriptions:
  odom
    Read the initial position and yaw from nav_msgs/Odometry

  transform
    Read the initial position from geometry_msgs/TransformStamped
    Initial yaw is set from the parameter ~fixed_yaw

Parameters:
  ~fixed_yaw:
    Initial yaw to be used for starting the trajectory.
    This will override the odometry yaw if non-zero.

"""

class SlamStarter:

    def __init__(self, configuration_directory, configuration_basename):

        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomCb)
        self.transform_sub = rospy.Subscriber('transform', TransformStamped, self.transformCb)
        self.start_trajectory_proxy = rospy.ServiceProxy('start_trajectory', StartTrajectory)

        self.odom_received = False
        self.transform_received = False

        self.configuration_basename = configuration_basename
        self.configuration_directory = configuration_directory

        self.fixed_yaw = rospy.get_param("~fixed_yaw", 0.0)

        self.rate = rospy.Rate(10)


    def transformCb(self, data):
        self.transform_received = True

        new_orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, self.fixed_yaw))

        self.start_trajectory(data.transform.translation.x,
                              data.transform.translation.y,
                              data.transform.translation.z,
                              new_orientation)



    def odomCb(self, data):
        self.odom_received = True

        if(abs(self.fixed_yaw > 0.001)):
          new_orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, self.fixed_yaw))
        else:
          q = [data.pose.pose.orientation.w,
              data.pose.pose.orientation.x,
              data.pose.pose.orientation.y,
              data.pose.pose.orientation.z]
          yaw = math.atan2( 2 * (q[0]*q[3] + q[1]*q[2]), 1 - 2 * (q[2]**2 + q[3]**2))
          new_orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, yaw))

        self.start_trajectory(data.pose.pose.position.x,
                              data.pose.pose.position.y,
                              data.pose.pose.position.z,
                              new_orientation)


    def start_trajectory(self, x, y, z, orientation):

        self.req = StartTrajectoryRequest()

        self.req.configuration_directory = self.configuration_directory
        self.req.configuration_basename = self.configuration_basename
        self.req.use_initial_pose = True
        self.req.relative_to_trajectory_id = -1
        self.req.initial_pose.orientation = orientation

        self.req.initial_pose.position.x = x
        self.req.initial_pose.position.y = y
        self.req.initial_pose.position.z = z

        try:
            rospy.sleep(1)
            self.start_trajectory_proxy.call(self.req)
            rospy.signal_shutdown('SLAM started')
        except rospy.ServiceException as e:
            rospy.logerr('Calling /start_trajectory failed :%s', e)


if __name__ == "__main__":
    rospy.init_node('SLAM_starter')
    argv = rospy.myargv(argv=sys.argv)
    slam_starter = SlamStarter(argv[1], argv[2])
    rospy.spin()
