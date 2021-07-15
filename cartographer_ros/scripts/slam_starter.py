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

  ~use_fixed_yaw:
     True if the fixed_yaw parameter is used.

  ~num_samples
    Number of measurements used to calculate the mean starting pose.
"""

class SlamStarter:

    def __init__(self, configuration_directory, configuration_basename):

        self.odom_sub = rospy.Subscriber('starter_odom', Odometry, self.odomCb)
        self.transform_sub = rospy.Subscriber('starter_transform', TransformStamped, self.transformCb)
        self.start_trajectory_proxy = rospy.ServiceProxy('start_trajectory', StartTrajectory)

        self.odom_received = False
        self.transform_received = False

        self.num_odoms = 0
        self.num_transforms = 0

        self.x_sum = 0
        self.y_sum = 0
        self.z_sum = 0
  
        self.roll_sum = 0
        self.pitch_sum = 0
        self.yaw_sum = 0

        self.configuration_basename = configuration_basename
        self.configuration_directory = configuration_directory

        self.fixed_yaw = rospy.get_param("~fixed_yaw", 0.0)
        self.use_fixed_yaw = rospy.get_param("~use_fixed_yaw", False)
        self.num_samples_param = rospy.get_param("~num_samples", 10)

        self.rate = rospy.Rate(10)


    def transformCb(self, data):
        self.transform_received = True

        self.num_transforms += 1

        self.x_sum += data.transform.translation.x
        self.y_sum += data.transform.translation.y
        self.z_sum += data.transform.translation.z
        
        if not self.use_fixed_yaw:
          q = [data.transform.rotation.w,
              data.transform.rotation.x,
              data.transform.rotation.y,
              data.transform.rotation.z]

          self.yaw_sum = math.atan2( 2 * (q[0]*q[3] + q[1]*q[2]), 1 - 2 * (q[2]**2 + q[3]**2))

        if self.num_transforms == self.num_samples_param:
          self.start_trajectory()
    #


    def odomCb(self, data):
        self.odom_received = True

        self.num_odoms += 1

        self.x_sum += data.pose.pose.position.x
        self.y_sum += data.pose.pose.position.y
        self.z_sum += data.pose.pose.position.z

        if not self.use_fixed_yaw:
          q = [data.pose.pose.orientation.w,
              data.pose.pose.orientation.x,
              data.pose.pose.orientation.y,
              data.pose.pose.orientation.z]

          self.yaw_sum += math.atan2( 2 * (q[0]*q[3] + q[1]*q[2]), 1 - 2 * (q[2]**2 + q[3]**2))
        
        if self.num_odoms == self.num_samples_param:
          self.start_trajectory()
    #

    def start_trajectory(self):
          
        if self.use_fixed_yaw:
          new_orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, self.fixed_yaw))
        else:
          new_orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, self.yaw_sum / self.num_samples_param))
          
        self.req = StartTrajectoryRequest()

        self.req.configuration_directory = self.configuration_directory
        self.req.configuration_basename = self.configuration_basename
        self.req.use_initial_pose = True
        self.req.relative_to_trajectory_id = -1
        self.req.initial_pose.orientation = new_orientation

        self.req.initial_pose.position.x = self.x_sum / self.num_samples_param
        self.req.initial_pose.position.y = self.y_sum / self.num_samples_param
        self.req.initial_pose.position.z = self.z_sum / self.num_samples_param

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
