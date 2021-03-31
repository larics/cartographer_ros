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
"""

class SlamStarter:

    def __init__(self, configuration_directory, configuration_basename):
        # rospy.Subscriber('odom', Odometry, self.odomCb)
        rospy.Subscriber('pozyx/measured', TransformStamped, self.pozyxCb)
        self.start_trajectory = rospy.ServiceProxy('start_trajectory', StartTrajectory)
        self.odom_received = False
        self.pozyx_received = False

        self.configuration_basename = configuration_basename
        self.configuration_directory = configuration_directory

    def pozyxCb(self, data):
        self.odom_received = True

        self.pozyx_position = data.transform.translation

        # Extract yaw
        q = [data.transform.rotation.w,
                data.transform.rotation.x,
                data.transform.rotation.y,
                data.transform.rotation.z]

        # yaw = math.atan2( 2 * (q[0]*q[3] + q[1]*q[2]), 1 - 2 * (q[2]**2 + q[3]**2))
        yaw = 1*math.pi/2 #   -0.8*math.pi/2 # 
        roll= 0 # math.pi
        new_orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(roll, 0, yaw))

        req = StartTrajectoryRequest()
        req.configuration_directory = self.configuration_directory
        req.configuration_basename = self.configuration_basename
        req.use_initial_pose = True

        req.initial_pose.position.x =  self.pozyx_position.x
        req.initial_pose.position.y =  self.pozyx_position.y
        req.initial_pose.position.z =  self.pozyx_position.z
        req.initial_pose.orientation = new_orientation

        # req.initial_pose = self.odom_pose

        req.relative_to_trajectory_id = -1

        try:
            rospy.sleep(1)
            self.start_trajectory.call(req)
            rospy.logwarn("Starting SLAM")
            rospy.signal_shutdown('SLAM started')
        except rospy.ServiceException, e:
            rospy.logerr('Calling /start_trajectory failed :%s', e)



if __name__ == "__main__":
    rospy.init_node('SLAM_starter')
    argv = rospy.myargv(argv=sys.argv)
    slam_starter = SlamStarter(argv[1], argv[2])
    rospy.spin()
