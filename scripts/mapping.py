#!/usr/bin/env python3 

import rospy
import numpy as np

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

from bruce_slam.utils.visualization import ros_colorline_trajectory


class SimpleMappingNode(object):
    def __init__(self):
        self.path_msg = Path()
        self.keyframe_step = 3
        self.counter = 0
        # Use point cloud for visualization
        self.traj_pub = rospy.Publisher(
                    "traj_dead_reck", Path, queue_size=10)
        self.cloudPublisher = rospy.Publisher("MapCloud", PointCloud2, queue_size=10)

    def node_init(self):
        rospy.init_node('mapping_node', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        rospy.Subscriber("localization/odom", Odometry, self.odom_callback)
        rospy.Subscriber("SonarCloud", PointCloud2, self.pc_callback)
        rospy.spin()
            


    def odom_callback(self, odom_msg):

        new_pose = odom_msg.pose.pose.position 
        header = odom_msg.header

        new_keyframe = False
        if (self.counter%self.keyframe_step) < 1:
            new_keyframe = True

        if new_keyframe:
            #self.keyframes.append(new_pose)
            self.path_msg.header.frame_id = "map"
            self.path_msg.header.stamp = header.stamp
            pose_stamped = PoseStamped()
            pose_stamped.pose.position = new_pose
            self.path_msg.poses.append(pose_stamped)
            self.traj_pub.publish(self.path_msg)

    def pc_callback(self.pc_msg):
        


        self.counter = self.counter + 1
  
if __name__ == '__main__':
    try:
        node = SimpleMappingNode()
        node.node_init()
    except rospy.ROSInterruptException:
        pass

