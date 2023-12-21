#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos, sin, pi, sqrt, pow
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Odometry, Path
import tf
import time

class path_pub:

    def __init__(self):
        rospy.init_node('path_pub', anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/global_path", Path, self.global_Path_callback)
        rospy.Subscriber("/rviz_global_path", Path, self.rviz_global_Path_callback)

        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.rviz_local_path_pub = rospy.Publisher('/rviz_local_path', Path, queue_size=1)

        # 초기화
        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'

        self.rviz_global_path_msg = Path()
        self.rviz_global_path_msg.header.frame_id = '/map'

        self.is_status = False
        self.local_path_size = 20

        self.x = 0
        self.y = 0

        rate = rospy.Rate(30)  # 20hz
        while not rospy.is_shutdown():
            if self.is_status == True:
                start = time.time()
                local_path_msg = Path()
                rviz_local_path_msg = Path()
                local_path_msg.header.frame_id = '/map'
                rviz_local_path_msg.header.frame_id = '/map'

                x = self.x
                y = self.y
                min_dis = float('inf')
                current_waypoint = -1

                for i, waypoint in enumerate(self.global_path_msg.poses):
                    distance = sqrt(pow(x - waypoint.pose.position.x, 2) + pow(y - waypoint.pose.position.y, 2))
                    if distance < min_dis:
                        min_dis = distance
                        current_waypoint = i

                if current_waypoint != -1:
                    if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                        for num in range(current_waypoint, current_waypoint + self.local_path_size):
                            tmp_pose = PoseStamped()
                            tmp_pose_rviz = PoseStamped()

                            # 실제 local path
                            tmp_pose.pose.position.x = self.global_path_msg.poses[num].pose.position.x
                            tmp_pose.pose.position.y = self.global_path_msg.poses[num].pose.position.y
                            tmp_pose.pose.orientation.w = 1

                            local_path_msg.poses.append(tmp_pose)

                            # rviz상 local path
                            tmp_pose_rviz.pose.position.x=self.rviz_global_path_msg.poses[num].pose.position.x
                            tmp_pose_rviz.pose.position.y=self.rviz_global_path_msg.poses[num].pose.position.y 
                            tmp_pose_rviz.pose.orientation.w=1



                            rviz_local_path_msg.poses.append(tmp_pose_rviz)

                    else:
                        for num in range(current_waypoint, len(self.global_path_msg.poses)):
                            tmp_pose = PoseStamped()
                            tmp_pose_rviz = PoseStamped()

                            # 실제 local path
                            tmp_pose.pose.position.x = self.global_path_msg.poses[num].pose.position.x
                            tmp_pose.pose.position.y = self.global_path_msg.poses[num].pose.position.y
                            tmp_pose.pose.orientation.w = 1

                            local_path_msg.poses.append(tmp_pose)

                            # rviz상 local path
                            tmp_pose_rviz.pose.position.x = self.rviz_global_path_msg.poses[num].pose.position.x
                            tmp_pose_rviz.pose.position.y = self.rviz_global_path_msg.poses[num].pose.position.y
                            tmp_pose_rviz.pose.orientation.w = 1
                            rviz_local_path_msg.poses.append(tmp_pose_rviz)


                self.local_path_pub.publish(local_path_msg)
                self.rviz_local_path_pub.publish(rviz_local_path_msg)

                print("LOCAL TIME : %.8f" % (time.time() - start))


            rate.sleep()

    def odom_callback(self, msg):
        self.is_status = True

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def global_Path_callback(self, msg):
        self.global_path_msg = msg

    def rviz_global_Path_callback(self, msg):
        self.rviz_global_path_msg = msg


if __name__ == '__main__':
    try:
        test_track = path_pub()

    except rospy.ROSInterruptException:
        pass

