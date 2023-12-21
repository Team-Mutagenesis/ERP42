#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import matplotlib.pyplot as plt

from math import cos,sin,pi,sqrt,pow
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path
import time


class read_path_pub :

    def __init__(self):
        rospy.init_node('read_path_pub', anonymous=True)
        self.global_path_pub = rospy.Publisher('/global_path',Path, queue_size=1)
        self.rviz_global_path_pub = rospy.Publisher('/rviz_global_path',Path, queue_size=1)

        self.global_path_msg=Path()
        self.rviz_global_path=Path()

        self.global_path_msg.header.frame_id='/map'
        self.rviz_global_path.header.frame_id='/map'

        rospack=rospkg.RosPack()
        full_path= '/home/ha/real/src/ts/src/tryout_path.txt'


        self.f=open(full_path,'r')
        lines=self.f.readlines()
       
        # MAP 시작 점 
        # standard = lines[1].rstrip("\n").split(',')[1:]
        standard = lines[1].split()
        print(standard)

        # 실제 맵
        for i in range(1, len(lines)):
            # tmp = lines[i].rstrip("\n").split(',')
            tmp = lines[i].split()
            read_pose=PoseStamped()
            read_pose.pose.position.x=float(tmp[0])
            read_pose.pose.position.y=float(tmp[1]) 
            self.global_path_msg.poses.append(read_pose)
        

        # 시각화 맵
        for i in range(1, len(lines)):
            #tmp = lines[i].rstrip("\n").split(',')
            tmp = lines[i].split()
            read_pose=PoseStamped()
            read_pose.pose.position.x=float(tmp[0]) - float(standard[0])
            read_pose.pose.position.y=float(tmp[1]) - float(standard[1])
            self.rviz_global_path.poses.append(read_pose)
        
        self.f.close()

        rate = rospy.Rate(30) # 20hz
        while not rospy.is_shutdown(): 
            start = time.time()
            self.global_path_pub.publish(self.global_path_msg)
            self.rviz_global_path_pub.publish(self.rviz_global_path)

            print("GLOBAL TIME : %.8f" % (time.time() - start))

 
            rate.sleep()

        
if __name__ == '__main__':
    try:
        test_track=read_path_pub()

    except rospy.ROSInterruptException:
        pass
