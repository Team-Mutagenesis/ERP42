#!/usr/bin/env python
# -*- coding: utf-8 -*-

from os import path
import rospy
from math import cos, sin, pi, sqrt, pow, atan2
import numpy as np
from tf.transformations import euler_from_quaternion
import math

from sensor_msgs.msg import NavSatFix
from detection_msgs.msg import BoundingBox, BoundingBoxes, Label
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from erp_driver.msg import erpCmdMsg, erpStatusMsg
from std_msgs.msg import Int16


class tryout_main:
    def __init__(self):
        rospy.init_node('tryout_main', anonymous=True)

        print("======ERP42 START======")

        rospy.Subscriber("/local_path", Path, self.path_callback) # Path msg type
        rospy.Subscriber("/erp42_status", erpStatusMsg, self.status_callback)
        rospy.Subscriber("/pursuit_to_main", erpCmdMsg, self.ptm_callback)
        rospy.Subscriber("/rubber_to_main", erpCmdMsg, self.rtm_callback)
        rospy.Subscriber("/lane_to_main", erpCmdMsg, self.ltm_callback)
        rospy.Subscriber("/Bounding_box", BoundingBox, self.ytm_callback)
        
        self.ctrl_cmd_pub = rospy.Publisher('erp42_ctrl_cmd', erpCmdMsg, queue_size=1)

        self.ctrl_cmd_msg = erpCmdMsg()
        self.change_point = Point()

        self.is_status = False
        self.is_gps = True
        self.is_lidar = False
        self.is_rubber = False
        self.is_lane = False
        
        self.ltm_flag = -1
        self.ytm_flag = "NOT"

        self.pure_steer = 0
        self.pure_speed = 0

        self.rubber_steer = 0
        self.rubber_speed = 0

        self.lane_steer = 0
        self.lane_speed = 0

        # self.change_point.x = 315368.4480704938	 
        # self.change_point.y = 4071529.088993086
        
        # kcity
        self.change_point.x = 302646.4049480326	
        self.change_point.y = 4123679.3384170025

        self.tryout_flag = {"construction" : "SITE SIGN", "person" : "DYNAMIC",
                            "dump" : "STATIC", "cone" : "RUBBER CONE", "NOT" : "NOTHING"}

        rate = rospy.Rate(30)  # 30hz        
        while not rospy.is_shutdown():
            # GPS 값이 들어오는 경우
            #print(self.is_status, self.is_gps)
            if self.is_status and self.is_gps: 
                print("GPS MODE", "Rubber : ", self.is_rubber)
                # 라바콘 주행
                if self.is_rubber:
                    self.ctrl_cmd_msg.steer = self.rubber_steer
                    self.ctrl_cmd_msg.speed = self.rubber_speed
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                    self.is_rubber = False # 모드 초기화 
   
                # Pure Pursuit 주행
                else:
                    self.ctrl_cmd_msg.steer = self.pure_steer
                    self.ctrl_cmd_msg.speed = self.pure_speed
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            # GPS 음영 구간
            elif self.is_status and not self.is_gps:
                print("Person : ", self.ytm_flag, "LANE : ", self.is_lane)
                # 동적 장애물 감지 시 정지
                if self.tryout_flag[self.ytm_flag] == "DYNAMIC" and self.square > 30000:
                    self.ctrl_cmd_msg.steer = 0
                    self.ctrl_cmd_msg.speed = 0
                    self.ctrl_cmd_msg.brake = 200
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                    
                    #start_time = rospy.Time.now()
                    # # 차량이 움찔거리는 걸 방지하기 위해 Delay 설정(?)
                    # while (rospy.Time.now() - start_time) < rospy.Duration.from_sec(2):
                    #     self.ctrl_cmd_msg.steer = 0
                    #     self.ctrl_cmd_msg.speed = 0
                    #     self.ctrl_cmd_msg.brake = 200
                    #     self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

                # 차선 주행
                elif self.is_lane:
                    print(self.lane_speed)
                    self.ctrl_cmd_msg.steer = self.lane_steer
                    self.ctrl_cmd_msg.speed = self.lane_speed
                    self.ctrl_cmd_msg.brake = 1
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                    self.is_lane = False # 모드 초기화
                    
            rate.sleep()


    # GPS UTM 좌표 값   
    def path_callback(self, msg):
        #print(msg.poses[0].pose.position.x, msg.poses[0].pose.position.y)
        if msg.poses[0].pose.position.x == self.change_point.x and msg.poses[0].pose.position.y == self.change_point.y:
            self.is_gps = False


    # ERP42 현재 상태
    def status_callback(self, msg):
        self.is_status = True


    # 기본주행 모드 : Pure Pursuit
    def ptm_callback(self, msg):
        self.pure_steer = msg.steer
        self.pure_speed = msg.speed
        print("222",self.pure_speed, self.pure_steer)
        

    # 라바콘 주행
    def rtm_callback(self, msg):
        self.is_rubber = True
        self.rubber_steer = msg.steer
        self.rubber_speed = msg.speed


    # 차선 주행
    def ltm_callback(self, msg):
        self.is_lane = True
        self.lane_steer = msg.steer
        self.lane_speed = msg.speed
        #print("!!!",self.lane_steer)


    # YOLO 객체 인식
    def ytm_callback(self, msg):
        self.ytm_flag = msg.Class
        x1 = msg.xmin
        y1 = msg.ymin 
        x2 = msg.xmax
        y2 = msg.ymax
        self.square = (x2-x1)*(y2-y1)
        #print(self.square)

        # print(self.ytm_flag)
        

if __name__ == '__main__':
    try:
        test_track = tryout_main()

    except rospy.ROSInterruptException:
        pass
