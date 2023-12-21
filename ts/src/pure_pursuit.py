#!/usr/bin/env python
# -*- coding: utf-8 -*-

from os import path
import time
import rospy
from math import cos, sin, pi, sqrt, pow, atan2
import numpy as np
from tf.transformations import euler_from_quaternion
import math

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from erp_driver.msg import erpCmdMsg, erpStatusMsg

class pure_pursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)
        rospy.Subscriber("/local_path", Path, self.path_callback) # Path msg type
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/erp42_status", erpStatusMsg, self.status_callback)
    

        # platform에게 값을 넘겨주기 위한 publisher
        self.ctrl_cmd_pub = rospy.Publisher('/pursuit_to_main',erpCmdMsg, queue_size=1)
        self.ctrl_cmd_msg = erpCmdMsg()
        # self.ctrl_cmd_msg.longlCmdType = 2

        self.is_path=False
        self.is_odom=False
        self.forward_point=Point()
        self.current_position=Point()
        self.is_look_forward_point=False
        self.vehicle_length=1.04
        self.lfd = 3
        self.kp = 1
        self.theta_current = 0
        rate = rospy.Rate(30) 
    
        while not rospy.is_shutdown():

            # 경로, gps, imu 값이 전부 들어오면 pure pursuit 내부 알고리즘 순환
            print(self.is_path, self.is_odom)
            if self.is_path == True and self.is_odom ==True:

                vehicle_position = self.current_position # 차량의 현재 좌표 : UTM with GPS Value
                self.is_look_forward_point = False 

                translation=[vehicle_position.x, vehicle_position.y]

                # yaw 값을 사용하기 위한 회전변환 행렬
                t = np.array([
                        [cos(self.vehicle_yaw), -sin(self.vehicle_yaw),translation[0]],
                        [sin(self.vehicle_yaw),cos(self.vehicle_yaw),translation[1]],
                        [0                    ,0                    ,1            ]])

                # ?
                det_t = np.array([
                        [t[0][0],t[1][0],-(t[0][0]*translation[0]+t[1][0]*translation[1])],
                        [t[0][1],t[1][1],-(t[0][1]*translation[0]+t[1][1]*translation[1])],
                        [0      ,0      ,1                                               ]])

                # path = way_point
                print(self.path.poses)
                for num, way_point_poses in enumerate(self.path.poses) :
                    path_point = way_point_poses.pose.position

                    global_path_point = [path_point.x,path_point.y,1] # 좌표 x, y, z 값 : ENU with path value / z = 1 로 고정

                    # det_t 와 global_path_point 내적하여 local_path 생성
                    # np.array_a.dot(array_b) : array_a 와 array_b 내적
                    local_path_point = det_t.dot(global_path_point) # 회전 변환한 waypoint 좌표
                    #print("local_path_point :", local_path_point)

                    #print(local_path_point)
                    # if local_path_point[0] > 0 :
                    dis = sqrt(pow(local_path_point[0], 2)+pow(local_path_point[1], 2))

                    if dis >= self.lfd :
                        self.forward_point=path_point
                        self.is_look_forward_point=True # 점 따라가기 활성화
                        break

                    else:
                        print("NUM :", num, "False dis :", dis)
   

                theta_current = atan2(local_path_point[1],local_path_point[0])
                #print(theta_current)

                if theta_current < -pi: theta_current += 2 * pi
                elif theta_current > pi : theta_current -=  2* pi


                # 라디안 값으로 나옴
                if self.is_look_forward_point :

                    self.ctrl_cmd_msg.steer= -int(self.RAD2DEG(atan2((2*self.vehicle_length*sin(theta_current)), self.lfd))) * 71
                    self.ctrl_cmd_msg.speed = 50

                    if self.ctrl_cmd_msg.steer <= -2000 : self.ctrl_cmd_msg.steer = -2000
                    if self.ctrl_cmd_msg.steer >= 2000 : self.ctrl_cmd_msg.steer = 2000

                    print("Steer :",self.ctrl_cmd_msg.steer)

                else :
                    print("no found forward point")

                    self.ctrl_cmd_msg.steer=0
                    self.ctrl_cmd_msg.speed=0

                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            rate.sleep()
    
    def RAD2DEG(self, x):
        deg = (x * 180 / pi)
        return deg

    # Path : waypoint data
    def path_callback(self,msg):
        self.is_path = True
        self.path = msg

    def odom_callback(self, msg):
        self.is_odom = True
        self.odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw = euler_from_quaternion(self.odom_quaternion)
        
        #self.vehicle_yaw = math.radians(msg.pose.pose.orientation.w)    
        self.current_position.x = msg.pose.pose.position.x 
        self.current_position.y = msg.pose.pose.position.y

        #print(self.RAD2DEG(self.vehicle_yaw))

    def status_callback(self, msg):  ## Vehicl Status Subscriber
        self.is_status = True
        self.status_msg = msg

  
    def steer_pid(self,target_theta, current_theta):
        pid_theta = self.kp * (target_theta - current_theta)
        return pid_theta
        

if __name__ == '__main__':
    try:
        test_track = pure_pursuit()

    except rospy.ROSInterruptException:
        pass