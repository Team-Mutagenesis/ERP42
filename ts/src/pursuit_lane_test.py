#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
# from pid import PID_Control
import numpy as np
import tf
import time

from geometry_msgs.msg import Point,PoseWithCovarianceStamped, PoseStamped, QuaternionStamped
from nav_msgs.msg import Odometry, Path
from erp_driver.msg import erpStatusMsg, erpCmdMsg
from std_msgs.msg import Int16

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import cos, sin, pi , sqrt, pow, atan2


class pure_pursuit :

    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)
        rospy.Subscriber("/lattice_path", Path, self.path_callback) # Path msg type
        rospy.Subscriber("/lattice_flag", Int16, self.flag__callback) #lane or lattice
        '''
        About Ctrl Cmd msg type
    
        int32 longlCmdType

        float64 accel
        float64 brake
        float64 steering

        float64 velocity
        float64 acceleration
        '''

        # platform에게 값을 넘겨주기 위한 publisher
        self.ctrl_cmd_pub = rospy.Publisher('/erp42_ctrl_cmd',erpCmdMsg, queue_size=1)
        self.ctrl_cmd_msg = erpCmdMsg()
        # self.ctrl_cmd_msg.longlCmdType = 2

        self.is_path = False
        self.is_gps = False
        self.is_imu = False
        self.forward_point = Point()
        self.is_look_forward_point = False
        self.vehicle_length = 1 # platform length value : platform 종류에 따라 값 변동
        self.lfd = 1 # look forward distance : 전방 주시 거리

        self.is_path=False
        self.is_gps=False
        self.is_imu=False
        self.forward_point=Point()
        self.current_postion=Point()
        self.is_look_forward_point=False
        self.vehicle_length=1.04
        self.lfd= 2
        self.kp = 1
        self.theta_current = 0
        rate = rospy.Rate(30) 

        
        self.recovery_lane = None
        self.flag_path = 0

        while not rospy.is_shutdown():

            # 경로, gps, imu 값이 전부 들어오면 pure pursuit 내부 알고리즘 순환
            if self.is_path == True:
                print(1)
                self.is_look_forward_point = False 

                local_ref_start_point      = (self.path.poses[0].pose.position.x, self.path.poses[0].pose.position.y)
                local_ref_start_next_point = (self.path.poses[1].pose.position.x, self.path.poses[1].pose.position.y)
                theta = atan2(local_ref_start_next_point[1] - local_ref_start_point[1], local_ref_start_next_point[0] - local_ref_start_point[0])
                translation = [local_ref_start_point[0], local_ref_start_point[1]]
                trans_matrix    = np.array([[cos(theta),                -sin(theta),                                                                      translation[0]], 
                                            [sin(theta),                 cos(theta),                                                                      translation[1]], 
                                            [         0,                          0,                                                                                  1 ]     ])

                det_trans_matrix = np.array([[trans_matrix[0][0], trans_matrix[1][0],        -(trans_matrix[0][0] * translation[0] + trans_matrix[1][0] * translation[1])], 
                                            [trans_matrix[0][1], trans_matrix[1][1],        -(trans_matrix[0][1] * translation[0] + trans_matrix[1][1] * translation[1])],
                                            [                 0,                  0,                                                                                   1]     ])

                # path = way_point
                for num, way_point_poses in enumerate(self.path.poses) :
                    #recovery lane----------------------
                    if self.recovery_lane == way_point_poses and self.flag_path:
                        start_time = time.time()
                        while time.time() - start_time < 3.0:  # 3초 동안 반복
                            self.ctrl_cmd_msg.steer = 800
                            self.ctrl_cmd_msg.speed = 40
                            #self.is_path = False
                            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                            self.flag_path = False
                        break
                    #-----------------------------------
                    path_point = way_point_poses.pose.position

                    local_path_point = np.array([[path_point.x],[path_point.y],[1]]) # 좌표 x, y, z 값 : ENU with path value / z = 1 로 고정
                    local_path_point = trans_matrix.dot(local_path_point)
                    if local_path_point[0] > 0 :
                        dis = sqrt(pow(local_path_point[0], 2)+pow(local_path_point[1], 2))

                        if dis >= self.lfd :
                            self.forward_point=path_point
                            self.is_look_forward_point=True # 점 따라가기 활성화
                            break

                        else:
                            print("NUM :", num, "False dis :", dis)

                theta_current = atan2(local_path_point[1],local_path_point[0])

                if theta_current < -pi: theta_current += 2 * pi
                elif theta_current > pi : theta_current -=  2* pi
                
                # 라디안 값으로 나옴
                if self.is_look_forward_point :
                    # self.ctrl_cmd_msg.steer= -(atan2((2*self.vehicle_length*sin(theta_current)), self.lfd) * 71)
                    self.ctrl_cmd_msg.steer = -int(self.RAD2DEG(atan2((2 * self.vehicle_length * sin(theta_current)), self.lfd))) * 71

                    self.ctrl_cmd_msg.speed = 80

                    if self.ctrl_cmd_msg.steer <= -2000 : self.ctrl_cmd_msg.steer = -2000
                    if self.ctrl_cmd_msg.steer >= 2000 : self.ctrl_cmd_msg.steer = 2000

                    print("Steer :",self.ctrl_cmd_msg.steer)

                else :
                    print("no found forward point")

                    self.ctrl_cmd_msg.steer = 0
                    self.ctrl_cmd_msg.speed = 0

                self.is_path = False
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            else:
                print(2)
                self.ctrl_cmd_msg.steer = 0
                self.ctrl_cmd_msg.speed = 40
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            rate.sleep()


    # Path : waypoint data
    def path_callback(self,msg):
        self.is_path = True
        self.path = msg
        self.recovery_lane = self.path.poses[-1]

    def flag__callback(self, msg):
        self.flag_path = msg #True: lattice,  False: lane or None

    def RAD2DEG(self, x):
        deg = (x * 180 / pi)
        return deg


if __name__ == '__main__':
    try:
        test_track = pure_pursuit()

    except rospy.ROSInterruptException:
        pass