#!/usr/bin/env python
# -*- coding: utf-8 -*-

from traceback import print_last
import rospy
import math
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseArray, PoseStamped, Point
from erp_driver.msg import erpCmdMsg, erpStatusMsg
from nav_msgs.msg import Odometry, Path
import time
import numpy as np
from visualization_msgs.msg import Marker
import warnings
from math import cos, sin, pi, sqrt, pow, atan2
from tf.transformations import euler_from_quaternion

# 경고를 무시하도록 설정
warnings.filterwarnings("ignore", category=np.RankWarning)


class rubber_cone_pursuit:
    def __init__(self):
        rospy.init_node('rubber_cone_center', anonymous=True)
        rospy.Subscriber('/cluster_blue_1', PoseArray, self.cluster_blue_callback)
        rospy.Subscriber('/cluster_yellow_1', PoseArray, self.cluster_yellow_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/erp42_status", erpStatusMsg, self.status_callback)

        self.ctrl_cmd_pub = rospy.Publisher('/erp42_ctrl_cmd', erpCmdMsg, queue_size=1)
        self.center_point_pub = rospy.Publisher('/center_point', PoseStamped, queue_size=1)
        self.marker_left_pub = rospy.Publisher('polyfit_marker_left', Marker, queue_size=10)
        self.marker_right_pub = rospy.Publisher('polyfit_marker_right', Marker, queue_size=10)
        self.marker_middle_pub = rospy.Publisher('polyfit_marker_middle', Marker, queue_size=10)
        self.path_pub = rospy.Publisher('path', Path, queue_size=10)

        self.status_msg = erpStatusMsg()
        self.center_point = PoseStamped()
        self.ctrl_cmd_msg = erpCmdMsg()
        self.forward_point = Point()
        self.current_position = Point()
        self.pid = pidControl()


        self.path = Path()
        self.path.header.frame_id='map'

        # Marker 메시지 생성
        self.marker_middle = Marker()
        self.marker_middle.header.frame_id = 'map'  # rviz에서 사용할 좌표 프레임 설정
        self.marker_middle.header.stamp = rospy.Time.now()
        self.marker_middle.ns = 'polyfit_data'
        self.marker_middle.id = 0
        self.marker_middle.type = Marker.LINE_STRIP
        self.marker_middle.action = Marker.ADD
        self.marker_middle.pose.orientation.w = 1.0
        self.marker_middle.scale.x = 0.05  # 선의 두께 설정
        self.marker_middle.color.g = 1.0 # 선 색상 설정 (빨간색)
        self.marker_middle.color.a = 1.0  # 투명도 설정 (0.0: 투명, 1.0: 불투명)

        # Marker 메시지 생성
        self.marker_left = Marker()
        self.marker_left.header.frame_id = 'map'  # rviz에서 사용할 좌표 프레임 설정
        self.marker_left.header.stamp = rospy.Time.now()
        self.marker_left.ns = 'polyfit_data'
        self.marker_left.id = 0
        self.marker_left.type = Marker.LINE_STRIP
        self.marker_left.action = Marker.ADD
        self.marker_left.pose.orientation.w = 1.0
        self.marker_left.scale.x = 0.05  # 선의 두께 설정
        self.marker_left.color.g = 1.0  # 선 색상 설정 (빨간색)
        self.marker_left.color.a = 1.0  # 투명도 설정 (0.0: 투명, 1.0: 불투명)

        # Marker 메시지 생성
        self.marker_right = Marker()
        self.marker_right.header.frame_id = 'map'  # rviz에서 사용할 좌표 프레임 설정
        self.marker_right.header.stamp = rospy.Time.now()
        self.marker_right.ns = 'polyfit_data'
        self.marker_right.id = 0
        self.marker_right.type = Marker.LINE_STRIP
        self.marker_right.action = Marker.ADD
        self.marker_right.pose.orientation.w = 1.0
        self.marker_right.scale.x = 0.05  # 선의 두께 설정
        self.marker_right.color.g = 1.0  # 선 색상 설정 (빨간색)
        self.marker_right.color.a = 1.0  # 투명도 설정 (0.0: 투명, 1.0: 불투명)

        self.center_point.header.frame_id = "map"

        self.is_blue_cluster = False
        self.is_yellow_cluster = False
        self.is_status = True
        self.is_odom = True
        self.is_look_forward_point = False
        
        self.vehicle_length = 1.04
        self.lfd = 1.5
        self.vehicle_yaw = 0.0


        self.kp = 1
        self.ki = 0
        self.kd = 0
        self.p_error = 0
        self.i_error = 0
        self.d_error = 0
        self.pid_steer = 0


        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            print(self.is_blue_cluster, self.is_yellow_cluster, self.is_status, self.is_odom)
            if (self.is_blue_cluster or self.is_yellow_cluster) and self.is_status and self.is_odom:
                start = time.time()

                left_rubber_x = [] # yellow cone
                left_rubber_y = [] # yellow cone
                
                right_rubber_x = [] # blue cone
                right_rubber_y = [] # blue cone

                # x축 : 전방, y축 : 좌,우
                if self.is_yellow_cluster:
                    for i in range(len(self.cluster_yellow_data.poses)):
                        left_rubber_x.append(self.cluster_yellow_data.poses[i].position.x)
                        left_rubber_y.append(self.cluster_yellow_data.poses[i].position.y)
                
                if self.is_blue_cluster:
                    for i in range(len(self.cluster_blue_data.poses)):
                        right_rubber_x.append(self.cluster_blue_data.poses[i].position.x)
                        right_rubber_y.append(self.cluster_blue_data.poses[i].position.y)
            
                degree = 2

                # 양쪽 둘다 잡았을 때
                print(self.is_blue_cluster, self.is_yellow_cluster)
                print(left_rubber_x, right_rubber_x)

                if len(left_rubber_x) == 1 and len(left_rubber_y) == 1 and len(right_rubber_x) == 1 and len(right_rubber_y) == 1:
                        point_middle = Point()
                        point_middle.x = (left_rubber_x[0] + right_rubber_x[0]) / 2
                        point_middle.y = (left_rubber_y[0] + right_rubber_y[0]) / 2
                        point_middle.z = 0.0  
                        self.marker_middle.points.append(point_middle)

                        read_pose = PoseStamped()
                        read_pose.pose.position.x = point_middle.x
                        read_pose.pose.position.y = point_middle.y

                        self.path.poses.append(read_pose)
                

                # 점
                elif len(left_rubber_x) == 1 and len(left_rubber_y) == 1 and (not right_rubber_x and not right_rubber_y):
                        point_left = Point()
                        point_left.x = left_rubber_x[0] 
                        point_left.y = left_rubber_y[0] - 1.5
                        point_left.z = 0.0  
                        self.marker_left.points.append(point_left)

                        read_pose = PoseStamped()
                        read_pose.pose.position.x = point_left.x
                        read_pose.pose.position.y = point_left.y
                        
                        
                        self.path.poses.append(read_pose)
                # 점
                elif len(right_rubber_x) == 1 and len(right_rubber_y) == 1 and (not left_rubber_x and not left_rubber_y):
                        point_right = Point()
                        point_right.x = right_rubber_x[0] 
                        point_right.y = right_rubber_y[0] + 1.5
                        point_right.z = 0.0  
                        self.marker_right.points.append(point_right)
                        
                        read_pose = PoseStamped()
                        read_pose.pose.position.x = point_right.x
                        read_pose.pose.position.y = point_right.y

                        self.path.poses.append(read_pose)

                # 중점
                elif left_rubber_x and left_rubber_y and right_rubber_x and right_rubber_y:    
                    coefficients_left = np.polyfit(left_rubber_x, left_rubber_y, degree)
                    coefficients_right = np.polyfit(right_rubber_x, right_rubber_y, degree)

                    x_range_left = np.linspace(min(left_rubber_x), max(left_rubber_x),20)  
                    x_range_right = np.linspace(min(right_rubber_x), max(right_rubber_x), 20)  

                    offset = []
                    for x_left, x_right in zip(x_range_left, x_range_right):
                        if not offset:
                            offset.append((x_left + x_right) / 2)
                            offset.append((np.polyval(coefficients_left, x_left) + np.polyval(coefficients_right, x_right)) / 2)

                        y_left = np.polyval(coefficients_left, x_left)
                        y_right = np.polyval(coefficients_right, x_right)
                        point_middle = Point()
                        point_middle.x = (x_left + x_right) / 2 
                        point_middle.y = (y_left + y_right) / 2
                        point_middle.z = 0.0  # 2D 데이터이므로 Z 축은 0
                        self.marker_middle.points.append(point_middle)

                        read_pose = PoseStamped()
                        read_pose.pose.position.x = point_middle.x
                        read_pose.pose.position.y = point_middle.y

                        self.path.poses.append(read_pose)

                # 왼쪽만 잡았을 때
                elif left_rubber_x and left_rubber_y:
                    self.path = Path()

                    coefficients_left = np.polyfit(left_rubber_x, left_rubber_y, degree)
                    x_range_left = np.linspace(min(left_rubber_x), max(left_rubber_x), 20)  
                
                    offset = []
                    for x in x_range_left:
                        if not offset:
                            offset.append(x)
                            offset.append(np.polyval(coefficients_left, x))

                        if offset[0] > 2.0:
                            y = np.polyval(coefficients_left, x)
                            point_left = Point()
                            point_left.x = x - offset[0] + 1.4
                            point_left.y = y - offset[1]  
                            point_left.z = 0.0  
                            self.marker_left.points.append(point_left)

                            read_pose = PoseStamped()
                            read_pose.pose.position.x = point_left.x
                            read_pose.pose.position.y = point_left.y

                            self.path.poses.append(read_pose)

                        else:
                            print("yy")
                            y = np.polyval(coefficients_left, x)
                            point_left = Point()
                            point_left.x = x - offset[0] 
                            point_left.y = y - offset[1] 
                            point_left.z = 0.0  
                            self.marker_left.points.append(point_left)

                            read_pose = PoseStamped()
                            read_pose.pose.position.x = point_left.x
                            read_pose.pose.position.y = point_left.y

                            self.path.poses.append(read_pose)
                    
                    # self.lfd = offset[0] / 2 + 0.3
  
                # 오른쪽만 잡았을 때
                elif right_rubber_x and right_rubber_y:
                    self.path = Path()

                    coefficients_right = np.polyfit(right_rubber_x, right_rubber_y, degree)
                    x_range_right = np.linspace(min(right_rubber_x), max(right_rubber_x), 20)
                    
                    offset = []
                    for x in x_range_right:
                        if not offset:
                            offset.append(x)
                            offset.append(np.polyval(coefficients_right, x))
                        
                        if offset[0] > 2.0:
                            y = np.polyval(coefficients_right, x)
                            point_right = Point()
                            point_right.x = x - offset[0] + 1.3
                            point_right.y = y - offset[1] 
                            point_right.z = 0.0  
                            self.marker_right.points.append(point_right)
                            
                            read_pose = PoseStamped()
                            read_pose.pose.position.x = point_right.x
                            read_pose.pose.position.y = point_right.y

                            self.path.poses.append(read_pose)

                        else:
                            print("ee")
                            y = np.polyval(coefficients_right, x)
                            point_right = Point()
                            point_right.x = x - offset[0] 
                            point_right.y = y - offset[1]
                            point_right.z = 0.0  
                            self.marker_right.points.append(point_right)
                            
                            read_pose = PoseStamped()
                            read_pose.pose.position.x = point_right.x
                            read_pose.pose.position.y = point_right.y

                            self.path.poses.append(read_pose)




                # 아무것도 못잡았을 때
                else:
                    self.ctrl_cmd_msg.steer = 0
                    self.ctrl_cmd_msg.speed = 50

                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

                    rate.sleep()

                # pure_pursuit
                # 라바콘 한개만 잡았을 때
                if len(self.path.poses) == 1:
                
                    path_point = self.path.poses[0].pose.position

                else:
                    for num, i in enumerate(self.path.poses):
                        path_point = i.pose.position
                        dis = sqrt(pow(path_point.x, 2) + pow(path_point.y, 2))

                        print(dis, self.lfd)
                        if dis >= self.lfd:
                            self.forward_point = path_point
                            break

                theta = self.RAD2DEG(atan2(path_point.y, path_point.x))

                if theta < 0: theta += 360

                if theta < - 180:
                    theta += 360

                elif theta > 180:
                    theta -= 360

                goal_steering = -int(theta * 71)
                current_steering = -int(atan2((2 * self.vehicle_length * sin(theta)), self.lfd) * 71)

                cte = goal_steering - current_steering
                pid_steering = self.steer_pid(cte)

                if pid_steering <= -2000: pid_steering = -2000
                if pid_steering >= 2000: pid_steering = 2000

                self.ctrl_cmd_msg.steer = pid_steering
                self.ctrl_cmd_msg.speed = 50

                print("STEER", self.ctrl_cmd_msg.steer)
                print("SPEED", self.ctrl_cmd_msg.speed)       

                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                self.marker_middle_pub.publish(self.marker_middle)
                self.marker_left_pub.publish(self.marker_left)
                self.marker_right_pub.publish(self.marker_right)
                self.path_pub.publish(self.path)

                self.marker_middle.points = []
                self.marker_left.points = []
                self.marker_right.points = []
                
                self.path = Path()
                self.path.header.frame_id='map'
                self.is_blue_cluster = False
                self.is_yellow_cluster = False

            rate.sleep()



    def RAD2DEG(self, x):
        deg = (x * 180 / pi)
        return deg
        

    def cluster_blue_callback(self, msg):
        self.cluster_blue_data = msg
        self.is_blue_cluster = True
    

    def cluster_yellow_callback(self, msg):
        self.cluster_yellow_data = msg
        self.is_yellow_cluster = True


    def odom_callback(self, msg):
        self.is_odom = True
        # self.vehicle_yaw = msg.pose.pose.orientation.w
        self.current_position.x = msg.pose.pose.position.x 
        self.current_position.y = msg.pose.pose.position.y 


    def status_callback(self, msg):  
        self.is_status = True
        self.status_msg = msg


    def steer_pid(self, cte):
        self.d_error = cte - self.p_error
        self.p_error = cte
        self.i_error += cte

        return self.kp * self.p_error + self.ki * self.i_error + self.kd * self.d_error
    

    def RAD2DEG(self, x):
        deg = (x * 180 / math.pi)
        return deg

    
    def distance_between_points(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


    def line_equation(self, x1, y1, x2, y2):
        if x1 == x2:
            slope = None
            y_intercept = None

        else:
            slope = (y2 - y1) / (x2 - x1)
            y_intercept = y1 - slope * x1

        return slope, y_intercept


    def midpoint(self, x1, y1, x2, y2):
        mid_x = (x1 + x2) / 2
        mid_y = (y1 + y2) / 2

        return mid_x, mid_y


    def ccw(self, p1, p2, p3):
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3

        val = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1)
        if val > 0:
            return 1  # 왼쪽에 위치

        elif val < 0:
            return -1  # 오른쪽에 위치


class pidControl:
    def __init__(self):
        self.p_gain = 1.0
        self.i_gain = 0.0
        self.d_gain = 0.0
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02


    def pid(self, target_vel, current_vel):
        error = target_vel - current_vel

        # PID 제어 생성
        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error - self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output
  

if __name__ == '__main__':
    try:
        rubber_cone = rubber_cone_pursuit()

    except rospy.ROSInterruptException:
        pass
