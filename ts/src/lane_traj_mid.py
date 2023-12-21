#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import numpy as np
from std_msgs.msg import String

from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
# from detection_msgs.msg import Lane
from erp_driver.msg import erpCmdMsg, erpStatusMsg
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Path

def basic_setting(name, id, color_r, color_g, color_b, color_a=255):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.ns = name
    marker.id = id
    marker.action = Marker.ADD
    marker.color = ColorRGBA(color_r / 255.0, color_g / 255.0, color_b / 255.0, color_a / 255.0)
    marker.pose.orientation.w = 1.0

    return marker

def linelist_rviz(name, id, lines, color_r=0, color_g=0, color_b=0, color_a=255, scale=0.05):
    marker = basic_setting(name, id, color_r, color_g, color_b, color_a)
    marker.type = Marker.LINE_LIST
    marker.scale.x = scale
    for line in lines:
        marker.points.append(Point(line[0], line[1], 0))

    return marker

def marker_array_append_rviz(marker_array, marker):
    marker_array.markers.append(marker)

    return marker_array


class Lane_trajectory(object):
    def __init__(self):
        self.debug_mode = True
        
        rospy.Subscriber('seg_result', String, self.seg_callback)
        rospy.Subscriber("/local_path", Path, self.path_callback) # Path msg type
     
        #rospy.Subscriber("/erp42_status", erpStatusMsg, self.status_callback)
        
        self.rviz_pub = rospy.Publisher("/visual_rviz", MarkerArray, queue_size=1)
        #self.steer_cmd_pub = rospy.Publisher("/erp42_ctrl_cmd", erpCmdMsg, queue_size = 1)
        self.mid_path_pub = rospy.Publisher('/lane_path',Path, queue_size=1)

        self.lane_path_msg = Path()
        self.lane_path_msg.header.frame_id='/map'

        self.erp_command =erpCmdMsg()
        self.erp_command.speed = 50

        self.max_len = 0
        self.left_min = -3
        self.right_min = 3
        self.left_lane = []
        self.right_lane = []
        self.velocity_x = 80
        self.steer_cmd_old = 0.0
        self.steer_cmd = 0.0
        self.alpha = 0.5

        self.change_point = Point()
        self.is_gps = True

        # self.change_point.x = 315368.4480704938	 
        # self.change_point.y = 4071529.088993086

        self.change_point.x = 302646.4049480326	
        self.change_point.y = 4123679.3384170025

        
        print("ready...")
        
    # def status_callback(self,data):
    #      self.velocity_x = data.velocity
    #      if self.velocity_x == 0.0: self.velocity_x = 1
        # GPS UTM 좌표 값   
    def path_callback(self, msg):
        #print(msg.poses[0].pose.position.x, msg.poses[0].pose.position.y)
        if msg.poses[0].pose.position.x == self.change_point.x and msg.poses[0].pose.position.y == self.change_point.y:
            self.is_gps = False



    def seg_callback(self, data):
        seg_result = eval(data.data)
        
        if len(seg_result) > 0 and not self.is_gps:
            st = time.time()
            lane_result = []
            rviz_result = []
            x_min = 0
            x_max = 30
            for lane in seg_result:
                first = lane[1][0][0]
                last = lane[1][-1][0]
                if abs(last-first) < 5: continue
                tmp = []
                tmp2 = []
                x = np.array([point[0] for point in lane[1]])
                y = np.array([point[1] for point in lane[1]])
                
                coefficients = np.polyfit(x, y, deg=2)
                poly = np.poly1d(coefficients)    
                
                x_values = np.linspace(x_min, x_max, num=30)
                y_values = poly(x_values)
                
                if poly(0) < 0 and poly(0) > self.left_min:
                    self.left_lane = y_values
                elif poly(0) > 0 and poly(0) < self.right_min:
                    self.right_lane = y_values
                # if poly(0) < 0:
                #     if self.left_lane == [] and poly(0) > self.left_min:
                #         self.left_lane = y_values
                #     elif self.left_lane != [] and poly(0) > self.left_lane[0]:
                #         self.left_lane = y_values
                # elif poly(0) > 0:
                #     if self.right_lane == [] and poly(0) < self.right_min:
                #         self.right_lane = y_values
                #     elif self.right_lane != [] and poly(0) < self.right_lane[0]:
                #         self.right_lane = y_values

                

                
                for x,y in zip(x_values, y_values):
                    tmp.append([x,y])
                lane_result.append([lane[0], tmp])

                if self.debug_mode:
                    x_values = np.linspace(-30, 30, num=30)
                    y_values = poly(x_values)
                    for x,y in zip(x_values, y_values):
                        tmp2.append([x,y])
                    rviz_result.append([lane[0], tmp2])


           
            traj_x = np.linspace(x_min, x_max, num=30)
            traj_y = np.mean([np.array(self.left_lane), np.array(self.right_lane)], axis=0)
                
            for tmp_x, tmp_y in zip(traj_x, traj_y):
                # tmp = lines[i].rstrip("\n").split(',')
                # tmp = lines[i].split()
                read_pose=PoseStamped()
                read_pose.pose.position.x=float(tmp_x)
                read_pose.pose.position.y=float(tmp_y) 
                self.lane_path_msg.poses.append(read_pose)
                
            #중앙좌표값    
            self.mid_path_pub.publish(self.lane_path_msg)
            self.lane_path_msg.poses=[]

            if len(traj_y) != 0:

                offset = 0.18 #0.17
                x0 = traj_x[0]
                y0 = traj_y[0]+offset
                x1 = traj_x[1]
                y1 = traj_y[1]+offset
                psi = np.arctan2((y1-y0),(x1-x0))
                e = y0+offset
                # (~80km/h)
                # angle_error_gain = 0.5
                # k = 0.75
                angle_error_gain = 0.5
                k = 1
                steerangle = (angle_error_gain*psi + np.arctan2(k*e, self.velocity_x))*180/np.pi
                steerangle = 8.72*steerangle
                steerangle = self.alpha*self.steer_cmd_old + (1-self.alpha)*steerangle
                self.steer_cmd_old = steerangle
                print("angle loss: {}, e loss: {}, e: {}".format(psi*180/np.pi,np.arctan2(k*e, self.velocity_x)*180/np.pi, e))

                # self.steer_cmd.data = int(steerangle)
                # self.steer_cmd.header.stamp = rospy.Time.now()
                # self.steer_cmd.header.frame_id = "lane"
                
                self.steer_cmd = -int(steerangle*71)

                if self.steer_cmd >2000:
                    self.steer_cmd =2000

                if self.steer_cmd < -2000:
                    self.steer_cmd = -2000
            
                self.erp_command.steer = self.steer_cmd
                # self.steer_cmd_pub.publish(self.erp_command)
                # print("steerangle =",self.steer_cmd)
                
            
                tmp = []
                for x,y in zip(traj_x, traj_y):
                    tmp.append([x,y])
                rviz_result.append([11, tmp])
                
                if self.debug_mode:
                    all_markers = MarkerArray()
                    ids = list(range(0,50))
                    
                    for lane in rviz_result:
                        if lane[0] == 0:
                            color_r = 255
                            color_g = 255
                            color_b = 255
                        elif lane[0] == 1:
                            color_r = 0
                            color_g = 255
                            color_b = 0
                        elif lane[0] == 2 or lane[0] == 3:
                            color_r = 255
                            color_g = 200
                            color_b = 0
                        elif lane[0] == 4 or lane[0] == 5:
                            color_r = 0
                            color_g = 0
                            color_b = 255
                        else:
                            color_r = 255
                            color_g = 0
                            color_b = 0
                            
                        points = []
                        for i in range(len(lane[1])-1):
                            points.append(lane[1][i])
                            points.append(lane[1][i+1])
                            
                        lane_marker = linelist_rviz(
                            name="linelist",
                            id=ids.pop(),
                            lines=points,
                            color_r=color_r,
                            color_g=color_g,
                            color_b=color_b,
                            scale=0.2
                        )
                        marker_array_append_rviz(all_markers, lane_marker)
                    if len(lane_result) < self.max_len:
                        for i in range(self.max_len - len(lane_result)):
                            lane_marker = linelist_rviz(
                                name="linelist",
                                id=ids.pop(),
                                lines=[[0,0],[1,1]],
                                color_r=0,
                                color_g=0,
                                color_b=0,
                                scale=0
                            )
                            marker_array_append_rviz(all_markers, lane_marker)
                    self.max_len = max(self.max_len, len(lane_result))
                    self.rviz_pub.publish(all_markers)
                    # print("process time: {}".format(time.time()-st))
                else:
                    print("Not Found")


if __name__=='__main__':
    rospy.init_node("lane_trajectory")
    detector = Lane_trajectory()
    rospy.spin()
    