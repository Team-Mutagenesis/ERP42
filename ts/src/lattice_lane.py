#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
import queue
import rospy
from math import cos, sin, pi, sqrt, pow, atan2
from erp_driver.msg import erpCmdMsg, erpStatusMsg
from geometry_msgs.msg import Point, PoseStamped, Point32, PoseArray, Pose
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry, Path
from detection_msgs.msg import BoundingBox
import numpy as np
import time
import math

class latticePlanner:
    def __init__(self):
        rospy.init_node('lattice_planner', anonymous=True)

        rospy.Subscriber("/lane_path", Path, self.path_callback)
        rospy.Subscriber("/erp42_status",erpStatusMsg, self.status_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/velodyne_points_cluster", PoseArray, self.object_callback)
        rospy.Subscriber("/rviz_local_path", Path, self.rviz_local_path_callback)
        #track
        #rospy.Subscriber("/cluster_dump_1", PoseArray, self.object_callback)
        #rospy.Subscriber("/cluster_cone_1", PoseArray, self.object_callback)

        self.lattice_path_pub = rospy.Publisher('/lattice_path', Path, queue_size = 1)
        self.select_path_pub = rospy.Publisher('select_path', Path, queue_size=1)

        self.is_path = False
        self.is_rviz_local_path = False
        self.is_status = False
        self.is_obj = False
        self.is_global_path = False
        
        # self.is_dump_data = False
        # self.is_cone_data =

        self.current_position = Point()
        self.vehicle_object_list= PoseArray()

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            start = time.time()
            print("local_path:", self.is_path,"status:", self.is_status, "object:", self.is_obj)
            
            if self.is_path and self.is_status and self.is_obj:
                self.vehicle_object_list.poses= []
                for num, i in enumerate(self.object_data.poses):
                    object_point = i.position

                    vehicle_axis_object_point =[object_point.x, object_point.y, 1]

                    tmp_pose = Pose()
                    tmp_pose.position.x = vehicle_axis_object_point[0]
                    tmp_pose.position.y = vehicle_axis_object_point[1]

                    self.vehicle_object_list.poses.append(tmp_pose)

                if self.checkObject(self.local_path, self.vehicle_object_list):
                    lattice_path = self.latticePlanner(self.local_path, self.status_msg)
                    lattice_path_index = self.collision_check(self.vehicle_object_list, lattice_path)
                    self.lattice_path_pub.publish(lattice_path[lattice_path_index])
                else:
                    self.lattice_path_pub.publish(self.local_path)

            #print("LATTICE TIME : %.8f" % (time.time() - start))

            rate.sleep()
    
    #track
    def dump_callback(self, msg):
        self.cluster_dump_data = msg
        self.is_dump_data = True
        

    def cone_callback(self, msg):
        self.is_cone_data = True
        
        
    def checkObject(self, ref_path, object_data):
        is_crash = False
        for obstacle in object_data.poses:
            for path in ref_path.poses:
                dis = sqrt(pow(path.pose.position.x - obstacle.position.x, 2) + pow(path.pose.position.y - (obstacle.position.y - 0.5), 2))
                if dis < 1.35:
                # if dis < 2.00:
                    is_crash = True
                    break

        return is_crash

    def collision_check(self, object_data, out_path):
        selected_lane = -1        
        lane_weight = [3, 2, 1, 1, 2, 3] #reference path 
        for obstacle in object_data.poses:
            for path_num in range(len(out_path)) :                    
                for path_pos in out_path[path_num].poses :                            
                    dis = sqrt(pow(obstacle.position.x  - path_pos.pose.position.x, 2) + pow((obstacle.position.y - 0.5) - path_pos.pose.position.y, 2))
                    if dis <1.5:
                    # if dis <2.5:
                        lane_weight[path_num] = lane_weight[path_num] + 100

        selected_lane = lane_weight.index(min(lane_weight)) 
        
        return selected_lane

    def odom_callback(self,msg):
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        self.vehicle_yaw = math.radians(msg.pose.pose.orientation.w)   

    def path_callback(self,msg):
        self.local_path = msg
        self.is_path = True

    def rviz_local_path_callback(self, msg):
        self.is_rviz_local_path = True
        self.rviz_local_path = msg

    def status_callback(self,msg):
        self.is_status = True
        self.status_msg = msg

    def object_callback(self,msg):
        self.is_obj = True
        self.object_data = msg

    def latticePlanner(self, ref_path, erp_status):
        out_path = []
        out_rviz_path = []

        vehicle_velocity = erp_status.speed / 10 * 3.6 # 최대 값 72
    
        look_distance = int(vehicle_velocity * 0.2 * 2) # 최대 값 28.8
        
        if look_distance < 5 :
            look_distance = 5                    

        if len(ref_path.poses) > look_distance :  
            local_ref_start_point      = (ref_path.poses[0].pose.position.x, ref_path.poses[0].pose.position.y)
            local_ref_start_point_1    =  np.array([[ref_path.poses[0].pose.position.x], [ref_path.poses[0].pose.position.y], [1]])
            local_ref_start_next_point = (ref_path.poses[1].pose.position.x, ref_path.poses[1].pose.position.y)
            local_ref_end_point = (ref_path.poses[look_distance * 2].pose.position.x, ref_path.poses[look_distance * 2].pose.position.y)
        
            theta = atan2(local_ref_start_next_point[1] - local_ref_start_point[1], local_ref_start_next_point[0] - local_ref_start_point[0])
            translation = [local_ref_start_point[0], local_ref_start_point[1]]

            trans_matrix    = np.array([    [cos(theta),                -sin(theta),                                                                      translation[0]], 
                                            [sin(theta),                 cos(theta),                                                                      translation[1]], 
                                            [         0,                          0,                                                                                  1 ]     ])

            det_trans_matrix = np.array([   [trans_matrix[0][0], trans_matrix[1][0],        -(trans_matrix[0][0] * translation[0] + trans_matrix[1][0] * translation[1])], 
                                            [trans_matrix[0][1], trans_matrix[1][1],        -(trans_matrix[0][1] * translation[0] + trans_matrix[1][1] * translation[1])],
                                            [                 0,                  0,                                                                                   1]     ])
            
            # det_trans_matrix = np.linalg.inv(trans_matrix)

            local_end_point = np.array([[local_ref_end_point[0]], [local_ref_end_point[1]], [1]])
            local_end_point = det_trans_matrix.dot(local_end_point)
            local_ego_vehicle_position = det_trans_matrix.dot(local_ref_start_point_1)
            # 차선 path의 첫번쨰 점에 offset 값으로 차량의 위치를 추종하면 된다.
            # 카메라로 인지된 path와 라이다로 인지된 vehicle axis 좌표계간의 translation을 수행해준다고생각하면 된다. 

            lane_off_set = [-4.0, -2.75, -1, 1, 2.75, 4.0]
            # lane_off_set = [-8.0, -6.75, -4, 4, 6.75, 8.0]

            local_lattice_points = []
            
            for i in range(len(lane_off_set)):
                local_lattice_points.append([local_end_point[0][0], local_end_point[1][0] + lane_off_set[i], 1])
                
            for end_point in local_lattice_points :
                lattice_path = Path()
                lattice_path_rviz = Path()
                
                lattice_path.header.frame_id = 'map'
                lattice_path_rviz.header.frame_id = 'map'

                x = []
                y = []
                x_interval = 0.5
                xs = 0
                xf = end_point[0]
                ps = local_ego_vehicle_position[1][0]

                pf = end_point[1]
                x_num = xf / x_interval

                for i in range(xs,int(x_num)) : 
                    x.append(i*x_interval)
                
                a = [0.0, 0.0, 0.0, 0.0]
                a[0] = ps
                a[1] = 0
                a[2] = 3.0 * (pf - ps) / (xf * xf)
                a[3] = -2.0 * (pf - ps) / (xf * xf * xf)

                # 3차 곡선 계획
                for i in x:
                    result = a[3] * i * i * i + a[2] * i * i + a[1] * i + a[0]
                    y.append(result)

                for i in range(0,len(y)) :
                    local_result = np.array([[x[i]], [y[i]], [1]])

                    read_pose = PoseStamped()
                    read_pose.pose.position.x = local_result[0][0]
                    read_pose.pose.position.y = local_result[1][0]
                    read_pose.pose.position.z = 0
                    read_pose.pose.orientation.x = 0
                    read_pose.pose.orientation.y = 0
                    read_pose.pose.orientation.z = 0
                    read_pose.pose.orientation.w = 1
                    lattice_path.poses.append(read_pose)

                out_path.append(lattice_path)
       
            add_point_size = min(int(vehicle_velocity * 2), len(ref_path.poses) )           
            
            for i in range(look_distance*2,add_point_size):
                if i+1 < len(ref_path.poses):
                    tmp_theta = atan2(ref_path.poses[i + 1].pose.position.y - ref_path.poses[i].pose.position.y,ref_path.poses[i + 1].pose.position.x - ref_path.poses[i].pose.position.x)                    
                    tmp_translation = [ref_path.poses[i].pose.position.x,ref_path.poses[i].pose.position.y]
                    tmp_t = np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]],
                                      [sin(tmp_theta), cos(tmp_theta), tmp_translation[1]], 
                                      [0, 0, 1]])

                    for lane_num in range(len(lane_off_set)) :
                        local_result = np.array([[0], [lane_off_set[lane_num]], [1]])
                        local_result = tmp_t.dot(local_result)

                        read_pose = PoseStamped()
                        read_pose.pose.position.x = local_result[0][0]
                        read_pose.pose.position.y = local_result[1][0]
                        read_pose.pose.position.z = 0
                        read_pose.pose.orientation.x = 0
                        read_pose.pose.orientation.y = 0
                        read_pose.pose.orientation.z = 0
                        read_pose.pose.orientation.w = 1
                        out_path[lane_num].poses.append(read_pose)

                        
            for i in range(len(out_rviz_path)):          
                globals()['lattice_pub_{}'.format(i+1)] = rospy.Publisher('/rviz_lattice_path_{}'.format(i+1),Path,queue_size=1)
               
        return out_path
  
if __name__ == '__main__':
    try:
        latticePlanner()
    except rospy.ROSInterruptException:
        pass

