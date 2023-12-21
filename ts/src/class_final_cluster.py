#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from sklearn.cluster import DBSCAN
import numpy as np
import pcl
import ctypes
import struct
import time

class lidar:
    def __init__(self):
        rospy.init_node('LiDAR', anonymous=True)
        rospy.Subscriber('/velodyne_points', PointCloud2, self.pcl_callback)
        # rospy.Subscriber('/Combined_velo', PointCloud2, self.pcl_callback)


        self.pub1 = rospy.Publisher("/velodyne_points_cluster", PoseArray, queue_size=10)
        self.pub2 = rospy.Publisher("/velodyne_points_passthrough", PointCloud2, queue_size=1)
        self.pub3 = rospy.Publisher("/bounding_box", Marker, queue_size=10)

        self.dbscan = DBSCAN(eps=0.5, min_samples=5)

        self.cloud = PointCloud2()
        self.cluster_msg = PoseArray()
        self.marker_msg = Marker()
        self.marker_msg.header.frame_id = 'lidar_link'  # 표시할 좌표계 설정
        self.marker_msg.type = Marker.CUBE_LIST
        self.marker_msg.action = Marker.ADD

        self.pcl_data = False
        self.marker_msg.color.r = 1.0  # bounding box 색상 설정
        self.marker_msg.color.g = 0.0
        self.marker_msg.color.b = 0.0
        self.marker_msg.color.a = 0.8

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            if self.pcl_data:
                start = time.time()
                self.cluster_msg.poses = []
                self.marker_msg.points = []

                
                self.cloud = self.do_voxel_grid_downsampling(self.cloud, 0.1)

                # # ROI 설정 erp
                # self.cloud = self.do_passthrough(self.cloud, 'x', 0.0, 4.5)
                # self.cloud = self.do_passthrough(self.cloud, 'y', -3.5, 3.5)
                # self.cloud = self.do_passthrough(self.cloud, 'z', -0.4, 2.5)
                
                # ROI 설정 erp test
                self.cloud = self.do_passthrough(self.cloud, 'x', 0.0, 4.5)
                self.cloud = self.do_passthrough(self.cloud, 'y', -1.5, 1.5)
                self.cloud = self.do_passthrough(self.cloud, 'z', -0.6, 1.0)


                # 아이오닉
                # self.cloud = self.do_passthrough(self.cloud, 'x', 0.0, 50.0)
                # self.cloud = self.do_passthrough(self.cloud, 'y', -5.5, 5.5)
                # self.cloud = self.do_passthrough(self.cloud, 'z', -0.8, 2.5)
                # _, _, self.cloud = self.do_ransac_plane_normal_segmentation(self.cloud, 0.05)

                cloud_new = self.pcl_to_ros(self.cloud)  # PCL을 ROS 메시지로 변경
                cloud_new.header.frame_id = 'map'
                
                self.pub2.publish(cloud_new)

                pc_np = self.pointcloud2_to_xyz(cloud_new)
                if len(pc_np) == 0:
                        # print("1", len(self.cluster_msg.poses))
                        pass

                else:
                    pc_xy = pc_np[:, :2]
                    db = self.dbscan.fit_predict(pc_xy)

                    n_cluster = np.max(db) + 1

                    self.cluster_msg.header.frame_id = 'map'

                    cluster_list = []

                    for c in range(n_cluster):
                        # TODO: (2) 각 Cluster를 대표하는 위치 값 계산
                        # DBSCAN으로 Clustering 된 각 Cluster의 위치 값을 계산하는 영역입니다.
                        # Cluster에 해당하는 Point들을 활용하여 Cluster를 대표할 수 있는 위치 값을 계산합니다.
                        # 계산된 위치 값을 ROS geometry_msgs/Pose type으로 입력합니다.
                        # Input : cluster
                        # Output : cluster position x,y

                        c_tmp = np.mean(pc_xy[db == c, :], axis=0)
                        # print(c_tmp)
                        tmp_pose = Pose()
                        tmp_pose.position.x = c_tmp.tolist()[0]
                        # print(tmp_pose.position.x)
                        tmp_pose.position.y = c_tmp.tolist()[1]

                        # ===========================================================
                        point = Point(c_tmp.tolist()[0], c_tmp.tolist()[1], 0)
                        cluster_min_x = np.min(pc_np[db == c, 0])
                        cluster_max_x = np.max(pc_np[db == c, 0])
                        cluster_min_y = np.min(pc_np[db == c, 1])
                        cluster_max_y = np.max(pc_np[db == c, 1])
                        cluster_min_z = np.min(pc_np[db == c, 2])
                        cluster_max_z = np.max(pc_np[db == c, 2])

                        cluster_width = cluster_max_x - cluster_min_x
                        cluster_length = cluster_max_y - cluster_min_y
                        cluster_height = cluster_max_z - cluster_min_z

                        self.marker_msg.points.append(point)
                        self.marker_msg.scale.x = cluster_width
                        self.marker_msg.scale.y = cluster_length
                        self.marker_msg.scale.z = cluster_height
                        # ============================================================

                        self.cluster_msg.poses.append(tmp_pose)

                # print(self.cluster_msg.poses)
                self.pub1.publish(self.cluster_msg)
                self.pub3.publish(self.marker_msg)
                print("LIDAR TIME : %.8f" % (time.time() - start))

                pass
            

    def pcl_callback(self, msg):
        self.cloud = self.ros_to_pcl(msg)
        self.pcl_data = True


    def ros_to_pcl(self, ros_cloud):
        """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB

            Args:
                ros_cloud (PointCloud2): ROS PointCloud2 message

            Returns:
                pcl.PointCloud_PointXYZRGB: PCL XYZRGB point cloud
        """
        points_list = []

        
        for data in pc2.read_points(ros_cloud, skip_nans=True):
            points_list.append([data[0], data[1], data[2], data[3]]) # X, Y, Z, Color Data

        pcl_data = pcl.PointCloud_PointXYZRGB()
        pcl_data.from_list(points_list)

        return pcl_data


    def pcl_to_ros(self, pcl_array):
        """ Converts a pcl PointXYZRGB to a ROS PointCloud2 message

            Args:
                pcl_array (PointCloud_PointXYZRGB): A PCL XYZRGB point cloud

            Returns:
                PointCloud2: A ROS point cloud
        """
        ros_msg = PointCloud2()

        ros_msg.header.stamp = rospy.Time.now()
        ros_msg.header.frame_id = "/map"

        ros_msg.height = 1
        ros_msg.width = pcl_array.size

        ros_msg.fields.append(PointField(
            name="x",
            offset=0,
            datatype=PointField.FLOAT32, count=1))
        ros_msg.fields.append(PointField(
            name="y",
            offset=4,
            datatype=PointField.FLOAT32, count=1))
        ros_msg.fields.append(PointField(
            name="z",
            offset=8,
            datatype=PointField.FLOAT32, count=1))
        ros_msg.fields.append(PointField(
            name="rgb",
            offset=16,
            datatype=PointField.FLOAT32, count=1))

        ros_msg.is_bigendian = False
        ros_msg.point_step = 32
        ros_msg.row_step = ros_msg.point_step * ros_msg.width * ros_msg.height
        ros_msg.is_dense = False
        buffer = []

        for data in pcl_array:
            s = struct.pack('>f', data[3])
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value

            r = (pack & 0x00FF0000) >> 16
            g = (pack & 0x0000FF00) >> 8
            b = (pack & 0x000000FF)

            buffer.append(struct.pack('ffffBBBBIII', data[0], data[1], data[2], 1.0, b, g, r, 0, 0, 0, 0))

        ros_msg.data = b"".join(buffer)

        return ros_msg


    def do_passthrough(self, pcl_data, filter_axis, axis_min, axis_max):
        """
        Create a PassThrough object and assigns a filter axis and range.
        :param pcl_data: point cloud data subscriber
        :param filter_axis: filter axis
        :param axis_min: Minimum axis to the passthrough filter object
        :param axis_max: Maximum axis to the passthrough filter object
        :return: passthrough on point cloud
        """
        passthrough = pcl_data.make_passthrough_filter()
        passthrough.set_filter_field_name(filter_axis)
        passthrough.set_filter_limits(axis_min, axis_max)
        return passthrough.filter()


    def pointcloud2_to_xyz(self, cloud_msg):
        point_list = []

        for point in pc2.read_points(cloud_msg, skip_nans=True):
            # TODO: (3) PointCloud Data로부터 Distance, Angle 값 계산
            # print('point',point)
            # LiDAR의 PointCloud Data로부터 Distance와 Angle 값을 계산하는 영역입니다.
            # 각 Point의 XYZ 값을 활용하여 Distance와 Yaw Angle을 계산합니다.
            # Input : point (X, Y, Z, Intensity)

            dist = np.sqrt(point[0] ** 2 + point[1] ** 2 + point[2] ** 2)
            angle = np.arctan2(point[1], point[0])

            if point[0] > 0 and 1.50 > point[2] > -1.25 and dist < 50:
                point_list.append((point[0], point[1], point[2], point[3], dist, angle))

        point_np = np.array(point_list, np.float32)

        return point_np
    
    # def do_ransac_plane_normal_segmentation(self, point_cloud, input_max_distance):
    #     segmenter = point_cloud.make_segmenter_normals(ksearch=50)
    #     segmenter.set_optimize_coefficients(True)
    #     segmenter.set_model_type(pcl.SACMODEL_NORMAL_PLANE)  #pcl_sac_model_plane
    #     segmenter.set_normal_distance_weight(0.1)
    #     segmenter.set_method_type(pcl.SAC_RANSAC) #pcl_sac_ransac
    #     segmenter.set_max_iterations(1000)
    #     segmenter.set_distance_threshold(input_max_distance) #0.03)  #max_distance
    #     indices, coefficients = segmenter.segment()

    #     inliers = point_cloud.extract(indices, negative=False)
    #     outliers = point_cloud.extract(indices, negative=True)

    #     return indices, inliers, outliers
    
    def do_voxel_grid_downsampling(self, pcl_data,leaf_size):
        vox = pcl_data.make_voxel_grid_filter()
        vox.set_leaf_size(leaf_size, leaf_size, leaf_size) # The bigger the leaf size the less information retained
        return  vox.filter()


if __name__ == '__main__':
    try:
        lidar_3D = lidar()

    except rospy.ROSInterruptException:
        pass
