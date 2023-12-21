#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import atexit
import numpy as np
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from detection_msgs.msg import BoundingBox, BoundingBoxes, Label
from std_msgs.msg import Int32
from sensor_msgs.msg import PointCloud2, CompressedImage, Image
import sensor_msgs.point_cloud2 as pc2
from numpy.linalg import inv
import math

parameters_cam = {
    # "WIDTH": 670, # image width
    # "HEIGHT": 480, # image height
    # "FOV_W": 67,
    # "FOV_H": 55

    "WIDTH": 640, # image width
    "HEIGHT": 480, # image height
    "FOV_W": 90,
    "FOV_H": 48
}

def getTransformMat():    
    Tr_lidar_to_cam = inv(np.array([
                                    0.0322654215711255,	-0.00982536527728900,	0.999431040526561, -0.5014744382031456,
                                    -0.999473096407265,	-0.00385044185254919,	0.0322289257595279, -0.062240195230826,
                                0.00353159013909823,	-0.999944316597033,	-0.00994442438663209, 0.388655283754717,
                                0,	0,	0, 1]).reshape(4,4))  

    return Tr_lidar_to_cam


def Transinv():
    Tr_cam_to_lidar = inv(getTransformMat())

    return Tr_cam_to_lidar

def getCameraMat(params_cam):
    # Camera Intrinsic Parameters
    # focalLength_w = params_cam["WIDTH"]/(2*np.tan(np.deg2rad(params_cam["FOV_W"]/2)))
    # focalLength_h = params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV_H"]/2)))
    # principalX = params_cam["WIDTH"]/2
    # principalY = params_cam["HEIGHT"]/2
    # CameraMat = np.array([focalLength_w,0.,principalX,
    #                       0,focalLength_h,principalY,
    #                       0,0,1]).reshape(3,3)

    CameraMat = np.array([525.394994, 0. , 309.485945,
                        0., 523.886046, 231.023575,
                        0.     ,   0.     ,   1.]).reshape(3,3)
    return CameraMat

def getInverseCameraMat(params_cam):
    # Camera Intrinsic Parameters
    # focalLength_w = params_cam["WIDTH"] / (2 * np.tan(np.deg2rad(params_cam["FOV_W"] / 2)))
    # focalLength_h = params_cam["HEIGHT"] / (2 * np.tan(np.deg2rad(params_cam["FOV_H"] / 2)))
    # principalX = params_cam["WIDTH"] / 2
    # principalY = params_cam["HEIGHT"] / 2
    # CameraMat = np.array([focalLength_w, 0., principalX,
    #                       0, focalLength_h, principalY,
    #                       0, 0, 1]).reshape(3, 3)
    
    CameraMat = np.array([525.394994, 0. , 309.485945,
                            0., 523.886046, 231.023575,
                            0.     ,   0.     ,   1.]).reshape(3,3)

    # Calculate the inverse of the camera matrix
    InvCameraMat = inv(CameraMat)
    return InvCameraMat

class lidar_cam_cal_Detector():
    def __init__(self, params_cam):

        self.bridge = CvBridge()
        self.class_names = {
            0: 'construction',
            1: 'dump',
            2: 'crosswalk',
            3: 'cone',
            4: 'person',
        }

        self.task = 'detect'
        self.weights = '/home/ha/real/src/rs/src/weights/922_tryout2.pt'
        self.device = 0
        self.half = False
        self.img_size = 640
        self.conf_thres = 0.7
        self.iou_thres = 0.7
        self.offset = 50
        
        self.model = YOLO(self.weights)#, self.task)
        
        
        self.process_time_old = 0.0
        self.process_times = []
        self.xy_i_received = []
        atexit.register(self.print_avg_process_time)

        # ========================lidar init===================================

        self.pc_np = None
        self.xy_i = None
        self.img = None
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]
        self.TransformMat = getTransformMat()
        self.InvTransforMat = Transinv()
        self.CameraMat = getCameraMat(params_cam)
        self.CameraInvMat = getInverseCameraMat(params_cam)

        self.img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)
        self.cluster_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.scan_callback)
        self.BoundingBox_pub = rospy.Publisher("/Bounding_box", BoundingBox, queue_size=5)
        self.pred_pub = rospy.Publisher("/Bounding_boxs", BoundingBoxes, queue_size=5)
        self.point_cloud_cone_pub = rospy.Publisher("/points_inside_box_cone_1", PointCloud2, queue_size=1)
        print("ready...")

    def print_avg_process_time(self):
        avg_process_time = sum(self.process_times) / len(self.process_times)
        #print(f"Average model process time: {avg_process_time} ms")

    def transformLiDARToCamera(self, pc_lidar):
        cam_temp = self.TransformMat.dot(pc_lidar)
        cam_temp = np.delete(cam_temp, 3, axis=0)
        return cam_temp
    
    def transformCameraToLiDAR(self, pc_camera):
        # lidar_temp = self.InvTransforMat.dot(pc_camera)
        lidar_temp = self.InvTransforMat.dot(pc_camera).T
        return lidar_temp
    
    
    def transformCameraToImage(self, pc_camera):
        cam_temp = self.CameraMat.dot(pc_camera)
        cam_temp = np.delete(cam_temp,np.where(cam_temp[2,:]<0),axis=1)
        cam_temp[0,:] /= cam_temp[2,:]
        cam_temp[1,:] /= cam_temp[2,:]
        cam_temp = np.delete(cam_temp,np.where(cam_temp[0,:]>self.width),axis=1)
        cam_temp = np.delete(cam_temp,np.where(cam_temp[1,:]>self.height),axis=1)
        return cam_temp

    def transformImageToCamera(self, pc_image):
        # 이미지 좌표를 카메라 좌표로 변환
        pc_camera = np.copy(pc_image)
        pc_camera[0, :] /= pc_camera[2, :]
        pc_camera[1, :] /= pc_camera[2, :]

        # 이미지 바깥에 있는 점 제거
        valid_indices = np.where(
            (pc_camera[0, :] >= 0) & (pc_camera[0, :] < self.width) &
            (pc_camera[1, :] >= 0) & (pc_camera[1, :] < self.height)
        )

        pc_camera = pc_camera[:, valid_indices]

        return pc_camera


    def draw_pts_img(self, img, xi, yi, zi):
        point_np = img
        zi = np.clip(zi, 0, 6)  
        z_max = np.max(zi)
        zi_float = zi.astype(float)
        zi_float /= z_max
        
        for ctr, z in zip(zip(xi, yi), zi_float):
            x, y = ctr
            point_np = cv2.circle(point_np, (int(x), int(y)), 2, (0, int(255 * z), int(255 * (1 - z))), -1)
        return point_np
      


    def img_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        results = self.model(img)
        self.alpha = 0.5
        self.process_time = results[0].speed['preprocess']+results[0].speed['inference']+results[0].speed['postprocess']
        self.process_times.append(self.process_time)
        self.process_time_old = self.process_time
        
        xyxy = results[0].boxes.xyxy.to('cpu').numpy()
        clss = results[0].boxes.cls
        conf = results[0].boxes.conf

        bounding_boxes_msg = BoundingBoxes()
        bounding_boxes_msg.header = data.header
        bounding_boxes_msg.image_header = data.header


        bounding_x_y_class = []

        if len(xyxy) != 0:
            for box, label, confidence in zip(xyxy, clss, conf):
                if confidence < 0.7:
                    continue

                bounding_box = BoundingBox()
                x1, y1, x2, y2 = box

                bounding_box.Class = self.class_names[label.item()]
                bounding_box.probability = confidence 
                bounding_box.xmin = int(x1)
                bounding_box.ymin = int(y1)
                bounding_box.xmax = int(x2)
                bounding_box.ymax = int(y2)

                square = (bounding_box.xmax-bounding_box.xmin)*((bounding_box.ymax-bounding_box.ymin))
                print(square)

                if self.BoundingBox_pub is not None:
                    self.BoundingBox_pub.publish(bounding_box)
                x1 = int(x1)
                y1 = int(y1)
                x2 = int(x2)
                y2 = int(y2)
                
                bounding_x_y_class.append((x1, y1, x2, y2, bounding_box.Class))

            
            selected_points_inside_box = []
            print(len(bounding_x_y_class))
            for i in bounding_x_y_class:
                std_z = []

                x1, y1, x2, y2, c = i
                for x, y, z in zip(self.xy_i[0, :], self.xy_i[1, :], self.xy_i[2, :]):
                    if (x1) <= x <= (x2) and (y1) <= y <= (y2):
                        std_z.append(z.item())

                if std_z:
                    std_z = sum(std_z) / len(std_z)
                    for x, y, z in zip(self.xy_i[0, :], self.xy_i[1, :], self.xy_i[2, :]):
                        if (x1) <= x <= (x2) and (y1) <= y <= (y2) and z < std_z:
                            selected_points_inside_box.append((x, y, z, c))

                else: continue
                    

            if selected_points_inside_box:  # selected_points_inside_box가 비어 있지 않은지 확인

                points_inside_box_lidar_cone = []
            
                for image_x, image_y, depth, c in selected_points_inside_box:
                    image_x *= depth  # x 좌표 역정규화
                    image_y *= depth  # y 좌표 역정규화
                    camera_coordinates = np.array([image_x, image_y, depth])
                    camera_coordinates = camera_coordinates.astype(np.float32)
                    camera_coordinates = self.CameraInvMat.dot(camera_coordinates)
                    camera_coordinates = np.array([camera_coordinates[0], camera_coordinates[1], camera_coordinates[2], 1]) 
                    lidar_coordinate = self.transformCameraToLiDAR(camera_coordinates)
                    point_lidar = lidar_coordinate.astype(np.float32)
                    # print(point_lidar[0],point_lidar[1])
                    if c == "cone":
                        points_inside_box_lidar_cone.append((point_lidar[0], point_lidar[1], point_lidar[2]))




                # selected_points_inside_box = np.concatenate(selected_points_inside_box, axis=1)
                # points_inside_box_lidar = []

                # for image_x, image_y, depth in selected_points_inside_box.T:
                #     image_coordinates = np.array([image_x, image_y, depth], dtype=np.float32)  # 3D 좌표
                #     camera_coordinates = np.dot(self.CameraInvMat, image_coordinates)
                #     camera_coordinates = np.append(camera_coordinates, 1.0)  # Homogeneous 좌표로 변환
                #     lidar_coordinate = self.transformCameraToLiDAR(camera_coordinates)
                #     # print(lidar_coordinate[3])
                #     points_inside_box_lidar.append(tuple(lidar_coordinate))

                # points_inside_box_lidar = np.array(points_inside_box_lidar, dtype=np.float32)
                # points_inside_box_lidar = points_inside_box_lidar[:, :3]


                pc_data_cone = pc2.create_cloud_xyz32(self.lidar_msg.header, points_inside_box_lidar_cone)

                if pc_data_cone:
                    self.point_cloud_cone_pub.publish(pc_data_cone)

        else:
            bounding_box = BoundingBox()
            bounding_box.Class = "NOT"
            self.BoundingBox_pub.publish(bounding_box)
       

    def scan_callback(self, msg):
        self.lidar_msg = msg
        point_list = []        
        for point in pc2.read_points(msg, skip_nans=True):      
            point_list.append((point[0], point[1], point[2]))
            # print(point)
        self.pc_np = np.array(point_list, np.float32)    

        xyz_p = self.pc_np
        xyz_p = np.insert(xyz_p,3,1,axis=1).T
    
        # ==============================filter=====================================
        xyz_p = np.delete(xyz_p,np.where(xyz_p[0,:]<-1),axis=1)
        xyz_p = np.delete(xyz_p,np.where(xyz_p[0,:]>10),axis=1) # 2.5m Front 
        xyz_p = np.delete(xyz_p,np.where(xyz_p[2,:]<-0.8),axis=1) #Ground Filter
        # ==============================filter=====================================

        xyz_c = self.transformLiDARToCamera(xyz_p)
        print("camera_shape",xyz_c.shape)
        self.xy_i = self.transformCameraToImage(xyz_c)
        print("image_shape",self.xy_i.shape)
        


if __name__ == '__main__':
    rospy.init_node("lidar_cam_cal_Detector")
    detector = lidar_cam_cal_Detector(parameters_cam)
    
    rospy.spin()
