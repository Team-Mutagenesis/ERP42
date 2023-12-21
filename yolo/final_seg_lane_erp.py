#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import yaml
import time
import numpy as np
import torch
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import torchvision.transforms as T
from cv_bridge import CvBridge, CvBridgeError


class seg_lane():
    def __init__(self):
        self.bridge = CvBridge()
        self.frame = None
        #yolo
        self.task = 'segment'
        self.weights = './weights/segment_best.pt' #'TensorRT_models/segment_640/segment_best_hs.engine'
        self.device = 0
        self.half = False
        self.img_size = 640
        self.conf_thres = 0.6
        self.iou_thres = 0.7
        self.classes = [0,1,2,3,4,5] # only lane
        self.R = np.array([[0, 1], [-1, 0]])
        self.bridge = CvBridge()

        self.model = YOLO(self.weights, self.task)
        self.img_topic = "image_topic"

        print("ready...")

        #Sub
        self.img_sub = rospy.Subscriber(self.img_topic, Image, self.img_callback)

        #Pub
        self.result_pub = rospy.Publisher('seg_result', String, queue_size=1)

    def perspective_transform(self):
        pts1 = np.float32([[170, 400],[435, 400],[38, 490],[560, 490]])
        # pts1 = np.float32([[160, 380],[460, 380],[0, 500],[640, 500]]) 
        pts2 = np.float32([[852,524],[1084,524],[852,599],[1084,599]])
        
        M = cv2.getPerspectiveTransform(pts1, pts2)

        return M    

    def img_callback(self, msg):
        #s = time.time()
        try:
        # ROS 이미지 메시지를 OpenCV 이미지로 변환
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            results = self.model.predict(self.frame, half=self.half, device=self.device, save=False, imgsz=self.img_size, conf=self.conf_thres, iou=self.iou_thres, classes=self.classes, verbose=False, retina_masks=False)
            
            lane_point = []
            result = []

            clss = results[0].boxes.cls.to('cpu').numpy()
            clss = clss.astype('uint8')
        
            M = self.perspective_transform()

            if results[0].masks != None:
                for i in range(len(results[0].masks)):
                    tmp = []

                    tensor = results[0].masks[i].data
                    tensor = tensor.squeeze()
                    mask = torch.unsqueeze(tensor, dim=2).to('cpu').numpy()
                    
                    if np.all(mask == 0): continue 

                    y_values = np.where(mask == 1)[0]
                    x_values = np.where(mask == 1)[1]
                    
                    coefficients = np.polyfit(y_values, x_values, deg=2)
                    poly = np.poly1d(coefficients)

                    y_min = np.min(y_values)
                    y_max = np.max(y_values)
                    num_points = 10
                    ratios = np.linspace(0, 1, num=num_points+1)**3

                    sample_y_values = y_min + ratios * (y_max - y_min)
                    sample_x_values = poly(sample_y_values)

                    t_sample_x = (sample_x_values / mask.shape[1]) * 640
                    t_sample_y = (sample_y_values / mask.shape[0]) * 640

                    #차선 imshow 부분------------------------------------------------------
                    for x_point, y_point in zip(t_sample_x, t_sample_y):
                        lane_point.append([x_point, y_point])

                        if len(lane_point)% 11 ==0 and len(lane_point) > 1:
                            num = len(lane_point)//11
                            if len(lane_point) ==11:
                                
                                cv2.polylines(self.frame, [np.array(lane_point[:12], dtype=np.int32)], isClosed=False, color=(0, 255 ,0), thickness=5)
                                        
                            else:
                                cv2.polylines(self.frame, [np.array(lane_point[11*(num-1)+1:(num*10)+1], dtype=np.int32)], isClosed=False, color=(0, 255 ,0), thickness=5)
                    
                    #------------------------------------------------------------------        

                    for x, y in zip(t_sample_x, t_sample_y):
                        point = np.array([[[x,y]]], dtype=np.float32)
                        t_point = cv2.perspectiveTransform(point, M)
                        x = (t_point[0][0][0] - 960) * 0.02
                        y = ((t_point[0][0][1] * -1) + 639) * 0.15 #0.067
                        m = np.dot(self.R, np.array([x,y]))
                        tmp.append([m[0], m[1]])
                    result.append([clss[i], tmp])
                
                result_str = str(result)
                self.result_pub.publish(result_str)
                #print(f"process time: {time.time()-st}")
            else:
                result_str = str(result)
                self.result_pub.publish(result_str)
                #print(f"process time: {time.time()-st}")
            
            #cv2.imshow("Results", results[0].plot())
            #cv2.waitKey(1)

        
        except CvBridgeError as e:
            print(e)

        


if __name__ == '__main__':
    rospy.init_node("seg_lane")
    detector = seg_lane()
    rospy.spin()
    # rate = rospy.Rate(30)
cv2.destroyAllWindows()
