#!/usr/bin/env python3
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
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension,Float32MultiArray

class Detector(object):
    def __init__(self):
        self.class_names = {
            #pedstrian
            0: 'construction',
            1: 'Dump',
            2: 'crosswalk',
            3: 'cone',
            4: 'person'
        }

        self.task = 'detect'
        self.weights = '/home/ha/real/src/yolo/weights/922_tryout2.pt'
        self.device = 0
        self.half = False
        self.img_size = 640
        self.conf_thres = 0.8
        self.iou_thres = 0.7
        self.square = 0

        self.img_topic = "/usb_cam/image_raw"

        self.model = YOLO(self.weights)#, self.task)
        
        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber(self.img_topic, Image, self.img_callback)
        self.cluster_sub = rospy.Subscriber("/cluster_to_img", Float32MultiArray, self.cluster_callback)
        self.BoundingBox_pub = rospy.Publisher("/Bounding_box", BoundingBox, queue_size=5)
        self.pred_pub = rospy.Publisher("/Bounding_boxs", BoundingBoxes, queue_size=5)

        self.process_time_old = 0.0
        self.process_times = []

        self.xy_i_received = []
        atexit.register(self.print_avg_process_time)
        # print("ready...")

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
        #삭제
        bounding_boxes = BoundingBoxes()
        bounding_boxes.header = data.header
        bounding_boxes.image_header = data.header

        if len(xyxy) != 0:
            for box, label, confidence in zip(xyxy, clss, conf):
                if confidence < 0.7:
                    continue
                bounding_box = BoundingBox()
                x1, y1, x2, y2 = box

                self.square = (x2-x1)*(y2-y1)
                print(self.square)
                bounding_box.Class = self.class_names[label.item()]
                bounding_box.probability = confidence 
                bounding_box.xmin = int(x1)
                bounding_box.ymin = int(y1)
                bounding_box.xmax = int(x2)
                bounding_box.ymax = int(y2)

                bounding_boxes.bounding_boxes.append(bounding_box)


                square = (bounding_box.xmax-bounding_box.xmin)*((bounding_box.ymax-bounding_box.ymin))
                print(square)

                if self.BoundingBox_pub is not None:
                    self.BoundingBox_pub.publish(bounding_box)
                    # print(bounding_box.Class)
                    #print(bounding_box)

                x1 = int(x1)
                y1 = int(y1)
                x2 = int(x2)
                y2 = int(y2)
                
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

                class_name = self.class_names[label.item()]
                label_text = f'{class_name}: {confidence.item():.2f}'

                (text_width, text_height), _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                text_x = x1
                text_y = y1 - 10 if y1 - 10 > 10 else y1 + 20

                cv2.rectangle(img, (text_x, text_y), (text_x + text_width, text_y - text_height), (0, 255, 0), cv2.FILLED)
                cv2.putText(img, label_text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

        else:
            bounding_box = BoundingBox()
            bounding_box.Class = "NOT"
            self.BoundingBox_pub.publish(bounding_box)
       

            #Publisher Bounding Box, Label
        self.pred_pub.publish(bounding_boxes)
    
        if len(self.xy_i_received) > 0:
            xi = self.xy_i_received[0]
            yi = self.xy_i_received[1]
            img_with_points = self.draw_pts_img(img, xi, yi)
            cv2.imshow('Image with Bounding Boxes and Labels', img_with_points)
        else:
            cv2.imshow('Image with Bounding Boxes and Labels', img)
        cv2.waitKey(1)
        
    def print_avg_process_time(self):
        avg_process_time = sum(self.process_times) / len(self.process_times)
        #print(f"Average model process time: {avg_process_time} ms")

    #==============================================================================================
    def draw_pts_img(self, img, xi, yi):
        point_np = img
        # print(point_np)
        for x, y in zip(xi, yi):
            ctr = (int(x), int(y))
            point_np = cv2.circle(point_np, ctr, 5, (0, 0, 255), -1)
        return point_np
    
    def cluster_callback(self, data):
        # self.xy_i_received = np.array(data.data).reshape(2, -1) 
        self.xy_i_received = np.array(data.data).reshape(-1, 3).T # Reshape the received data
        # projectionImage = draw_pts_img(Transformer.img, xy_i_received[0,:], xy_i_received[1,:])
        
if __name__ == '__main__':
    rospy.init_node("detector")
    detector = Detector()
    
    rospy.spin()