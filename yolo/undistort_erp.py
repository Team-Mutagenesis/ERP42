#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import yaml
import time
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Undistort(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.img_callback)

        self.img_pub = rospy.Publisher('image_topic', Image, queue_size=1)
        #brio
        #mtx = [[639.80763,   0.     , 325.40373],
         #       [0.     , 643.00213, 235.77328],
          #      [0.     ,   0.     ,   1.     ]]

        #dist = [0.063096, -0.134543, -0.003215, -0.000659, 0.000000] 

	#920 r
        mtx = [[525.394994, 0. , 309.485945],
                [0., 523.886046, 231.023575],
                [0.     ,   0.     ,   1.     ]]

        dist = [0.093751, -0.169769, -0.000626, -0.006120, 0.000000] 
	
        self.mtx = np.array(mtx)
        self.dist = np.array(dist)
        self.frame = None

        print("ready...")
    
    def img_callback(self, data):
        st = time.time()
        img = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        # undistort image
        h,  w = img.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(self.mtx,self.dist,(w,h),1,(w,h))
        dst = cv2.undistort(img, self.mtx, self.dist, None, newcameramtx)
        x,y,w,h = roi
        img = dst[y:y+h, x:x+w]
        img = cv2.resize(img, (640, 640))
        self.frame = img

        x_coord, y_coord =  160, 380 # 빨간 점의 x 좌표480, 380  # 빨간 점의 y 좌표
        red_color = (0, 0, 255)  # 빨간색 (BGR 포맷)
        # pts1 = np.float32([[160, 380],[460, 380],[0, 500],[640, 500]])
        # pts1 = np.float32([[170, 400],[435, 400],[38, 490],[560, 490]])
        # # 점 그리기
        # cv2.circle(img, (x_coord, y_coord), 5, red_color, -1)  # -1은 원 안을 채우라는 의미

        cv2.imshow('Image with Red Dot', img)
        cv2.waitKey(1) 


        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        img_msg.header.stamp = data.header.stamp
        self.img_pub.publish(img_msg)
        #print(time.time()-st)


if __name__=='__main__':
    rospy.init_node("undistort")
    detector = Undistort()

    #imshow 확인용
    # while not rospy.is_shutdown():
    #     frame = detector.frame

    #     if frame is not None:
    #         cv2.imshow("Undistorted Image", frame)
    #         cv2.waitKey(1)
    #-------------------------------------------
    rospy.spin()
    #cv2.destroyAllWindows()

