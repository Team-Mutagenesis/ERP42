#!/usr/bin/env python3
#-*- coding:utf-8 -*-
import numpy as np
import cv2
import rospy
import math
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
#from erp_driver.msg import erpCmdMsg, erpStatusMsg

# window size
x_size = 640
y_size = 480

def grid_score(frame, left_high, right_low):
    score = np.sum(frame[left_high[1]:right_low[1], left_high[0]:right_low[0]])
    
    return score

class ImgParser:
    def __init__(self):
        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)
        #self.ctrl_cmd_pub = rospy.Publisher('erp42_ctrl_cmd', erpCmdMsg, queue_size=1)
        #rospy.Subscriber("/erp42_status", erpStatusMsg, self.status_callback)
        # self.ctrl_cmd = erpCmdMsg()
        # self.ctrl_cmd.speed = 50
        # self.ctrl_cmd.steer = 0
        # color indexing
        self.red_color = (0, 0, 255)
        self.green_color = (0, 255, 0)
        self.blue_color = (255, 0, 0)

        #Required img
        self.img_bgr = None
        self.img_lane = None

        # Region of Interest
        self.RoI = np.float32([[160, 140], [0, 240], [480, 140], [640, 240]])
        #[[200, 190],[0,330],[460,190],[640,330]]
        # window size
        self.x_size = 640
        self.y_size = 480

        # searching point for sliding window
        self.search_point = [0, 0]
        self.vehicle_length = 2
        self.offset = 0.17
        self.lfd = 3

        self.error = 0  # 이전 오차값을 저장하기 위한 변수
        self.integral = 0  # 오차의 적분값을 저장하기 위한 변수
        self.prev_error = 0  # 이전 오차값을 저장하기 위한 변수
        self.Kp = 0  # 비례 제어 게인      (v, p, i) (10, 0.05, 0.001) (20, 0.02, 0.0005)
        self.Ki = 0.0005  # 적분 제어 게인
        self.Kd = 0
        self.p_error = 0
        self.i_error = 0
        self.d_error = 0
        # Steering limits
        self.steering_min = -2000.0  # Minimum steering value
        self.steering_max = 2000.0  # Maximum steering value

        self.frame = None

    def img_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # -------- CALIBRATION-------------
            # mtx = [[639.80763,   0.     , 325.40373],
            #     [0.     , 643.00213, 235.77328],
            #     [0.     ,   0.     ,   1.     ]]

            # dist = [0.063096, -0.134543, -0.003215, -0.000659, 0.000000] 

            # mtx = np.array(mtx)
            # dist = np.array(dist)
            # self.frame = cv2.undistort(src=self.frame, cameraMatrix=mtx, distCoeffs=dist)
            
        except Exception as e:
            print(e)

    # sliding window
    def sliding_window(self, frame, search_point):
        frame_color = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        
        # grid number
        grid_x = 20
        grid_y = 15

        # grid number for searching
        left_grid = 3
        right_grid = 3

        # search starting point of next frame
        next_search = [0,0]

        # grid size
        margin_x = frame.shape[1] / grid_x  #[1] = 640
        margin_y = frame.shape[0] / grid_y  #[0] = 480

        # histogram of white pixel to get search starting point of left, right lane
        histogram = np.sum(frame[:,:], axis=0)
        # get midpoint of image and it become boundary of left and right lane
        midpoint = int(histogram.shape[0]/2)  

        # search starting point from histogram
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint
        

        # if there are search points from previous frame,
        # initialize starting point
        if search_point[0] != 0 and search_point[0] < midpoint:
            leftx_base = search_point[0]
        if search_point[1] != 0 and midpoint < search_point[1]: 
            rightx_base = search_point[1]
        
        # from leftx_base and rightx_base, get index of searching window
        leftx_current = int(leftx_base/margin_x)
        rightx_current = int(rightx_base/margin_x)

        # list of lane point
        left_line = []
        right_line = []

        # start searching lane from bottom of image to top
        for grid in range(grid_y):
            # white pixels of each window are calculated to score
            left_score = []
            right_score = []

            # first, assume that there are no lane point
            left_point_exist = False
            right_point_exist = False


            # search white pixel of left side of image
            for left in range(left_grid):
                left_p1 = (int(margin_x * (leftx_current + left - int(left_grid / 2))), int(margin_y * (grid_y - grid - 1)))
                left_p2 = (int(margin_x * (leftx_current + left - int(left_grid / 2) + 1)), int(margin_y * (grid_y - grid)))
                left_score.append(grid_score(frame, left_p1, left_p2))      # calculate white pixel score of each window and append it to list
                cv2.rectangle(frame_color, left_p1, left_p2, self.red_color, 2)  # draw window
            
            # if there are no white pixel in left side
            if np.max(left_score) == 0:
                left_grid = 5   # set number of searching window of next frame to 5
            # if there are white pixel in left side
            else:
                leftx_current = leftx_current - int(left_grid / 2) + np.argmax(left_score)  # set searching point of upper window
                left_grid = 3                                                               # set number of searching window of next frame to 3
                left_point_exist = True


            # search white pixel of right side of image
            for right in range(right_grid):
                right_p1 = (int(margin_x * (rightx_current + right - int(right_grid / 2))), int(margin_y * (grid_y - grid - 1)))
                right_p2 = (int(margin_x * (rightx_current + right - int(right_grid / 2) + 1)), int(margin_y * (grid_y - grid)))
                right_score.append(grid_score(frame, right_p1, right_p2))         # calculate white pixel score of each window and append it to list
                cv2.rectangle(frame_color, right_p1, right_p2, self.blue_color, 2)   # draw window
            
            # if there are no white pixel in right side
            if np.max(right_score) == 0:
                right_grid = 5  # set number of searching window of next frame to 5
            # if there are white pixel in right side
            else:
                rightx_current = rightx_current - int(right_grid / 2) + np.argmax(right_score)    # set searching point of upper window
                right_grid = 3                                                                  # set number of searching window of next frame to 3
                right_point_exist = True
            
            # set left and right lane points
            left_point = (int(margin_x * leftx_current + margin_x / 2), int(margin_y * (grid_y - grid - 1) + margin_y / 2))
            right_point = (int(margin_x * rightx_current + margin_x / 2), int(margin_y * (grid_y - grid - 1) + margin_y / 2))

            # if right_point and left point are close each other, choice one point that have more points before
            if (right_point[0] - left_point[0]) < 200:
                if len(left_line) < len(right_line):
                    left_point_exist = False
                elif len(left_line) > len(right_line):
                    right_point_exist = False

            if left_point_exist == True:
                # draw left point
                cv2.line(frame_color, left_point, left_point, self.red_color, 10)
                if right_point_exist == True:
                    # left point O, right point O
                    cv2.line(frame_color, right_point, right_point, self.blue_color, 10) # draw right point
                    # if calculated left point is in range
                    if right_point[0] < x_size:
                        right_line.append(right_point)  # append it to list
                else:
                    # left point O, right point X
                    # assume that left lane is curved lane, and reinforce searching of left lane
                    left_grid = 5
                # if calculated left point is in range
                # add) right_line X pred_point
                if left_point[0] > 0:
                    pred_right_point = (left_point[0]+480,left_point[1])
                    right_line.append(pred_right_point)
                    left_line.append(left_point)    # append it to list
            else:
                if right_point_exist == True:
                    # left point X, right point O
                    # assume that right lane is curved lane, and reinforce searching of right lane
                    right_grid = 5
                    cv2.line(frame_color, right_point, right_point, self.blue_color, 10) # draw right point
                    # if calculated right point is in range
                    # add) left_line X pred_point
                    if right_point[0] < x_size:
                        pred_left_point = (right_point[0]-480,right_point[1])
                        left_line.append(pred_left_point)
                        right_line.append(right_point)  # append it to list
            
            # lane points of second window from bottom of image are saved to help next frame to set searching point
            if grid == 1:
                if left_point_exist == True:
                    next_search[0] = left_point[0]
                if right_point_exist == True:
                    next_search[1] = right_point[0]

         # 좌우 차선 점을 그립니다.
        for point in left_line:
            cv2.circle(frame_color, point, 10, (0, 0, 255), -1)  # 왼쪽 차선은 빨간색
        for point in right_line:
            cv2.circle(frame_color, point, 10, (255, 0, 0), -1)  # 오른쪽 차선은 파란색

        # 좌우 중앙선을 계산하여 그립니다.
        center_line = []
        for left_point, right_point in zip(left_line, right_line):
            print(left_point[0])
            center_x = (left_point[0] + right_point[0]) // 2
            center_y = left_point[1]  # 또는 right_point[1], 양쪽 차선 높이는 대략 같을 것입니다.

            center_line.append((center_x, center_y))
        
        for center_point in center_line:
            cv2.circle(frame_color, center_point, 10, (0, 255, 0), -1)  # 중앙선은 녹색
        


        if len(center_line) > 1:
            cv2.polylines(frame_color, [np.array(center_line)], isClosed=False, color=(0, 255, 0), thickness=5)

        current_steering = -int(math.atan2((2 * self.vehicle_length * math.sin(self.offset)), self.lfd) * 71)

        if len(center_line) > 5:
            PI = math.pi
            if center_line[-1][0] > 320:
                angle = math.atan2((center_line[-1][0]- 320), (480 - center_line[-1][1]))
                deg = (angle*180)/PI
                print(deg)
                cte = deg -current_steering
                PID_steering = self.PID_control(cte)
                # self.ctrl_cmd.steer = int(deg*40)
                # if self.ctrl_cmd.steer <-2000:
                #     self.ctrl_cmd.steer = -2000
                #self.ctrl_cmd_pub.publish(self.ctrl_cmd)

            else:
                angle = math.atan2((320-center_line[-1][0]), (480 - center_line[-1][1]))
                deg = -(angle * 180) / PI
                print(deg)
                cte = deg -current_steering
                PID_steering = self.PID_control(cte)
                # self.ctrl_cmd.steer = -int(deg * 40)
                # if self.ctrl_cmd.steer > 2000:
                #     self.ctrl_cmd.steer = 2000
                #self.ctrl_cmd_pub.publish(self.ctrl_cmd)
            #print(self.ctrl_cmd.steer)


        return frame_color, left_line, right_line, next_search
        


    def scharr_filter(self, frame):
        img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # scharr x,y filtering for gradient detection
        img_scharr_x = cv2.Scharr(img_gray, cv2.CV_64F, 1, 0)
        img_scharr_x = cv2.convertScaleAbs(img_scharr_x)
        img_scharr_y = cv2.Scharr(img_gray, cv2.CV_64F, 0, 1)
        img_scharr_y = cv2.convertScaleAbs(img_scharr_y)

        # scharr x, y = scharr x + scharr y
        img_scharr = cv2.addWeighted(img_scharr_x, 1, img_scharr_y, 1, 0)

        _, white_line = cv2.threshold(img_scharr, 150, 255, cv2.THRESH_BINARY)
        return white_line

    def top_view(self, frame):
        # set RoI
        pts = np.float32([[0, 0], [0, y_size], [x_size, 0], [x_size, y_size]])
        matrix = cv2.getPerspectiveTransform(self.RoI, pts)
        matrix_inv = cv2.getPerspectiveTransform(pts, self.RoI)
        frame = cv2.warpPerspective(frame, matrix_inv, (x_size, y_size))
        return frame

    def bird_eye_view(self, frame):
        # set ROI
        pts = np.float32([[0, 0], [0, y_size], [x_size, 0], [x_size, y_size]])
        matrix = cv2.getPerspectiveTransform(self.RoI, pts)
        matrix_inv = cv2.getPerspectiveTransform(pts, self.RoI)
        frame = cv2.warpPerspective(frame, matrix, (x_size, y_size))
        # Adding Center ROI
        x, y = x_size, y_size
        #center_ROI = np.array([[int(0*x), int(y)], [int(0*x), int(0*y)], [int(0.3*x), int(0*y)], [int(0.3*x), int(y)], [int(0.7*x), int(y)], [int(0.7*x), int(0*y)],[int(1*x), int(0*y)], [int(1*x), int(y)], [int(0*x), int(y)]])
        mask = np.zeros_like(frame)
        if len(frame.shape) > 2:
            channel_count = frame.shape[2]
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255

        #cv2.fillPoly(mask, np.int32([center_ROI]), ignore_mask_color)
        # masked_image = cv2.bitwise_and(frame, mask)

        # return masked_image
        return frame

    def yellow_and_white_filter(self, image):
        """
        Filter the image to include only yellow and white pixels
        """
        # Filter white pixels
        #white_threshold = 150 #130
        lower_white = np.array([170, 170, 160])
        upper_white = np.array([250, 250, 245])
        white_mask = cv2.inRange(image, lower_white, upper_white)
        white_image = cv2.bitwise_and(image, image, mask=white_mask)

        # Filter yellow pixels
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([10, 40, 100])
        upper_yellow = np.array([23, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        yellow_image = cv2.bitwise_and(image, image, mask=yellow_mask)

        # lower_yellow = np.array([10, 40, 100])
        # upper_yellow = np.array([23, 255, 255])
        # yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        # yellow_image = cv2.bitwise_and(image, image, mask=yellow_mask)
        
        out = cv2.bitwise_or(white_image,yellow_image)
        # Combine the two above images
        #out = cv2.addWeighted(white_image, 1., yellow_image, 1., 0.)
        
        return white_image, yellow_image, out

    def PID_control(self, cte):
        self.d_error = cte - self.p_error
        self.p_error = cte
        self.i_error += cte

        return self.Kp * self.p_error + self.Ki * self.i_error + self.Kd * self.d_error

if __name__ == '__main__':
    rospy.init_node('usb_cam_subscriber')
    
    img_parser = ImgParser()

    while not rospy.is_shutdown():
        frame = img_parser.frame
        if frame is not None:
            # step1. Resize image (Calibration skip)
            frame = cv2.resize(frame, (x_size, y_size))
            #cv2.imshow("1.resize image", frame)

            # step2. Bird-eye-view (Perspective transformation) ====
            transformed_img = img_parser.bird_eye_view(frame)
            cv2.imshow('2. bird eye view image', transformed_img)

            # step3-1. Scharr filtering ====
            scharr_filtered_img = img_parser.scharr_filter(transformed_img)
            #cv2.imshow('3-1. Scharr filtered image', scharr_filtered_img)

            # step3-2. Yellow and White color filtering ====
            white_filtered, yellow_filtered, color_filtered_img = img_parser.yellow_and_white_filter(transformed_img)
            color_filtered_img = cv2.cvtColor(color_filtered_img, cv2.COLOR_BGR2GRAY)
            _, color_filtered_img = cv2.threshold(color_filtered_img, 1, 255, cv2.THRESH_BINARY)
            #cv2.imshow('3-2. white_filtered image', white_filtered)
            #cv2.imshow('3-2. yellow_filtered image', yellow_filtered)
            #cv2.imshow('3-2. Yellow and White filtered image', color_filtered_img)

            # setp3-3. thickening detected lane
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            thickened_color_filtered_img = cv2.dilate(color_filtered_img, kernel)
            #cv2.imshow('3-3. thickened', thickened_color_filtered_img)

            # step3-4. Final Filtering
            filtered_img = cv2.bitwise_and(scharr_filtered_img, thickened_color_filtered_img)
            #cv2.imshow('3-4. Filtered image', filtered_img)

            # step3-5. Median blur
            median_img=cv2.medianBlur(thickened_color_filtered_img, 5)
            median_img=cv2.medianBlur(filtered_img, 5)

            # step4. Sliding Window
            window_searched_img, left_ptr, right_ptr, search_point = img_parser.sliding_window(median_img, img_parser.search_point)
            cv2.imshow("sliding window", window_searched_img)
            
            # step5. Reverse perspective transform
            # if not left_ptr or not right_ptr:
            #     pass
            # else :
            #     steering_value = img_parser.calculate_steering(left_ptr, right_ptr)
            #

            original = img_parser.top_view(window_searched_img)
            #cv2.imshow('5. Reverse Perspective Transform', original)
            cv2.waitKey(1)

    cv2.destroyAllWindows()
