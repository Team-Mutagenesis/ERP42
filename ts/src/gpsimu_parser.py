#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped, QuaternionStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import tf
import turtlesim.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math


class GPSIMUParser:
    def __init__(self):
        rospy.init_node('GPS_IMU_parser', anonymous=True)

        self.gps_sub = rospy.Subscriber("/utm", PoseStamped, self.gps_utm_callback)
        self.imu_sub = rospy.Subscriber("/filter/quaternion", QuaternionStamped, self.imu_callback)
        self.fix_sub = rospy.Subscriber("/ublox_gps/fix", NavSatFix, self.navsatfix_callback)

        self.odom_pub = rospy.Publisher('/odom',Odometry, queue_size=1)
        self.rviz_odom_pub = rospy.Publisher('/rviz_odom',Odometry, queue_size=1)

        self.is_imu=False
        self.is_gps=False

        self.covariance = []

        self.odom_msg=Odometry()
        self.odom_msg.header.frame_id='/map'
        self.odom_msg.child_frame_id='/base_link'

        self.rviz_odom_msg=Odometry()
        self.rviz_odom_msg.header.frame_id='/map'
        self.rviz_odom_msg.child_frame_id = '/base_link'

        rate = rospy.Rate(30)

        offset_x = 315386.03765138006
        offset_y = 4071657.414922109

        	
        while not rospy.is_shutdown():
            print(self.is_imu, self.is_gps )
            if self.is_imu==True and self.is_gps == True:

                pose_x = self.odom_msg.pose.pose.position.x - offset_x
                pose_y = self.odom_msg.pose.pose.position.y - offset_y
                print("Rviz X, Y : %f, %f" %(pose_x, pose_y))
                print("Covaiance : %.2f, %.2f, %.2f" % (self.covariance[0], self.covariance[1], self.covariance[2]))
                # self.odom_msg.pose.pose.orientation.w
                turtlename = "base_link"
                br = tf.TransformBroadcaster()
                br.sendTransform((pose_x, pose_y, 0),
                                 tf.transformations.quaternion_from_euler(0, 0, self.vehicle_yaw),
                                 rospy.Time.now(),
                                 turtlename,
                                 "odom")
                
                br2 = tf.TransformBroadcaster()
                br2.sendTransform((0.3, 0.0, 0.2),
                        (0.0, 0.0, 0.0, 1.0),
                       rospy.Time.now(),
                        "lidar_link",
                          "base_link")

                self.odom_pub.publish(self.odom_msg)
                self.rviz_odom_pub.publish(self.rviz_odom_msg)

                rate.sleep()

    def gps_utm_callback(self, gps_msg):
        # 실제 GPS상 차량 위치
        self.odom_msg.header.stamp = rospy.get_rostime()
        self.odom_msg.pose.pose.position.x = gps_msg.pose.position.x
        self.odom_msg.pose.pose.position.y = gps_msg.pose.position.y
        self.odom_msg.pose.pose.position.z = 0.0

        # rviz상 차량 위치
        self.rviz_odom_msg.header.stamp = rospy.get_rostime()
        self.rviz_odom_msg.pose.pose.position.x = gps_msg.pose.position.x - 315386.03765138006
        self.rviz_odom_msg.pose.pose.position.y = gps_msg.pose.position.y - 4071657.414922109

        self.rviz_odom_msg.pose.pose.position.z = 0.0

        self.is_gps = True

    def imu_callback(self, data):
        self.odom_quaternion = (data.quaternion.x, data.quaternion.y, data.quaternion.z, data.quaternion.w)
        

        if data.quaternion.w == 0:
            self.odom_msg.pose.pose.orientation.x = 0.0
            self.odom_msg.pose.pose.orientation.y = 0.0
            self.odom_msg.pose.pose.orientation.z = 0.0
            self.odom_msg.pose.pose.orientation.w = 1.0
            

        else:
            self.odom_msg.pose.pose.orientation.x = data.quaternion.x
            self.odom_msg.pose.pose.orientation.y = data.quaternion.y
            self.odom_msg.pose.pose.orientation.z = data.quaternion.z
            self.odom_msg.pose.pose.orientation.w = data.quaternion.w
            _,_,self.vehicle_yaw = euler_from_quaternion(self.odom_quaternion)
   

        #print(data.quaternion.w)

        self.is_imu = True


    def navsatfix_callback(self, data):
        self.covariance = [data.position_covariance[0], data.position_covariance[4], data.position_covariance[8]]
        

if __name__ == '__main__':
    try:
        GPS_IMU_parser = GPSIMUParser()
    except rospy.ROSInterruptException:
        pass
