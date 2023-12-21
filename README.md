# ERP42
[무인모빌리티부문] 2023 대학생 창작 모빌리티 경진대회

<br/>

# Commands

Default Path = ~/<your_path>

<br/>

## Sensor Setting
### GPS & IMU

``roslaunch ublox_gps ublox_device.launch``

### Velodyne 16 Channel LiDAR

``roslaunch velodyne_pointcloud VLP16_points.launch``

``python filtering.py``

### USB Camera

``roslaunch usb_cam usb_cam-test.launch``

``roslaunch darknet_ros darknet_ros.launch``

### ERP42        
  
``roslaunch erp_driver erp42_base.launch``     
 
### Main

``python main.py``
 
### Map & Path_Tracking

``python path_pub.py``

``python pure_pursuit_ERP.py``

수정 중
