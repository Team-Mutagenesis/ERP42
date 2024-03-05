# ERP42
[무인모빌리티부문] 2023 대학생 창작 모빌리티 경진대회

<br/>
<p align="center" width="100%">
<img src=https://github.com/Team-Mutagenesis/ERP42/assets/82595288/6d1a9a84-d673-45ec-863f-3edfbaecf406 width="500" height="500"/>

<img src=https://github.com/Team-Mutagenesis/ERP42/assets/82595288/afc5b37d-898e-43ff-819d-2953737c6356 width="500" height="500" />
</p>

<br/>
<br/>

# Member
1. SON MIN KI
2. Moon Ha Neul
3. Park Hero
4. Song Dong Hyeun
5. Lee Gang Hyeun
   
<br/>

## Sensor Setting
### GPS & IMU

``roslaunch ublox_gps ublox_device.launch``

<br/>

### Velodyne 16 Channel LiDAR

``roslaunch velodyne_pointcloud VLP16_points.launch``

``python filtering.py``

<br/>

### USB Camera
This section will need to  ["ultralytics"](https://github.com/ultralytics/ultralytics)
Pip or Git clone in your workspace

``roslaunch usb_cam usb_cam-test.launch``

``roslaunch darknet_ros darknet_ros.launch``

<br/>

### ERP42        
  
``roslaunch erp_driver erp42_base.launch``     

<br/>
 
### Main

``python main.py``

<br/>
 
### Map & Path_Tracking

``python path_pub.py``

``python pure_pursuit_ERP.py``

수정 중
