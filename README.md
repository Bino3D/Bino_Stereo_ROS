# Bino_Stereo_ROS
## 1.使用前准备
#### 1.1 下载Bino_Stereo_SDK
下载及配置方法路径:

#### 1.2 安装ROS

* 我们使用的ROS版本为kinetic   [ROS安装教程](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* 安装时请选择完全安装，即使用命令

```
$sudo apt-get install ros-kinetic-desktop-full
```

## 2.编译Bino_Stereo_ROS包
* 将binocamera的ros驱动包放入用户自己的ros工作空间catkin_ws中，使用catkin_make进行编译

```
$ roscd catkin_ws
$ cd src
$ git clone <http:://>
$ cd ..
$ catkin_make
```
* 若无catkin_ws,[参照此链接建立catkin_ws](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

## 3.使用Bino_Stereo_ROS包
#### 3.1 修改launch文件中相机设备地址
* 使用ll /dev/video*命令查看本机共有几个video设备，然后插入binocamera，再使用一次ll /dev/video*命令，新增的设备号即为binocamera设备号
* 将Bino_Stereo_ROS/launch/bino_stereo.launch中的video_device参数的值修改为本机设备号

```
$ ll /dev/video*
插入摄像头
$ ll /dev/video*
$ roscd catkin_ws
$ vim src/bino_stereo_ros/launch/bino_stereo.launch
    <param name="video_device" value="/dev/video1"/> 
改为<param name="video_device" value="/dev/<video_yours>"/>
```
#### 3.2 运行Bino_Stereo_ROS获取图像
###### 1.启动相机
```
$roslaunch bino_stereo_ros bino_stereo.launch
```
###### 2.观看相机图像
```
$roslaunch bino_stereo_ros rviz_camera.launch
```

#### 3.3 运行Bino_Stereo_ROS获取IMU数据
若未安装imu-filter-madgwick，首先执行：
```
$sudo apt-get install ros-kinetic-imu-filter-madgwick
```
###### 1.启动相机
```
$roslaunch bino_stereo_ros bino_stereo.launch
```
###### 2.IMU姿态解算
```
$roslaunch bino_stereo_ros imu.launch
```
###### 3.观看IMU姿态
```
$roslaunch bino_stereo_ros rviz_imu.launch
```
## 4.API说明

#### 4.1 Published Topics
###### /bino_camera/left/image_raw([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
    发布BinoStereo左目原始图像  		
###### /bino_camera/right/image_raw([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
    发布BinoStereo右目原始图像  		
###### /bino_camera/left/image_rect([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
    发布BinoStereo左目校正后的图像  		
###### /bino_camera/right/image_rect([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
    发布BinoStereo右目校正后图像  		
###### /bino_camera/left/camera_info([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))
    发布BinoStereo左目摄像头校正参数  		
###### /bino_camera/right/camera_info([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))
    发布BinoStereo右目摄像头校正参数  
###### /bino_camera/ImuData([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html))
    发布BinoStereo的IMU数据

#### 4.2 Parameters
###### ~video_device(std::string,default:/dev/video0)
    相机设备号
###### ~framerate(int,default:60)
    相机帧率
###### ~camera_left_frame_id(std::string,default:bino_left_frame)
    相机左目坐标系名称
###### ~camera_right_frame_id(std::string,default:bino_right_frame)
    相机右目坐标系名称
###### ~camera_imu_frame_id(std::string,default:bino_imu_frame)
    相机IMU坐标系名称
###### ~intrinsuc_file(std::string)
    相机标定内参yml文件绝对路径
###### ~extrinsic_file(std::string)
    相机标定外参yml文件绝对路径




