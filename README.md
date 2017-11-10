# Bino_Stereo_ROS
## 1.使用前准备
#### 1.1 Linux
* [我们测试使用的linux版本为ubuntu16.04](https://www.ubuntu.com/download/desktop)

#### 1.2 安装build工具
```
$ sudo apt-get install build-essential cmake git
```
#### 1.3 安装OpenCV依赖
```
$ sudo apt-get install pkg-config libgtk2.0-dev
```
#### 1.4 安装摄像头驱动依赖
```
$ sudo apt-get install libssl-dev libv4l-dev v4l-utils
```
#### 1.5 安装OpenCV
##### 快速安装
```
$ git clone https://github.com/opencv/opencv.git
$ cd opencv/
$ git checkout tags/3.2.0
$ mkdir build
$ cd build/
$ cmake ..
$ make
```
##### [OpenCV详细配置请见OpenCV官方Git](https://github.com/opencv/opencv)

#### 1.6 ROS

* 我们使用的ROS版本为kinetic   [ROS安装教程](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* 安装时请选择完全安装，即使用命令

```
$sudo apt-get install ros-kinetic-desktop-full
```

## 2.编译Bino_Stereo_ROS包
* 将binocamera的ros驱动包放入用户自己的ros工作空间catkin_ws中，使用catkin_make进行编译

```
$ cd <your catkin_ws path>
$ cd src
$ git clone https://github.com/Bino3D/Bino_Stereo_ROS
$ cd ..
$ catkin_make
$ echo "source <your catkin_ws path>/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc 
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
#### 3.2 修改launch文件中标定参数地址
* 进入Bino_Stereo_ROS/params目录，执行pwd命令，得到本地相机标定参数的绝对路径
* 将Bino_Stereo_ROS/launch/bino_stereo.launch中的intrinsuc_file参数的值修改为本机相机标定内参的绝对路径
* 将Bino_Stereo_ROS/launch/bino_stereo.launch中的extrinsic_file参数的值修改为本机相机标定外参的绝对路径

```
$roscd catkin_ws
$cd src/Bino_Stereo_ROS/params
$pwd
/home/li/catkin_ws/src/Bino_Stereo_ROS/params
$vim src/bino_stereo_ros/launch/bino_stereo.launch
    <param name="intrinsuc_file" type="string"  value="/home/li/catkin_ws/src/Bino_Stereo_ROS/params/intrinsics.yml"/>
改为<param name="intrinsuc_file" type="string"  value="<yours_path>/intrinsics.yml"/>
    <param name="extrinsic_file" type="string"  value="/home/li/catkin_ws/src/Bino_Stereo_ROS/params/extrinsics.yml"/> 
改为<param name="extrinsic_file" type="string"  value="<yours_path>/extrinsics.yml"/> 
```
#### 3.3 运行Bino_Stereo_ROS 
###### 启动相机
```
$roslaunch bino_stereo_ros bino_stereo.launch
```
###### 观看相机图像
```
$roslaunch bino_stereo_ros rviz_camera.launch
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

#### 4.2 Parameters
###### ~video_device(std::string,default:/dev/video0)
    相机设备号
###### ~framerate(int,default:60)
    相机帧率
###### ~camera_frame_id(std::string,default:bino_camera)
    相机坐标系名称
###### ~intrinsuc_file(std::string)
    相机标定内参yml文件绝对路径
###### ~extrinsic_file(std::string)
    相机标定外参yml文件绝对路径

#### 4.3 Required tf Transforms
    bino_camera——>base_link


