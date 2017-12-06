/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <ros/ros.h>
#include <iostream> //C++标准输入输出库 
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>
#include <std_srvs/Empty.h>
#include <cv_bridge/cv_bridge.h>   
#include <sensor_msgs/image_encodings.h>   
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Twist.h>    
#include <nav_msgs/Odometry.h>    
#include "tf/LinearMath/Matrix3x3.h"    
#include "geometry_msgs/Quaternion.h" 
#include <geometry_msgs/TwistStamped.h>  
#include <sensor_msgs/LaserScan.h>   
#include <opencv2/opencv.hpp>  
#include <BinoCamera.h>
#include <memory>

/***IMU死区***/
const float deadmin = -0.8; 
const float deadmax = 0.8;

/***IMU加速度计零偏***/
const float axerr = 0.0045; 
const float ayerr = -0.0266; 
const float azerr = -0.0100;
std::string right_frame_id;
std::string left_frame_id;
std::string imu_frame_id;
std::string intrinsuc_file;
std::string extrinsic_file;
int  framerate_;


void getCameraInfo(sensor_msgs::CameraInfoPtr&,sensor_msgs::CameraInfoPtr&);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bino_stereo");
	ros::NodeHandle node_;
	ros::NodeHandle nh("~");
    BinoCameraParameterList paraList;
	nh.param("video_device",paraList.devPath, std::string("/dev/video0"));
	nh.param("framerate", framerate_, 60);
	nh.param("camera_left_frame_id",left_frame_id, std::string("/bino_left_frame"));
	nh.param("camera_right_frame_id",right_frame_id, std::string("/bino_right_frame"));
	nh.param("camera_imu_frame_id",imu_frame_id, std::string("/bino_imu_frame"));
	nh.param<std::string>("intrinsuc_file",intrinsuc_file, "intrinsics.yml");
	nh.param<std::string>("extrinsic_file",extrinsic_file, "extrinsics.yml");
	paraList.intParameterPath = intrinsuc_file;
	paraList.extParameterPath = extrinsic_file;
	sensor_msgs::CameraInfoPtr left_info(new sensor_msgs::CameraInfo());
	sensor_msgs::CameraInfoPtr right_info(new sensor_msgs::CameraInfo());
	std::shared_ptr<BinoCamera> camera(new BinoCamera(paraList));
	camera->setImuCorrection(deadmin,deadmax,axerr,ayerr,azerr);
	//get camera info
	getCameraInfo(left_info,right_info);
	ros::Rate loop_rate(framerate_);
	ros::Publisher left_pub = node_.advertise<sensor_msgs::Image>("/bino_camera/left/image_raw", 1);//原始双目图像数据
	ros::Publisher right_pub = node_.advertise<sensor_msgs::Image>("/bino_camera/right/image_raw", 1);
	ros::Publisher left_rect_pub = node_.advertise<sensor_msgs::Image>("/bino_camera/left/image_rect", 1);//校正过的双目图像数据
	ros::Publisher right_rect_pub = node_.advertise<sensor_msgs::Image>("/bino_camera/right/image_rect", 1);
	ros::Publisher left_info_pub = node_.advertise<sensor_msgs::CameraInfo>("/bino_camera/left/camera_info", 1);//相机参数
	ros::Publisher right_info_pub = node_.advertise<sensor_msgs::CameraInfo>("/bino_camera/right/camera_info", 1);
	ros::Publisher imu_pub = node_.advertise<sensor_msgs::Imu>("/bino_camera/ImuData", 1);//IMU数据
	sensor_msgs::ImagePtr left_raw,right_raw,left_rect,right_rect;
    sensor_msgs::Imu imu_msg;
	
	cv::Mat gray_l,gray_r;
	std::vector<ImuData> imudatas;
	ImuData imudata;
	ros::Time image_ros_time;
	bool image_useimu = false;
	int32_t timestamp_cam = 0,timestamp_cam_pre = 0,cam_time_beg = -1;
	double timestamp_ros = 0,timestamp_ros_pre = 0,ros_time_beg = 0;
	double timestamp_offset = 0;
	double offset = 0;
	while (node_.ok())
	{
		int image_raw_SubNumber = left_pub.getNumSubscribers();
		int image_rect_SubNumber = left_rect_pub.getNumSubscribers();
		int image_info_SubNumber = left_info_pub.getNumSubscribers();
		int imu_SubNumber = imu_pub.getNumSubscribers();
		ros::Time time;
		if(imu_SubNumber > 0 && image_useimu){
			time = image_ros_time;
		}else{
			time = ros::Time::now(); 
		}
		if(image_raw_SubNumber > 0){
			//get the stereo raw image
			camera->getOrgImage(gray_l, gray_r);

			//mat->sensor_msgs
			left_raw = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray_l).toImageMsg();
			right_raw = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray_r).toImageMsg();

			//raw image publish
			left_raw->header.stamp = time;
			left_raw->header.frame_id=left_frame_id;
			right_raw->header.stamp = time;
			right_raw->header.frame_id=right_frame_id;
			left_pub.publish(left_raw); 
			right_pub.publish(right_raw); 
		}

		if(image_rect_SubNumber > 0){
			//get the stereo rect image
			camera->getRectImage(gray_l, gray_r);

			//mat->sersor_msgs
			left_rect = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray_l).toImageMsg();
			right_rect = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray_r).toImageMsg();

			//rect image publish
			left_rect->header.stamp = time;
			left_rect->header.frame_id=left_frame_id;
			right_rect->header.stamp = time;
			right_rect->header.frame_id=right_frame_id;
			left_rect_pub.publish(left_rect); 
			right_rect_pub.publish(right_rect); 
		}

		if(image_info_SubNumber > 0){
			//camera info publish
			left_info->header.stamp = time;
			left_info->header.frame_id=left_frame_id;
			right_info->header.stamp = time;
			right_info->header.frame_id=right_frame_id;
			left_info_pub.publish(left_info);
			right_info_pub.publish(right_info);
		}

		if(imu_SubNumber > 0){
			camera->getImuRawData(imudatas,timestamp_cam);
			timestamp_ros = ros::Time::now().toSec();
			if(cam_time_beg = -1){
				cam_time_beg = timestamp_cam;
				ros_time_beg = timestamp_ros;
			}
			if(timestamp_cam_pre > 0){
				offset = ((timestamp_cam - timestamp_cam_pre)*0.001f) - (timestamp_ros - timestamp_ros_pre);
			}
			timestamp_cam_pre = timestamp_cam;
			timestamp_ros_pre = timestamp_ros;
			int size = imudatas.size();
			double offset_each = offset/size;
			for(int i = 0; i < size; i++){
				imudata = imudatas[i];
				double offset_fix = timestamp_offset + offset_each * i;
				ros::Time imu_ros_time(ros_time_beg + (imudata.time - cam_time_beg)*0.001f - offset_fix); 
				imu_msg.header.stamp = imu_ros_time;
				imu_msg.header.frame_id = imu_frame_id;

				imu_msg.linear_acceleration.x = imudata.accel_x * 9.8;
				imu_msg.linear_acceleration.y = imudata.accel_y * 9.8;
				imu_msg.linear_acceleration.z = imudata.accel_z * 9.8;

				imu_msg.linear_acceleration_covariance[0] = 0.04;
				imu_msg.linear_acceleration_covariance[1] = 0;
				imu_msg.linear_acceleration_covariance[2] = 0;

				imu_msg.linear_acceleration_covariance[3] = 0;
				imu_msg.linear_acceleration_covariance[4] = 0.04;
				imu_msg.linear_acceleration_covariance[5] = 0;

				imu_msg.linear_acceleration_covariance[6] = 0;
				imu_msg.linear_acceleration_covariance[7] = 0;
				imu_msg.linear_acceleration_covariance[8] = 0.04;

				imu_msg.angular_velocity.x = imudata.gyro_x;
				imu_msg.angular_velocity.y = imudata.gyro_y;
				imu_msg.angular_velocity.z = imudata.gyro_z;

				imu_msg.angular_velocity_covariance[0] = 0.02;
				imu_msg.angular_velocity_covariance[1] = 0;
				imu_msg.angular_velocity_covariance[2] = 0;

				imu_msg.angular_velocity_covariance[3] = 0;
				imu_msg.angular_velocity_covariance[4] = 0.02;
				imu_msg.angular_velocity_covariance[5] = 0;

				imu_msg.angular_velocity_covariance[6] = 0;
				imu_msg.angular_velocity_covariance[7] = 0;
				imu_msg.angular_velocity_covariance[8] = 0.02;

				imu_pub.publish(imu_msg);
				
				if(i = size - 1){
					image_ros_time = imu_ros_time;
				}
			}
				timestamp_offset += offset;
				image_useimu = true;

		}else{
			image_useimu = false;
			cam_time_beg = -1;
			timestamp_cam_pre = 0;
			timestamp_offset = 0;
			offset = 0;
		}

		ros::spinOnce(); 
		loop_rate.sleep();
	}
	return true;
}


void getCameraInfo(sensor_msgs::CameraInfoPtr& left_info,sensor_msgs::CameraInfoPtr& right_info){
	cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
	cv::FileStorage fs_intrinsuc(intrinsuc_file, cv::FileStorage::READ);
	cv::FileStorage fs_extrinsic(extrinsic_file, cv::FileStorage::READ);

	left_info->height = 480;
	right_info->height = 480;
	left_info->width = 752;
	right_info->width = 752;

	left_info->roi.height = 480;
	right_info->roi.height = 480;
	left_info->roi.width = 752;
	right_info->roi.width = 752;
	left_info->distortion_model = "plumb_bob";
	right_info->distortion_model = "plumb_bob";
	left_info->binning_x=1;
	left_info->binning_y=1;
	right_info->binning_x=1;
	right_info->binning_y=1;


	fs_intrinsuc["M1"] >> K_l;
	fs_intrinsuc["M2"] >> K_r;

	fs_extrinsic["P1"] >> P_l;
	fs_extrinsic["P2"] >> P_r;

	fs_extrinsic["R1"] >> R_l;
	fs_extrinsic["R2"] >> R_r;

	fs_intrinsuc["D1"] >> D_l;
	fs_intrinsuc["D2"] >> D_r;
	left_info->height = right_info->height = 480;
	left_info->width = right_info->width = 752;

	left_info->D.resize(5);
	left_info->D[0] = D_l.at<double>(0,0);
	left_info->D[1] = D_l.at<double>(0,1);
	left_info->D[2] = D_l.at<double>(0,2);
	left_info->D[3] = D_l.at<double>(0,3);
	left_info->D[4] = D_l.at<double>(0,4);

	left_info->K[0] = K_l.at<double>(0,0);
	left_info->K[1] = K_l.at<double>(0,1);
	left_info->K[2] = K_l.at<double>(0,2);
	left_info->K[3] = K_l.at<double>(1,0);
	left_info->K[4] = K_l.at<double>(1,1);
	left_info->K[5] = K_l.at<double>(1,2);
	left_info->K[6] = K_l.at<double>(2,0);
	left_info->K[7] = K_l.at<double>(2,1);
	left_info->K[8] = K_l.at<double>(2,2);

	left_info->R[0] = R_l.at<double>(0,0);
	left_info->R[1] = R_l.at<double>(0,1);
	left_info->R[2] = R_l.at<double>(0,2);
	left_info->R[3] = R_l.at<double>(1,0);
	left_info->R[4] = R_l.at<double>(1,1);
	left_info->R[5] = R_l.at<double>(1,2);
	left_info->R[6] = R_l.at<double>(2,0);
	left_info->R[7] = R_l.at<double>(2,1);
	left_info->R[8] = R_l.at<double>(2,2);

	left_info->P[0] = P_l.at<double>(0,0);
	left_info->P[1] = P_l.at<double>(0,1);
	left_info->P[2] = P_l.at<double>(0,2);
	left_info->P[3] = P_l.at<double>(0,3);
	left_info->P[4] = P_l.at<double>(1,0);
	left_info->P[5] = P_l.at<double>(1,1);
	left_info->P[6] = P_l.at<double>(1,2);
	left_info->P[7] = P_l.at<double>(1,3);
	left_info->P[8] = P_l.at<double>(2,0);
	left_info->P[9] = P_l.at<double>(2,1);
	left_info->P[10] = P_l.at<double>(2,2);
	left_info->P[11] = P_l.at<double>(2,3);

	right_info->D.resize(5);
	right_info->D[0] = D_r.at<double>(0,0);
	right_info->D[1] = D_r.at<double>(0,1);
	right_info->D[2] = D_r.at<double>(0,2);
	right_info->D[3] = D_r.at<double>(0,3);
	right_info->D[4] = D_r.at<double>(0,4);

	right_info->K[0] = K_r.at<double>(0,0);
	right_info->K[1] = K_r.at<double>(0,1);
	right_info->K[2] = K_r.at<double>(0,2);
	right_info->K[3] = K_r.at<double>(1,0);
	right_info->K[4] = K_r.at<double>(1,1);
	right_info->K[5] = K_r.at<double>(1,2);
	right_info->K[6] = K_r.at<double>(2,0);
	right_info->K[7] = K_r.at<double>(2,1);
	right_info->K[8] = K_r.at<double>(2,2);

	right_info->R[0] = R_r.at<double>(0,0);
	right_info->R[1] = R_r.at<double>(0,1);
	right_info->R[2] = R_r.at<double>(0,2);
	right_info->R[3] = R_r.at<double>(1,0);
	right_info->R[4] = R_r.at<double>(1,1);
	right_info->R[5] = R_r.at<double>(1,2);
	right_info->R[6] = R_r.at<double>(2,0);
	right_info->R[7] = R_r.at<double>(2,1);
	right_info->R[8] = R_r.at<double>(2,2);

	right_info->P[0] = P_r.at<double>(0,0);
	right_info->P[1] = P_r.at<double>(0,1);
	right_info->P[2] = P_r.at<double>(0,2);
	right_info->P[3] = P_r.at<double>(0,3);
	right_info->P[4] = P_r.at<double>(1,0);
	right_info->P[5] = P_r.at<double>(1,1);
	right_info->P[6] = P_r.at<double>(1,2);
	right_info->P[7] = P_r.at<double>(1,3);
	right_info->P[8] = P_r.at<double>(2,0);
	right_info->P[9] = P_r.at<double>(2,1);
	right_info->P[10] = P_r.at<double>(2,2);
	right_info->P[11] = P_r.at<double>(2,3);
}
