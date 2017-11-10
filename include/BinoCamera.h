/*
 * StereoCamera.h
 *
 *  Created on: 2017年11月6日
 *      Author: clover
 */

#ifndef SDKLINK_SDKSRC_BINOCAMERA_H_
#define SDKLINK_SDKSRC_BINOCAMERA_H_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <functional>
#include <thread>
#include <mutex>

struct BinoCameraParameterList{
	std::string devPath = "/dev/video0";  //设备节点地址
	std::string extParameterPath = "";    //OPENCV外参标定文件
	std::string intParameterPath = "";    //OPENCV内参标定文件
};

class BinoCamera {
public:
	BinoCamera(BinoCameraParameterList paraList);
	virtual ~BinoCamera();
	void getOrgImage(cv::Mat& L, cv::Mat& R);
	void getRectImage(cv::Mat& L, cv::Mat& R);
	void getDisparity(const cv::Mat& rectLeft,	//标定后的左图像
			const cv::Mat& rectRight,				//标定后的右图像
			cv::Mat& disparateU16Mat);				//U16视差图输出
	bool getCameraParameter(cv::Mat& para);		//fx,fy,cx,cy,width,height,bf  cv::Mat(1,7, CV_64FC1)
};

#endif /* SDKLINK_SDKSRC_BINOCAMERA_H_ */
