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

/**
 *@brief 相机参数表述结构体 
 *其中包含相
 * */
struct BinoCameraParameterList{
    std::string devPath = "/dev/video0";  /**< 设备节点地址 */
    std::string extParameterPath = "";    /**< 相机标定外参文件所在地址*/
    std::string intParameterPath = "";    /**< 相机标定内参文件所在地址*/
};

/**
 *@brief imu数据结构体 
 * */
struct ImuData{
    uint32_t time;/**< 采集imu数据的时间*/

    float accel_x; /**< imu加速度计x轴数据，单位（g）*/
    float accel_y; /**< imu加速度计y轴数据，单位（g）*/
    float accel_z; /**< imu加速度计z轴数据，单位（g）*/

    float gyro_x; /**< imu陀螺仪x轴数据，单位（弧度/秒）*/
    float gyro_y; /**< imu陀螺仪y轴数据，单位（弧度/秒）*/
    float gyro_z; /**< imu陀螺仪z轴数据，单位（弧度/秒）*/
};


/**
 *@brief BinoCamera 类头文件 
 *
 * */
class BinoCamera {
public:
    /**
     *@brief BinoCamera 构造函数
     *@param paraList 传入的相机启动参数
     * */
	BinoCamera(BinoCameraParameterList paraList);
	virtual ~BinoCamera();
	/**
     *@brief 抓取一次相机数据，需要在主循环中调用
     * */
	void Grab();
    /**
     *@brief 得到双目原始图像
     *@param L 返回左相机图像
     *@param R 返回右相机图像
     * */
	void getOrgImage(cv::Mat& L, cv::Mat& R);
    /**
     @brief 得到双目畸变矫正后图像
     @param L 返回左相机矫正后图像
     @param R 返回右相机矫正后图像
     * */
	void getRectImage(cv::Mat& L, cv::Mat& R);
    /**
     @brief 得到得到视差图 
     @param rectLeft 输入左相机矫正后图像
     @param rectRight 输入右相机矫正后图像
     @param disparateU16Mat 返回uint16类型的视差图，想得到真实视差值需要转换为float型数据，转换例子：
      matU16.convertTo( matF32, CV_32F, 1.0/16);
     * */
	void getDisparity(const cv::Mat& rectLeft,
			const cv::Mat& rectRight,
			cv::Mat& disparateU16Mat);
    /**
     @brief 获取相机参数 
     @param para 返回Mat数据的类型为CV_64FC1，row = 1, col = 7，数据内容为：
     fx,fy,cx,cy,image_wideth,image_height,bf
     * */
	bool getCameraParameter(cv::Mat& para);

    /**
     @brief imu原始数据校正 
     @param deadmin 死区最小值
	 @param deadmax 死区最大值
	 @param axerr   加速度计x轴零偏校正
	 @param ayerr   加速度计y轴零偏校正
	 @param azerr   加速度计z轴零偏校正
     * */
	void setImuCorrection(float deadmin,float deadmax,float axerr,float ayerr,float azerr);

	/**
	  @brief 得到imu原始数据 
	  @param imudatas imu的原始数据
	  @param timestamp 图像帧曝光时刻的时间 
	 * */
	void getImuRawData(std::vector<ImuData> &imuDatas, uint32_t &timestamp);
	/**
	  @brief imu解算程序 
	  @param imu imu原始数据
	  @param timestamp 两次imu数据采集之间的时间戳
	  @param q 四元数
	 * */
	void ImuRect(ImuData imu,float timestamp,float *q);
private:
	cv::Mat LeftImg,RightImg;
	std::vector<ImuData> ImuDatas;
	uint32_t TimeStamp;
};

#endif /* SDKLINK_SDKSRC_BINOCAMERA_H_ */
