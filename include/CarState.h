#pragma once

#include <ros/ros.h>   
#include <string>
#include <iostream>
#include <vector>
#include "common_msgs/HUAT_ASENSING.h"         
#include "common_msgs/HUAT_Carstate.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <fstream>

//pi，为什么不用M_PI？我不知道
 #define PII 3.14159265358979   
 //用于简单转化为弧度制
 #define DEG2RAD(deg) ((deg) * M_PI / 180.0)
 //雷达到惯导的距离
 #define  lidarToIMUDist 1.87


class CarState{
   private:

   /**
    * @brief 订阅惯导话题：/INS/ASENSING_INS
   */
   ros::Subscriber INSsub;

   /**
    * @brief 发布转换过的话题：/Carstate
   */
   ros::Publisher INSpub;

/**
 * @brief 发布轮胎可视化话题：/whole
*/
   ros::Publisher whole_pub;

   /**
    * @brief 发布车身可视化话题：//carBody
   */
   ros::Publisher carPub;

   /**
    * @brief ros节点
   */
   ros::NodeHandle nh;

   /**
    * @brief 用于存储经纬度转换为东北天坐标系的x,y,z
   */
   double  enu_xyz[3]; 

   /**
    * @brief 第一次接收的经纬度之类的数据是
   */
   double first_lat, first_lon, first_alt;

   /**
    * @brief 标志符，false代表第一次没有接收到惯导消息
   */
   bool isFirstMsgReceived = false;  

   /**
    * @brief 用于存储需要发布的消息的信息
   */
   common_msgs::HUAT_Carstate Carstate;

   /**
    * @brief 用于存储接收消息的信息
   */
   common_msgs::HUAT_ASENSING my_ins;

   /**
    * @brief 用于车轮可视化方向的四元数，方向为车前进方向
   */
   double dir_x = 0.0;
   double dir_y = 0.0;
   double dir_z = 0.0;

   /**
    * @brief 用于车轮可视化，有四个wheel被发布
   */
   void visWhole();
   
   /**
    * @brief 这个是车身的四个点，rviz中的车身就是通过这四个点连线得到的
   */
   geometry_msgs::Point p1, p2, p3, p4;

   /**
    * @brief 用于车身可视化
   */
   void visCar();

/**
 * @brief 用于存储转换为弧度制的角
*/
   double tfRoll,tfPitch;

   /**
    * @brief 判定是不是第一次接受的消息
   */
   bool theFirstTime = true;

/**
 * @brief 用于存储第一次的角度值，用于转换整个东北天坐标系为以车身起始位置为基准的坐标系
*/
   double oldAzimuth;

   public:
   /**
    * @brief 构造方法
    * @param ros节点
   */
   CarState(ros::NodeHandle &nh_);
   
   /**
    * @brief 回调方法，处理信息得到以车身起始位置为基准的坐标系，并发布
    * @param 惯导消息
   */
   void doINSMsg(const common_msgs::HUAT_ASENSING::ConstPtr& msgs);

   /**
    * @brief 使用经纬度得到东北天下的车身坐标，并转化成为以车身起始位置为基准的坐标系下的车身坐标
   */
   void GeoDetic_TO_ENU(double lat, double lon, double h, double lat0, double lon0, double h0, double enu_xyz[3]);

   /**
    * @brief 计算车前进方向，用于计算车轮方向
   */
   void calc_vehicle_direction(double roll, double pitch, double yaw, double &x, double &y, double &z);
   void saveState(double x,double y,double yaw);
   void saveAngle(double first,double now,double diff);
};

  