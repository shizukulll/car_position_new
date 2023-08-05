/**
 * 订阅/INS/ASENSING_INS消息;
 * 处理消息得到以车身起始状态为坐标轴的车辆位置和航线角等信息;
 * 车辆可视化；
 * 发布话题/Carstate传递给cone_position处理锥筒坐标
*/
#include "CarState.h"

int main(int argc ,char* argv[]){
     std::cout<<"[car_position]start"<<std::endl;
     ros::init(argc, argv, "car_positon");
     ros::NodeHandle nh;
     CarState car(nh);
    ros::Rate rate(10);
     while (ros::ok()){  
        ros::spinOnce();  
        rate.sleep();
     }
     return 0; 
}