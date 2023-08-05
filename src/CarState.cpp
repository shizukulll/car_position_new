#include "CarState.h"


CarState::CarState(ros::NodeHandle &nh_){
    nh = nh_;
    INSsub = nh.subscribe<common_msgs::HUAT_ASENSING>("/INS/ASENSING_INS",10,&CarState::doINSMsg, this);
    whole_pub = nh.advertise<visualization_msgs::Marker>("/whole",10);
    carPub = nh.advertise<visualization_msgs::Marker>("/carBody",10);
    INSpub =  nh.advertise<common_msgs::HUAT_Carstate>("/Carstate", 10);
}


void CarState::GeoDetic_TO_ENU(double lat, double lon, double h, double lat0, double lon0, double h0, double enu_xyz[3])
{
    // 定义一些常量，表示参考椭球体的参数
    double a, b, f, e_sq;
	 a = 6378137; // 长半轴，单位为米
	 b = 6356752.3142; // 短半轴，单位为米
	f = (a - b) / a; // 扁率
	e_sq = f * (2 - f); // 第一偏心率平方

	// 计算站点（非原点）的ECEF坐标（地心地固坐标系）
	double lamb, phi, s, N, sin_lambda, cos_lambda, sin_phi, cos_phi, x, y, z;
	lamb = lat;  // 纬度，单位为度
	phi = lon; // 经度，单位为度
	s = sin(lamb); // 纬度的正弦值
	N = a / sqrt(1 - e_sq * s * s); // 卯酉圈曲率半径

	sin_lambda = sin(lamb); // 纬度的正弦值
	cos_lambda = cos(lamb); // 纬度的余弦值
	sin_phi = sin(phi); // 经度的正弦值
	cos_phi = cos(phi); // 经度的余弦值

	x = (h + N) * cos_lambda * cos_phi; // x坐标，单位为米
	y = (h + N) * cos_lambda * sin_phi; // y坐标，单位为米
	z = (h + (1 - e_sq) * N) * sin_lambda; // z坐标，单位为米

	// 计算原点的ECEF坐标（地心地固坐标系）
	double lamb0, phi0, s0, N0, sin_lambda0, cos_lambda0, sin_phi0, cos_phi0, x0, y0, z0;
	lamb0 = lat0; // 原点的纬度，单位为度
	phi0 = lon0; // 原点的经度，单位为度
	s0 = sin(lamb0); // 原点纬度的正弦值
	N0 = a / sqrt(1 - e_sq * s0 * s0); // 原点卯酉圈曲率半径

	sin_lambda0 = sin(lamb0); // 原点纬度的正弦值
	cos_lambda0 = cos(lamb0); // 原点纬度的余弦值
	sin_phi0 = sin(phi0); // 原点经度的正弦值
	cos_phi0 = cos(phi0); // 原点经度的余弦值

	x0 = (h0 + N0) * cos_lambda0 * cos_phi0; // 原点x坐标，单位为米
	y0 = (h0 + N0) * cos_lambda0 * sin_phi0; // 原点y坐标，单位为米
	z0 = (h0 + (1 - e_sq) * N0) * sin_lambda0; // 原点z坐标，单位为米

	// 计算站点相对于原点的ECEF坐标差
	double xd, yd, zd, t;
	xd = x - x0;
	yd = y - y0;
	zd = z - z0;

	// 计算站点的ENU坐标（本地东北天坐标系）
	t = -cos_phi0 * xd - sin_phi0 * yd;

	enu_xyz[0] = -sin_phi0 * xd + cos_phi0 * yd; // 东方向坐标，单位为米
	enu_xyz[1] = t * sin_lambda0 + cos_lambda0 * zd; // 北方向坐标，单位为米
	enu_xyz[2] = cos_lambda0 * cos_phi0 * xd + cos_lambda0 * sin_phi0 * yd + sin_lambda0 * zd; // 天空方向坐标，单位为米

    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3( 0,0,0));
    tf2::Quaternion q;
    q.setRPY(0, 0,  (oldAzimuth - 90) * (PII / 180));
    transform.setRotation(q);
    tf2::Vector3 state(enu_xyz[0],enu_xyz[1],0);
    state = transform * state;
    // Carstate.car_state.theta = - std::atan2(state.y(),state.x());

    // 更新车辆状态的x和y坐标，并打印出来
    Carstate.car_state.x = state.x();
    Carstate.car_state.y = state.y();
    // std::cout << "state x= " << Carstate.car_state.x << std::endl;
    // std::cout << "state y= " << Carstate.car_state.y << std::endl;
    // std::cout << "state yaw= " << Carstate.car_state.theta << std::endl;
    saveState(Carstate.car_state.x,Carstate.car_state.y,Carstate.car_state.theta);

    // 发布车辆状态的消息，并设置接收标志位为真
    INSpub.publish(Carstate);
    isFirstMsgReceived = true;
}


 void CarState::doINSMsg(const common_msgs::HUAT_ASENSING::ConstPtr& msgs){
     ROS_DEBUG("进入回调函数进行处理");    
     my_ins.east_velocity= msgs->east_velocity;
	 my_ins.north_velocity = msgs->north_velocity;
	 my_ins.ground_velocity = msgs->ground_velocity;
     my_ins.azimuth = msgs->azimuth;
    
     if (!isFirstMsgReceived) {
        Carstate.car_state.theta = 0;//第一次和第一次的夹角为0
        oldAzimuth = msgs->azimuth;//得到第一次的角度
        Carstate.V=sqrt(pow(my_ins.east_velocity,2)+pow( my_ins.north_velocity,2)+pow(my_ins.ground_velocity,2));//计算速度
        first_lat = msgs->latitude;
        first_lon = msgs->longitude;
        first_alt = msgs->altitude;
        Carstate.car_state.x = 0;
        Carstate.car_state.y = 0;
        INSpub.publish(Carstate);
        isFirstMsgReceived = true;
    } else {
        double diff = - (msgs->azimuth - oldAzimuth);
        Carstate.car_state.theta =  - (msgs->azimuth - oldAzimuth)*PII/180;
        if (Carstate.car_state.theta > PII) {
           Carstate.car_state.theta -= 2 * PII;
           diff -=360;
        } else if (Carstate.car_state.theta < -PII) {
        Carstate.car_state.theta += 2 * PII;
        diff+=360;
        }
        saveAngle(oldAzimuth,msgs->azimuth,diff);
        Carstate.V=sqrt(pow(my_ins.east_velocity,2)+pow( my_ins.north_velocity,2)+pow(my_ins.ground_velocity,2));
        GeoDetic_TO_ENU((msgs->latitude) * PII / 180, (msgs->longitude) * PII / 180, msgs->altitude,
                            first_lat * PII / 180, first_lon * PII / 180, first_alt, &enu_xyz[0]);
    }
     calc_vehicle_direction(msgs->roll,msgs->pitch,-(Carstate.car_state.theta-PII/2),dir_x,dir_y,dir_z);
     visCar();
     visWhole();
}


//p1         p2点的顺序
//p3         p4
void CarState::visCar(){
  // 创建一个线标记
visualization_msgs::Marker marker;
marker.header.frame_id = "velodyne";
marker.header.stamp = ros::Time();
marker.ns = "rectangle";
marker.id = 0;
marker.type = visualization_msgs::Marker::LINE_STRIP;
marker.action = visualization_msgs::Marker::ADD;

// 将车的位置转换为标记坐标系中的位置
tf2::Transform transform;
transform.setOrigin(tf2::Vector3( lidarToIMUDist * cos((Carstate.car_state.theta)),  lidarToIMUDist * sin((Carstate.car_state.theta)), 0)); // 偏移量
tf2::Quaternion q;
q.setRPY(0, 0, (Carstate.car_state.theta));
transform.setRotation(q);
tf2::Vector3 pos(Carstate.car_state.x, Carstate.car_state.y, 0);
//pos = transform * pos;

// 计算每个点相对于车的位置的向量
tf2::Vector3 v1(0, 0.25, 0);
tf2::Vector3 v2(0, -0.25, 0);
tf2::Vector3 v3(-1.5, 0.25, 0);
tf2::Vector3 v4(-1.5, -0.25, 0);

// 将每个向量旋转相同的角度
v1 = transform * v1;
v2 = transform * v2;
v3 = transform * v3;
v4 = transform * v4;

// 将每个向量添加到标记中
geometry_msgs::Point p1, p2, p3, p4;
p1.x = pos.x() + v1.x();
p1.y = pos.y() + v1.y();
p1.z = 0;
p2.x = pos.x() + v2.x();
p2.y = pos.y() + v2.y();
p2.z = 0;
p3.x = pos.x() + v3.x();
p3.y = pos.y() + v3.y();
p3.z = 0;
p4.x = pos.x() + v4.x();
p4.y = pos.y() + v4.y();
p4.z = 0;

// 添加四个点到线标记中
marker.points.push_back(p1);
marker.points.push_back(p2);
marker.points.push_back(p4);
marker.points.push_back(p3);
marker.points.push_back(p1);
marker.points.push_back(p2);

// 设置标记的尺寸（线的宽度）
marker.scale.x = 0.3;

// 设置标记的颜色和透明度
marker.color.a = 1; // 透明度为50%
marker.color.r = 1.0; // 红色
marker.color.g = 0.0; // 绿色
marker.color.b = 0.0; // 蓝色

// 发布标记
carPub.publish(marker);
}


// 计算车辆前进方向单位向量的函数
void CarState::calc_vehicle_direction(double roll, double pitch, double yaw, double &x, double &y, double &z) {
    // 将角度值转换为弧度
    tfRoll = DEG2RAD(roll);
    tfPitch = DEG2RAD(pitch);
    
    x = cos(tfPitch) * cos(-yaw + PII / 2.0);
    y = sin(tfRoll) * sin(tfPitch) * cos(-yaw + PII / 2.0) + cos(tfRoll) * sin(-yaw + PII / 2.0);
    z = -tfRoll * sin(-yaw + PII / 2.0)  + tfPitch * sin(-yaw + PII / 2.0);
}


//发布四个轮子
void CarState::visWhole(){
    visualization_msgs::Marker wheel_fl, wheel_fr, wheel_rl, wheel_rr;
    double angle ;
    double move;

    wheel_fl.header.frame_id = "velodyne";
    wheel_fl.header.stamp = ros::Time::now();
    wheel_fl.ns = "car_marker";
    wheel_fl.id = 1;
    wheel_fl.type = visualization_msgs::Marker::CYLINDER;
    wheel_fl.action = visualization_msgs::Marker::ADD;

    wheel_fr.header.frame_id = "velodyne";
    wheel_fr.header.stamp = ros::Time::now();
    wheel_fr.ns = "car_marker";
    wheel_fr.id = 2;
    wheel_fr.type = visualization_msgs::Marker::CYLINDER;
    wheel_fr.action = visualization_msgs::Marker::ADD;

    wheel_rl.header.frame_id = "velodyne";
    wheel_rl.header.stamp = ros::Time::now();
    wheel_rl.ns = "car_marker";
    wheel_rl.id = 3;
    wheel_rl.type = visualization_msgs::Marker::CYLINDER;
    wheel_rl.action = visualization_msgs::Marker::ADD;

     wheel_rr.header.frame_id = "velodyne";
    wheel_rr.header.stamp = ros::Time::now();
    wheel_rr.ns = "car_marker";
    wheel_rr.id = 4;
    wheel_rr.type = visualization_msgs::Marker::CYLINDER;
    wheel_rr.action = visualization_msgs::Marker::ADD;

    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3( lidarToIMUDist * cos((Carstate.car_state.theta)),  lidarToIMUDist * sin((Carstate.car_state.theta)), 0)); // 偏移量
    tf2::Quaternion qq;
    qq.setRPY(0, 0, (Carstate.car_state.theta));
    transform.setRotation(qq);

    // 计算每个轮子相对于车的位置的向量
    tf2::Vector3 v1(-0.2, 0.35, 0);
    tf2::Vector3 v2(-0.2, -0.35, 0);
    tf2::Vector3 v3(-1.2, 0.35, 0);
    tf2::Vector3 v4(-1.2, -0.35, 0);

    // 将每个向量旋转相同的角度
    v1 = transform * v1;
    v2 = transform * v2;
    v3 = transform * v3;
    v4 = transform * v4;

wheel_fl.pose.orientation.x =  dir_x;
wheel_fl.pose.orientation.y = dir_y;
wheel_fl.pose.orientation.z = dir_z;
wheel_fl.pose.orientation.w = 1;
    //std::cout<<"四元数之和为："<<q_dir.w()*q_dir.w()+q_dir.z()*q_dir.z()+q_dir.y()*q_dir.y()+q_dir.x()*q_dir.x()<<std::endl;
    wheel_fl.scale.x = 0.5;
    wheel_fl.scale.y = 0.5;
    wheel_fl.scale.z = 0.10;
    wheel_fl.color.a = 1.0;
    wheel_fl.color.r = 0.0;
    wheel_fl.color.g = 1.0;
    wheel_fl.color.b = 0.0;
    wheel_fl.pose.position.x = v1.getX() + Carstate.car_state.x;
    wheel_fl.pose.position.y = v1.getY() + Carstate.car_state.y;

    wheel_fr.pose.orientation.x = dir_x;
    wheel_fr.pose.orientation.y = dir_y;
    wheel_fr.pose.orientation.z =  dir_z;
    wheel_fr.pose.orientation.w = 1;
    wheel_fr.scale.x = 0.5;
    wheel_fr.scale.y = 0.5;
    wheel_fr.scale.z = 0.10;
    wheel_fr.color.a = 1.0;
    wheel_fr.color.r = 0.0;
    wheel_fr.color.g = 1.0;
    wheel_fr.color.b = 0.0;
    wheel_fr.pose.position.x = v2.getX() + Carstate.car_state.x;
    wheel_fr.pose.position.y = v2.getY() + Carstate.car_state.y;

    wheel_rl.pose.orientation.x = dir_x;
    wheel_rl.pose.orientation.y = dir_y;
    wheel_rl.pose.orientation.z = dir_z;
    wheel_rl.pose.orientation.w = 1;
    wheel_rl.scale.x = 0.5;
    wheel_rl.scale.y = 0.5;
    wheel_rl.scale.z = 0.10;
    wheel_rl.color.a = 1.0;
    wheel_rl.color.r = 0.0;
    wheel_rl.color.g = 1.0;
    wheel_rl.color.b = 0.0;
    wheel_rl.pose.position.x = v3.getX() + Carstate.car_state.x;
    wheel_rl.pose.position.y = v3.getY() + Carstate.car_state.y;

    wheel_rr.pose.orientation.x = dir_x;
    wheel_rr.pose.orientation.y = dir_y;
    wheel_rr.pose.orientation.z = dir_z;
    wheel_rr.pose.orientation.w = 1;
    wheel_rr.scale.x = 0.5;
    wheel_rr.scale.y = 0.5;
    wheel_rr.scale.z = 0.10;
    wheel_rr.color.a = 1.0;
    wheel_rr.color.r = 0.0;
    wheel_rr.color.g = 1.0;
    wheel_rr.color.b = 0.0;
    wheel_rr.pose.position.x = v4.getX() + Carstate.car_state.x;
    wheel_rr.pose.position.y = v4.getY() + Carstate.car_state.y;

    whole_pub.publish(wheel_fl);
    whole_pub.publish(wheel_fr);
    whole_pub.publish(wheel_rl);
    whole_pub.publish(wheel_rr);
}



void CarState::saveState(double x,double y,double yaw){
   std::stringstream ss;
       ss<<x<<"\t"<<y<<"\t"<<yaw<<std::endl;
    std::string str = ss.str();
    std::ofstream f;
    f.open("/home/ros/LIDAR_ye/src/testData/State.txt",std::ios_base::app); 
    if (f.fail()) { 
        std::cerr << "Failed to open file: " << std::strerror(errno) << std::endl; 
 
        return; 
    }
    f << str;
    f.close();
}

void CarState::saveAngle(double first,double now,double diff){
   std::stringstream ss;
       ss<<first<<"\t"<< now<<"\t"<<diff<<std::endl;
    std::string str = ss.str();
    std::ofstream f;
    f.open("/home/ros/LIDAR_ye/src/testData/Angle.txt",std::ios_base::app); 
    if (f.fail()) { 
        std::cerr << "Failed to open file: " << std::strerror(errno) << std::endl; 
 
        return; 
    }
    f << str;
    f.close();
}