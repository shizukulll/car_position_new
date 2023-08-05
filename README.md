# car_position
## 订阅
●订阅惯导消息:/INS/ASENSING_INS
## 发布
●车的全局位姿:/Carstate

●可视化车身;/carBody

●可视化车轮:/whole

## 对惯导消息的处理
### 角度处理
●使用全局变量“oldAzimuth”存储第一次的航向角，用于转换坐标轴为以车身起始位置朝向为x+，航向角为0度的坐标轴
●并使用此代码使航向角变为于x+的夹角，逆时针为+，顺时针为-，并且范围为(-pi,+pi)
●惯导的是顺时针为+，逆时针为-正好相反
``` Carstate.car_state.theta =  - (msgs->azimuth - oldAzimuth)*PII/180; 
        if (Carstate.car_state.theta > PII) {
           Carstate.car_state.theta -= 2 * PII;
           diff -=360;
        } else if (Carstate.car_state.theta < -PII) {
        Carstate.car_state.theta += 2 * PII;
        diff+=360;
        }
``` 
### 位置处理
●首先通过经纬度计算车身再东北天下的坐标

``` 
void GeoDetic_TO_ENU(double lat, double lon, double h, double lat0, double lon0, double h0, double enu_xyz[3])
``` 

●使用存下的第一次航向角(因为是以北为正方向，所以需要减去90度转成以东为正方向)对东北天下的车身坐标进行旋转，得到以车身初始位置为坐标系的新全局坐标

    ``` 
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3( 0,0,0));
    tf2::Quaternion q;
    q.setRPY(0, 0,  (oldAzimuth - 90) * (PII / 180));
    transform.setRotation(q);
    tf2::Vector3 state(enu_xyz[0],enu_xyz[1],0);
    Carstate.car_state.x = state.x();
    Carstate.car_state.y = state.y();
    ``` 

