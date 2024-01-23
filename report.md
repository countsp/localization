# localization 定位工作
## 2.JAN.2024
### 1.GNSS工作原理

全球卫星导航系统（GNSS），如全球定位系统（GPS），是一种基于卫星的导航系统，它可以在全球范围内提供地理位置和时间信息。GNSS的工作原理主要基于以下几个关键要素：

#### 卫星组成
- GNSS由一组轨道卫星组成，这些卫星围绕地球运行，并持续广播包含其位置和当前时间的信号。

#### 信号传输
- 卫星发出的信号是一种复杂的电磁波，其中包含卫星的身份信息、位置（星历），以及信号发送的准确时间。

#### 地面接收器
- GNSS接收器（如智能手机或专用GPS设备）接收来自多个卫星的信号，用于计算接收器的地理位置和时间。

#### 距离测量
- 接收器通过测量信号从卫星到达接收器所需的时间来计算与每个卫星的距离。由于信号以光速传播，因此通过计算传播时间与光速的乘积，可以得出距离。

#### 三角定位（Trilateration）
- 通过至少四颗卫星的信号，接收器可以确定其在三维空间中的位置。接收器到每个卫星的距离定义了以卫星为中心的一个球体，这些球体的交点即是接收器的位置。至少需要四颗卫星是因为需要额外的一颗卫星来校正时钟误差。

#### 时钟误差校正
- GNSS接收器的内置时钟与卫星的原子钟相比精度较低。通过使用额外的卫星信号，接收器可以计算并校正自身时钟的误差。

#### 大气延迟校正
- 卫星信号在穿过大气层时会减速，导致延迟。GNSS系统通过建模或使用额外的补偿信息来校正这种延迟。

综上所述，GNSS的定位原理基于从卫星到接收器的信号传播时间测量，结合三角定位技术计算出接收器的精确位置。这种系统能够在全球范围内提供定位服务，广泛应用于导航、测绘、农业、海洋和航空等众多领域。

### 2.ROS中GNSS消息类型
在 ROS（Robot Operating System）中处理 GNSS 数据时，通常使用特定的消息类型来封装这些数据。GNSS 数据可能包括位置、速度、方向、时间戳等信息。ROS 提供了几种标准消息类型来处理这类数据，以及允许用户根据需要自定义消息类型。
#### 标准消息类型
- sensor_msgs/NavSatFix:

    这是最常用的消息类型，用于发布 GNSS 定位数据。
    包含经度、纬度、海拔高度以及位置的协方差信息。
    通常用于 GPS 和其他 GNSS 接收器的位置数据。

- sensor_msgs/NavSatStatus:
    提供关于 GNSS 状态的信息，如服务状态和卫星使用情况。
    可以与 NavSatFix 消息一起使用，提供更详细的定位信息。
  
![navsat](https://img-blog.csdnimg.cn/9386c260c42b4549be742bb1381dd8dd.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBA5a2m5peg5q2i5aKD55qE5bCP6b6f,size_20,color_FFFFFF,t_70,g_se,x_16#pic_center)

> \# Navigation Satellite fix for any Global Navigation Satellite System
> 
> \# 使用 WGS 84 坐标系
>  
> \# header.stamp specifies the ROS time for this measurement (the corresponding satellite time may be reported using the sensor_msgs/TimeReference message).
> 
> \# header.stamp指定此测量的ROS时间（可以使用sensor_msgs/TimeReference消息报告相应的卫星时间）
> 
> \# header.frame_id is the frame of reference reported by the satellite receiver, usually the location of the antenna.  This is a Euclidean frame relative to the vehicle, not a reference ellipsoid.
> 
> \# header.frame_id 是卫星接收器报告的坐标系，通常是GPS天线的位置。
> \# 这是相对于车辆(中心)的欧几里得坐标变换，而不是参考椭球坐标系。
>  
> **Header header**
>  
> \# satellite fix status information    卫星定位状态信息
> 
> **NavSatStatus status**
>  
> \# Latitude [degrees]. Positive is north of equator; negative is south.
> \# 纬度[度]。 正数位于赤道以北； 负面是南方。
> 
> **float64 latitude**
>  
> \# Longitude [degrees]. Positive is east of prime meridian; negative is west.
> \# 经度[度]。 正数位于本初子午线以东； 负面是西方。
> 
> **float64 longitude**
>  
> \# Altitude [m]. Positive is above the WGS 84 ellipsoid
> \# (quiet NaN if no altitude is available).
> \# 海拔[m]。 正值高于WGS 84椭球（如果没有可用的海拔高度，则为NaN）。
> 
> **float64 altitude**
>
> \# Position covariance [m^2] defined relative to a tangential plane through the reported position. The components are East, North, and Up (ENU), in row-major order.
> 
> \# 位置协方差[m ^ 2]: 相对于切线平面的位置协方差。 组件是East，North和Up（ENU），按行优先顺序排列。
> 
> \# Beware: this coordinate system exhibits singularities at the poles.
> 
> \# 注意：此坐标系在极点处表现出奇异性。
>  
> **float64[9] position_covariance**
> 
> \# If the covariance of the fix is known, fill it in completely. If the GPS receiver provides the variance of each measurement, put them along the diagonal. If only Dilution of Precision is available, estimate an approximate covariance from that.
> 
> \# 3 - 如果已知修正的协方差，请完全填写。
> \# 2 - 如果GPS接收器提供了每次测量的方差，请将其沿对角线放置。
> \# 1 - 如果只有“精度稀释”可用，请据此估计近似协方差。
>  
> **uint8 COVARIANCE_TYPE_UNKNOWN = 0**
> 
> **uint8 COVARIANCE_TYPE_APPROXIMATED = 1**
> 
> **uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2**
> 
> **uint8 COVARIANCE_TYPE_KNOWN = 3**
>  
> **uint8 position_covariance_type**


### 3.常见的 GNSS 坐标系以及转换方式
- WGS 84（世界大地坐标系1984 LatlongALt）:
        GPS 系统使用的全球参考标准。
        定义了地球的形状和尺寸，提供了全球统一的经纬度和高度。

- 地心地固坐标系（Earth-Centered, Earth-Fixed, ECEF）:
        以地球质心为原点，固定在地球上，随地球旋转。
        通常用 X、Y、Z 坐标表示位置。

**Cartographer中WGS-ECEF转换方式**
 
> Eigen::Vector3d LatLongAltToEcef(const double latitude, const double longitude,  
>                                  const double altitude) {  
> 
>   // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates  
>   
>   constexpr double a = 6378137.;  // semi-major axis, equator to center.  
>   constexpr double f = 1. / 298.257223563;  
>   constexpr double b = a * (1. - f);  // semi-minor axis, pole to center.  
>   constexpr double a_squared = a * a;  
>   constexpr double b_squared = b * b;  
>   constexpr double e_squared = (a_squared - b_squared) / a_squared;  
>   const double sin_phi = std::sin(cartographer::common::DegToRad(latitude));  
>   const double cos_phi = std::cos(cartographer::common::DegToRad(latitude));  
>   const double sin_lambda = std::sin(cartographer::common::DegToRad(longitude));  
>   const double cos_lambda = std::cos(cartographer::common::DegToRad(longitude));  
>   const double N = a / std::sqrt(1 - e_squared * sin_phi * sin_phi);  
>   const double x = (N + altitude) * cos_phi * cos_lambda;  
>   const double y = (N + altitude) * cos_phi * sin_lambda;  
>   const double z = (b_squared / a_squared * N + altitude) * sin_phi;  
> 
>   return Eigen::Vector3d(x, y, z);  
> }

## 15.JAN.2024
### 1.ins_eskf_kitti 框架跑通
![Screenshot from 2024-01-15 16-49-27](https://github.com/countsp/localization/assets/102967883/2632c858-ae3c-48bb-8f00-90061199767c)

![Screenshot from 2024-01-15 16-55-51](https://github.com/countsp/localization/assets/102967883/ab4dd370-08d5-472a-85e8-aa37d504875e)
#### Steps:
##### Dependency
Eigen:
``` 
sudo apt-get install libeigen3-dev 
```
geographiclib :
  ```
sudo apt-get install libgeographic-dev
  ```
  
glog : 
  ```
sudo apt-get install libgoogle-glog-dev
  ```

yaml-cpp:
  ```
sudo apt-get install libyaml-cpp-dev
  ```

##### Install

1. Edit the ```include/global_definition.h``` change the PROJECT_PATH to yours.

2.   

```
mkdir ins_eskf && cd ins_eskf
mkdir src && cd src
git clone https://github.com/leo6862/ins_eskf_kitti.git
cd ..
catkin_make_isolated
```
catkin_make also viable

##### Sample datasets
Using a Kitti Dataset provided online.

You can download it from  [baidu net disk](https://pan.baidu.com/s/15V587gC7cC6YZp_250ShEQ) 提取码: wtu9.

##### Run the package

Run the launch file:
```
roslaunch ins_eskf ins_eskf.launch
```

### 2-3 gps\imu\output发布速率、封装结构和内容
gps: 10hz at /kitti/oxts/gps/fix

imu: 100hz at /kitti/oxts/imu/extract

output: 10hz at /kitti_gps_odometry

![Screenshot from 2024-01-16 08-58-42](https://github.com/countsp/localization/assets/102967883/e677783d-00b9-4fbb-b11d-fd0c857b5cb3)

### 5.任意一种INS融合方法
KF-GINS： An EKF-Based GNSS/INS Integrated Navigation System 
>https://github.com/countsp/KF-GINS

command:
> ./bin/KF-GINS ./dataset/kf-gins.yaml

![Screenshot from 2024-01-16 09-12-15](https://github.com/countsp/localization/assets/102967883/a4e7b0ff-66d3-4705-b5d8-200102b026f5)

## 23.JAN.2024
### 迭代最近点（Iterative Closest Point，ICP）算法

迭代最近点（Iterative Closest Point，ICP）算法是一种用于点云间配准的方法，它试图将两个点云之间的差距（即位姿差异）最小化。ICP 算法通过迭代方式进行，并包括以下几个主要步骤：

#### 步骤

1. **初始化**：选择一个初始估计来对齐点云，这通常是基于先验知识或简单的假设。
2. **寻找对应点**：在每次迭代中，算法对源点云中的每个点寻找最近的点（或一组最近的点）作为对应点。这些对应点通常来自目标点云。
3. **估计变换**：根据源点云和目标点云之间的对应点对，计算最佳的刚体变换（包括平移和旋转）。这个变换旨在最小化对应点间的欧几里得距离总和。通常使用最小二乘法来估计这个变换。
4. **应用变换**：将估计的变换应用到源点云上，以改进其与目标点云的对齐。
5. **迭代和收敛判定**：重复步骤2到步骤4，直到达到预定的迭代次数，或者变换更新量低于某个阈值，表明已经收敛到最优解。



