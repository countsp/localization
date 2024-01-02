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
> **uint8 COVARIANCE_TYPE_APPROXIMATED = 1**
> **uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2**
> **uint8 COVARIANCE_TYPE_KNOWN = 3**
>  
> **uint8 position_covariance_type**
