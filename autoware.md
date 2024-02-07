### 状态向量表示法
在 EKFLocalizer 中，dim_x_(6) 表示扩展卡尔曼滤波器（Extended Kalman Filter, EKF）状态向量的维度。状态向量是EKF用来估计系统状态的关键部分。在这个特定的例子中，状态向量的维度为6，包含以下各个分量：

x：在二维空间中的x坐标，通常代表物体在水平平面上的东西位置。

y：在二维空间中的y坐标，通常代表物体在水平平面上的南北位置。

yaw：偏航角，表示物体在水平平面上的朝向或方向。偏航角是围绕垂直轴的旋转角度，通常以弧度或度来度量。

yaw_bias：偏航偏差，用于表示偏航角的长期偏差或漂移。在一些应用中，如自动导航，考虑偏航偏差可以提高方向估计的准确性。

vx：沿x轴的速度，即在东西方向上的速度。

wz：绕z轴的角速度，即围绕垂直轴的旋转速度。

---
### HADMapBin (Highly Automated Driving)
HADMapBin是一个用于表示高自动化驾驶（Highly Automated Driving, HAD）地图数据的二进制格式。它是一种专门用于ROS（Robot Operating System）消息传递系统中，以二进制形式传输Lanelet2地图数据的格式。Lanelet2是一种用于自动驾驶和智能车辆应用的开源地图格式，它以细节丰富的方式描述了道路网络，包括车道、交通标志、信号灯等信息

---

###  lanelet::PolygonLayer
lanelet::PolygonLayer在Lanelet2库中封装了地图中多边形区域的信息。具体来说，它包含以下信息：

**多边形的集合**：每个多边形由一系列的顶点定义，这些顶点以地理坐标（经度、纬度）或投影坐标（如UTM坐标）的形式给出，从而形成封闭的区域。lanelet::PolygonLayer可以包含一个或多个这样的多边形。

**属性和标签**：每个多边形可以有与之相关的属性和标签，用于描述该多边形的特性，如“行人区域”、“停车区”或“绿化带”等。这些属性可以用于地图的解析、可视化和导航规划。

**地理位置信息**：lanelet::PolygonLayer中的多边形通过其顶点的地理位置信息来定义其在现实世界中的位置和形状。

**关系和连接**：Lanelet2库还允许定义多边形之间的关系和连接，比如一个多边形可以是另一个多边形的邻居。这种信息有助于理解地图的结构和多边形之间的空间关系。

**元数据**：除了地理信息和属性外，lanelet::PolygonLayer还可以包含其他元数据，如创建时间、作者信息或用于特定应用的自定义数据。

#### lanelet::PolygonLayer是Lanelet2地图模型的一部分，与道路（lanelets）、交叉口（intersections）等其他元素一起，为自动驾驶系统提供了一个全面的道路和环境表示。通过使用这些多边形层，自动驾驶系统可以更好地理解和解释地图上的非道路区域，为路径规划和决策提供支持。

--- 
### Eigen::Affine3f

Eigen::Affine3f = Eigen::Translation3f *  Eigen::Quaternionf

---

### Sophus::SE3f
Eigen::Affine3f = {Eigen::Quaternionf, Eigen::Translation3f }

---
