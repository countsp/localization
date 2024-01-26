### 状态向量表示法
在 EKFLocalizer 中，dim_x_(6) 表示扩展卡尔曼滤波器（Extended Kalman Filter, EKF）状态向量的维度。状态向量是EKF用来估计系统状态的关键部分。在这个特定的例子中，状态向量的维度为6，包含以下各个分量：

x：在二维空间中的x坐标，通常代表物体在水平平面上的东西位置。

y：在二维空间中的y坐标，通常代表物体在水平平面上的南北位置。

yaw：偏航角，表示物体在水平平面上的朝向或方向。偏航角是围绕垂直轴的旋转角度，通常以弧度或度来度量。

yaw_bias：偏航偏差，用于表示偏航角的长期偏差或漂移。在一些应用中，如自动导航，考虑偏航偏差可以提高方向估计的准确性。

vx：沿x轴的速度，即在东西方向上的速度。

wz：绕z轴的角速度，即围绕垂直轴的旋转速度。