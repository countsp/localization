# ekf流程
// simplest structure
```
#include <Eigen/Dense>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class ExtendedKalmanFilter {
 public:
    ExtendedKalmanFilter(int state_dim, int measurement_dim)
        : x_(VectorXd::Zero(state_dim)),
          P_(MatrixXd::Identity(state_dim, state_dim)),
          Q_(MatrixXd::Identity(state_dim, state_dim)),
          R_(MatrixXd::Identity(measurement_dim, measurement_dim)),
          H_(MatrixXd::Zero(measurement_dim, state_dim)),
          F_(MatrixXd::Identity(state_dim, state_dim)) {}

    void Predict() {
        // 状态预测
        x_ = F_ * x_;
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    void Update(const VectorXd &z) {
        // 观测更新
        VectorXd y = z - H_ * x_;
        MatrixXd S = H_ * P_ * H_.transpose() + R_;
        MatrixXd K = P_ * H_.transpose() * S.inverse();

        x_ = x_ + K * y;
        P_ = (MatrixXd::Identity(x_.size(), x_.size()) - K * H_) * P_;
    }

    // 设置状态转换矩阵
    void SetF(const MatrixXd &F) {
        F_ = F;
    }

    // 设置测量矩阵
    void SetH(const MatrixXd &H) {
        H_ = H;
    }

    // 设置过程噪声协方差矩阵
    void SetQ(const MatrixXd &Q) {
        Q_ = Q;
    }

    // 设置观测噪声协方差矩阵
    void SetR(const MatrixXd &R) {
        R_ = R;
    }

    // 获取当前状态估计
    VectorXd GetState() const {
        return x_;
    }

 private:
    VectorXd x_; // 状态
    MatrixXd P_; // 状态协方差
    MatrixXd Q_; // 过程噪声协方差
    MatrixXd R_; // 观测噪声协方差
    MatrixXd H_; // 测量矩阵
    MatrixXd F_; // 状态转换矩阵
};

int main() {
    // 示例：1D状态空间，1D观测空间
    ExtendedKalmanFilter ekf(1, 1);

    // 初始化状态转换矩阵和测量矩阵
    ekf.SetF(MatrixXd::Identity(1, 1));
    ekf.SetH(MatrixXd::Identity(1, 1));

    // 初始化过程噪声和观测噪声协方差矩阵
    ekf.SetQ(MatrixXd::Identity(1, 1) * 0.1);
    ekf.SetR(MatrixXd::Identity(1, 1) * 0.1);

    // 模拟一系列观测值和状态预测
    for (int i = 0; i < 5; ++i) {
        ekf.Predict();
        VectorXd z(1);
        z << i + 0.1 * (rand() % 10); // 模拟观测值
        ekf.Update(z);

        std::cout << "State after update " << i << ": " << ekf.GetState().transpose() << std::endl;
    }

    return 0;
}

```
#### 预测（Predict()）:
  
  这个步骤用于根据系统的动态（通过状态转换矩阵F_表示）来预测下一个状态。
  
  状态预测是通过计算x_ = F_ * x_实现的，这里x_是当前状态估计。
  
  同时，状态协方差P_也被更新，以反映预测的不确定性。新的协方差是通过P_ = F_ * P_ * F_.transpose() + Q_计算的，其中Q_是过程噪声协方差。

#### 更新（Update()）:
  
  在这个步骤中，使用新的观测数据z来更新状态估计。

  首先，计算观测残差y = z - H_ * x_，这里H_是测量矩阵，用于将状态映射到观测空间。
  
  然后，计算卡尔曼增益K，它是用来决定观测数据对最终状态估计的影响程度。

  最后，使用卡尔曼增益和观测残差来更新状态估计：x_ = x_ + K * y，并更新状态协方差：P_ = (MatrixXd::Identity(x_.size(), x_.size()) - K * H_) * P_。
  
## 马氏距离门控（Mahalanobis Distance Gate）

马氏距离门控是一种在数据融合和状态估计中常用的技术，特别是在卡尔曼滤波器的应用中。它利用马氏距离（Mahalanobis Distance）来评估测量数据与当前状态估计之间的一致性。马氏距离考虑了数据的协方差，因此它是一种在统计意义上的距离度量。
马氏距离的定义：

马氏距离是一个多维空间中的度量，用于测量一个点与一个分布之间的距离。给定一个点 x 和一个分布的均值 μ 及其协方差矩阵 S，马氏距离 D_M​ 定义为：

$$
D_M(x, \mu) = \sqrt{(x - \mu)^T S^{-1} (x - \mu)}
$$

​
### 马氏距离门控在卡尔曼滤波中的应用：
#### 测量一致性检查：
在卡尔曼滤波器中，使用马氏距离来评估测量数据 yy 相对于滤波器当前状态估计 x^x^ 的一致性。如果这个距离过大，测量数据可能被认为是异常值（outlier）。

#### 门控阈值：

通常设定一个马氏距离的阈值。如果计算出的马氏距离超过这个阈值，测量更新可能会被忽略，以防止异常数据干扰状态估计。

#### 提高鲁棒性： 

这种方法提高了滤波器对噪声和异常值的鲁棒性，确保仅当测量数据与当前状态估计在统计意义上足够接近时，才进行状态更新。

在实际应用中，如自动驾驶、机器人导航等，马氏距离门控帮助确保了算法的准确性和鲁棒性，通过排除那些与当前状态估计显著不一致的测量数据。


