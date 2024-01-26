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
