# icp

## 点点匹配
![Screenshot from 2024-07-12 08-52-39](https://github.com/user-attachments/assets/1146e3c7-ec3b-4d11-a181-9b76b400c55c)

注意：


1.初值常常通过粗匹配得到（PFH)

2.最近临用Kd-tree

3.source点云的 points数量 不能太多 ，通过voxelgrid 降采样

4.收敛判断（综合判断，避免局部最优）

5.优化方向

鲁棒核函数

有置信度高的先验时，建立权重矩阵

## 求残差分别 对R t的雅各比矩阵

![Screenshot from 2024-07-12 09-32-54](https://github.com/user-attachments/assets/adc1c9fb-9ff0-4f73-acbe-5b37c5bf21da)

---

## 点线匹配

### 求残差
![Screenshot from 2024-07-12 09-57-10](https://github.com/user-attachments/assets/926b53fe-474e-421a-ad54-fb7f275c6780)

### 求残差分别 对R t的雅各比矩阵
![Screenshot from 2024-07-12 10-01-31](https://github.com/user-attachments/assets/94790170-a489-432c-8244-a49e6dbca400)

---

## 点面匹配
![Screenshot from 2024-07-12 10-05-59](https://github.com/user-attachments/assets/a9001522-1636-44e9-a652-690971b423a9)
