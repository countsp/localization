# icp

![Screenshot from 2024-07-12 08-52-39](https://github.com/user-attachments/assets/1146e3c7-ec3b-4d11-a181-9b76b400c55c)

注意：


1.初值常常通过粗匹配得到（PFH)

2.最近临用Kd-tree

3.source点云的 points数量 不能太多 ，通过voxelgrid 降采样

4.收敛判断（综合判断，避免局部最优）

5.优化方向

鲁棒核函数

有置信度高的先验时，建立权重矩阵
