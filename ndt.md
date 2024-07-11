
![ndt1](https://github.com/countsp/localization/assets/102967883/bd554a41-d743-4736-abe1-7e7b85f86b9d)

![ndt2](https://github.com/countsp/localization/assets/102967883/58c2ecbc-87e8-420c-9673-74cb758c998a)

# 1. 正态分布变换(NDT)

在点云匹配中，ICP基于距离直接最优化变换矩阵的参数，由于是欠定方程且旋转矩阵的约束，使得结果很难优化，为此在新的维度优化变换矩阵的参数，被很好的提出：

先将参考点云（目标点云）转换为多维变量的正态分布，匹配的点云如果采用某组变换参数后，使得新的点云和目标点云的正态分布参数匹配很好，那么变换点在参考系中的概率密度将会很大。因此，可以考虑用优化的方法求出使得概率密度之和最大的变换参数，此时两幅激光点云数据将匹配的最好。

# 2. 多元正态分布

我们知道，如果随机变量X满足正态分布X∼N(μ,σ),则其概率密度函数为：

![image](https://github.com/countsp/localization/assets/102967883/36dd60b3-58e5-455b-8a26-645ca9b49b5f)


对于多元正态分布而言，其概率密度函数可以表示为：

![image](https://github.com/countsp/localization/assets/102967883/9b2356a1-3c32-4a65-a26e-e103629d9133)

## 2.1. NDT算法流程

将参考点云网格化，计算每个网格的概率密度函数：

![image](https://github.com/countsp/localization/assets/102967883/ebba1436-180e-4e66-9a41-46f1b06b90e2)

网格的概率密度函数则为：

![image](https://github.com/countsp/localization/assets/102967883/64a5dcde-2914-4af7-b61a-caa2ee32d59e)

## 2.2. 变换参数和最大似然

我们需要优化的参数就是对当前点云的坐标变换（旋转，平移等），转换函数表示使用姿态变换\vec{p}来变换\vec{x_{k}}，结合之前的一组状态密度函数，那么最好的变换参数\vec{p}应该是最大化似然函数的姿态变换：

![image](https://github.com/countsp/localization/assets/102967883/798bac56-a9af-4fb0-99f1-e561a0bacf21)

 那么最大化似然也就相当于最小化负对数似然 −logΘ;

![image](https://github.com/countsp/localization/assets/102967883/cab8c6f1-e2a0-453f-8238-7b4c954acd9a)
             
就到了我们最熟悉的最优化的部分了。

现在的任务就是使用优化算法来调整变换参数\vec{p} ​ 来最小化这个负对数似然。NDT算法使用牛顿法进行参数优化。


# 代码
```
#include <iostream>
#include <thread>
 
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
 
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
 
#include <pcl/visualization/pcl_visualizer.h>
 
using namespace std::chrono_literals;
 
int main ()
{
  // Loading first scan of room.
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("room_scan1.pcd", *target_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << target_cloud->size () << " data points from room_scan1.pcd" << std::endl;
 
  // Loading second scan of room from new perspective.
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("room_scan2.pcd", *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan2.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << input_cloud->size () << " data points from room_scan2.pcd" << std::endl;
 
  // Filtering input scan to roughly 10% of original size to increase speed of registration.
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
  approximate_voxel_filter.setInputCloud (input_cloud);
  approximate_voxel_filter.filter (*filtered_cloud);
  std::cout << "Filtered cloud contains " << filtered_cloud->size ()
            << " data points from room_scan2.pcd" << std::endl;
 
  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
 
  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon (0.01);
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize (0.1);
  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution (1.0);
 
  // Setting max number of registration iterations.
  ndt.setMaximumIterations (35);
 
  // Setting point cloud to be aligned.
  ndt.setInputSource (filtered_cloud);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget (target_cloud);
 
  // Set initial alignment estimate found using robot odometry.
  Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
  Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();
 
  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align (*output_cloud, init_guess);
 
  std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
            << " score: " << ndt.getFitnessScore () << std::endl;
 
  // Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());
 
  // Saving transformed input cloud.
  pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *output_cloud);
 
  // Initializing point cloud visualizer
  pcl::visualization::PCLVisualizer::Ptr
  viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_final->setBackgroundColor (0, 0, 0);
 
  // Coloring and visualizing target cloud (red).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  target_color (target_cloud, 255, 0, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "target cloud");
 
  // Coloring and visualizing transformed input cloud (green).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  output_color (output_cloud, 0, 255, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "output cloud");
 
  // Starting visualizer
  viewer_final->addCoordinateSystem (1.0, "global");
  viewer_final->initCameraParameters ();
 
  // Wait until visualizer window is closed.
  while (!viewer_final->wasStopped ())
  {
    viewer_final->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }
 
  return (0);
}```
