# Robomaster Algorithm Tutorial - Navigation Pipeline
# 25赛季培训——导航入门
## 一些仿真包，pcl包，tf包 etc
```bash
sudo apt update
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
                 ros-noetic-pcl-ros ros-noetic-pcl-conversions \
                 ros-noetic-rviz ros-noetic-tf ros-noetic-sensor-msgs \
                 ros-noetic-velodyne-simulator

```
## 雷达可视化大概是这样子
```bash
roslaunch velodyne_description example.launch
```

## 导航都干些什么
导航模块主要负责机器人（**哨兵**）在比赛场地内的自主移动与路径规划。它需要处理以下几个关键任务：
- **环境感知**：通过传感器（如激光雷达、摄像头等）获取周围环境的信息，构建地图。
- **状态估计**：通过传感器数据和运动模型，估计机器人在地图中的位置和姿态。
- **路径规划**：根据目标位置和环境地图，计算出一条可行的路径。
- **运动控制**：将规划好的路径转换为具体的运动指令，控制机器人移动，实时避障等等。
- **其他任务**：和其他模块通信交互，如决策，雷达站，视觉等模块。
## 今天培训的侧重
我们今天不会把以上全部包含，在今天的操作中，我们主要通过聚焦关于点云输入处理这一个细节，让大家对导航模块有一个初步的了解。我们会介绍点云数据的获取、处理和使用方法。
## 硬件-LiDar
- LiDar = Light Detection And Ranging
- 通过发射激光束并测量其反射时间来获取周围环境的距离信息，从而构建三维空间模型。
- 与使用电磁波的雷达（Radar）不同，LiDar使用的是激光光束，具有更高的分辨率和精度。
- 我们使用Mid-360激光雷达，关于使用和驱动，可以参考官方的[livox_ros_driver2文档](https://github.com/Livox-SDK/livox_ros_driver2)
### LiDar 配置文件
LiDar使用json格式的配置文件，由于我们在哨兵使用双雷达，所以需要配置两个雷达的参数，主要包括：
- **frame_id**：雷达数据的坐标系名称。
- **ip**：雷达的IP地址。
- **extrinsic_parametres**：雷达相对于机器人坐标系的位姿，包括位置（x, y, z）和姿态（roll, pitch, yaw）。
(这里作为补充，实际上手时会对这个配置文件进行更多操作)
## 点云
点云是由激光雷达扫描得到的一组三维坐标点，代表了机器人周围环境的空间结构。每个点包含了X、Y、Z三个坐标值，有时还包含强度信息。通过处理点云数据，机器人可以识别障碍物、地形等信息，从而进行路径规划和避障。
一般点云的格式为：
```
pcl::PointCloud<pcl::PointXYZ>
pcl::PointCloud<pcl::PointNormal>
```
其中，`pcl::PointXYZ`表示每个点包含X、Y、Z坐标，而`pcl::PointNormal`则在此基础上还包含法向量信息。
每一个类的域一般会包含，比如：
```
float32 x
float32 y
float32 z
float32 intensity
float64 timestamp
```
这样的信息。
## PCL库
- 一个开源的点云处理库，提供了丰富的算法和工具，用于点云的过滤、分割、配准、特征提取等操作。
- 在导航模块中，PCL库被广泛用于处理激光雷达获取的点云数据，以实现环境感知和路径规划。
- 官方网站：[Point Cloud Library (PCL)](https://pointclouds.org/)
## 点云处理流程
- 订阅点云数据：通过ROS订阅激光雷达发布的点云消息。
- 点云预处理：包括滤波、下采样等操作，以减少噪声和数据量。
- 分段与聚类：将点云分割成不同的区域或对象。
- 特征提取：计算点云的几何特征，如法向量、曲率等。
- 发布处理后的点云：将处理后的点云数据发布到ROS话题，供其他模块使用。
## 预处理
### 下采样
- 目的：减少点云数据量，提高处理速度，同时保留关键的几何特征。
- 方法：体素网格滤波（Voxel Grid Filter）
- 过程：将点云划分为固定大小的体素（立方体），并用每个体素内的点的质心来代表该体素，从而实现数据的压缩。
### 滤波
- 目的：去除点云中的噪声和异常点，提高数据质量。
    
- 方法：统计离群点移除（Statistical Outlier Removal）
- 详细过程：计算每个点与其邻近点的距离，基于统计分析移除那些距离过远的点。
## 分段与聚类
- 目的：将点云分割成不同的区域或对象，便于后续处理。
- 方法：欧几里得聚类（Euclidean Cluster Extraction）
- 过程：基于点之间的距离，将点云划分为多个簇，每个簇代表一个独立的对象或区域。

或者

- 方法：基于模型的分割（如RANSAC）
- RANSAC的原理：随机采样一致性算法，通过迭代方式从点云中拟合模型，并识别符合该模型的点，从而实现分割。
- 过程：选择一个模型（如平面、圆柱等），通过RANSAC算法识别并分割出符合该模型的点云部分。

## 特征提取
- 目的：计算点云的几何特征，为后续的识别和分类提供依据。
- 方法：法向量计算、曲率估计等
- 过程：利用PCL库中的算法，计算每个点的法向量和曲率等特征信息。

