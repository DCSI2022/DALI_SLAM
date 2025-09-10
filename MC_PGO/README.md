# Backend_mapping





### 1.1 位姿图优化
使用命令行传入参数，在rosbag目录下Log文件夹内输出优化后的轨迹与全局地图（可选）

参数1: rosbag绝对路径；

参数2: odometry traj绝对路径；

参数3: LiDAR至IMU外参绝对路径。

参数4: 雷达类型（1 LIVOX 2 VELODYNE 3 OUSTER 4 HESAI）；

参数5: 激光消息名称；

参数6: rosbag开始时间；

参数7: rosbag持续时间；

参数8: 子图时间跨度；

参数9: 重叠度阈值；

其余参数在代码中默认设置如下: 

1）宏定义参数

NoConsecutiveSubmapSearch 10.01 //共视子图搜索范围

EnableNonPlanarFilter 是否使用非平面点滤除

EnableOverlapConstraint 是否添加共视子图约束

EnableLoopConstraint 是否添加回环子图约束

2)代码中: 

point_filter_num(loadLidarDataFromRosbag): 3

子图配准体素大小，huber核函数，匹配点对距离阈值等参数

### 1.2 地图绝对误差计算
见map_error_compute.cpp

### 1.3 地图熵计算
见mean_map_entropy.cpp

### 1.4 地理定向地图生成
见gtMap_generate.cpp

## 2 引用
在2023 ICCV SLAM挑战赛“LiDAR-inertial”赛道获得全球季军

Robust LiDAR-inertial odometry with multi-constraint PGO方法分享：https://www.youtube.com/watch?v=VnULxGjk9-8&t=227s 

## 3 TODO
1、加入重力约束

2、加入错误回环剔除策略

