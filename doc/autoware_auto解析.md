# Autoware.Auto-v1.0.0 深入解析

Autoware.Auto-v1.0.0是AVP Demo释放时的代码，以这个版本为基本开始深入研究。

一级章节均为源代码`src`中的一级子目录，二级章节为相应的二级子目录。只针对最主要的核心内容进行解读，不重要的包省略不述，解读次序尽量按照调用顺序。

## 一、tools

### 1.1 autoware_auto_create_pkg

本包是一个小工具，可以生成一个脚本标准化所有包的创建，类似于在IDE中新建项目时，自动生成的项目文件。

### 1.2 autoware_auto_avp_demo

launch文件夹下包含launch文件，用于单命令启动整个项目工程。

其中文件名包含：

- `_core.launch.py` 包含核心包
- `_sim.launch.py` 用于仿真
- `_vehicle.launch.py` 用于实车

### 1.3 autoware_demos

启动单个模式高度的launch文件 

### 1.4 joystick_vehicle_interface, joystick_vehicle_interface_nodes

用于外接遥控杆的控制接口

### 1.5 lidar_integration

用于激光雷达3D感知算法的的测试，可以模拟VLP16的UDP数据。

### 1.6 test_trajectory_following

用于测试测试跟踪模块，主要用于控制器与仿真器之间的集成测试。

### 1.7 visualization

用于支持 Autoware.Auto 中各个包的可视化测试。

## 二、common

### 2.1 algorithm

包含常用的算法，使用 algorirhm.hpp 包含其它算法的头文件。当前仅有 quick_stort.hpp 快速排序算法，并且使用模板类实现。

### 2.2 autoware_auto_common

本包又分为两个子模块：

* common：文件系统 filesystem.hpp 用于创建目录、文件，UTF8编码格式变换等操作；类型定义 types.hpp 用于定义常用的数据类型
* helper_functions：用于浮点数比较、msg-adapter等

### 2.3 autoware_auto_geometry

提供一些数据结构 interval、lookup_table、spatial_hash等。

### 2.4 autoware_auto_tf2

Ros中Tf2的超集，主要增加了Autoware.Auto自定义Msg的坐标变换

### 2.5 covariance_insertion

暂未知，从翻译来看是：协方差插入

### 2.6 covariance_insertion_nodes

将输入的消息增加协方差field或者覆盖已有的协方差field

### 2.7 had_map_utils

应该是判断是否有地图

### 2.8 lidar_utils

提供一些激光雷达点云的参数和操作函数

### 2.9 motion_mode

提供一些运动估计的常用模型，CV、CA、CATR等。

### 2.10 mpark_variant_vendor

本包由开源项目供应商 mpark/variant 提供，主要是数据类型 std::variant 的 C++17 实现。

### 2.11 optimization

提供一些优化方法，比较牛顿法、线性搜索法(more-thuente)等。

### 2.12 reference_tracking_controller

提供一些简单控制器的API参数，如PID。

### 2.13 滤波器

对一些信号进行频域过滤

### 2.14 time_utils

提供一些时间处理的便利函数

