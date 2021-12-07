# Gazebo Test Field for NICS AKM Car

## Build

Ubuntu18.04 + ros melodic

预先安装以下功能包（若有遗漏则根据报错自行安装）

```shell
sudo apt-get install ros-melodic-ackermann-msgs
sudo apt-get install ros-melodic-navigation
sudo apt-get install ros-melodic-geographic-info
sudo apt-get install ros-melodic-controller-manager
sudo apt-get install ros-melodic-gazebo-ros-control
sudo apt-get install ros-melodic-effort-controllers
sudo apt-get install ros-melodic-joint-state-controller
sudo apt-get install ros-melodic-position-controllers
sudo apt-get install ros-melodic-teb-local-planner
sudo apt-get install ros-melodic-velocity-controllers
```

**请修改nics_world.world、nics_world_cone.world中的URI路径，**并编译功能包。

## Introduction

- nics_bringup

  NICS赛道的配置和加载，包含载入带有摄像头和雷达的小车以及赛道gazebo环境，其中赛道有三种模式：简单、静态障碍物和动态障碍物。**动态障碍物环境中，会有一辆AI小车绕赛道运动**，并且用户小车上的摄像头和激光雷达均可探索到这个AI小车。

- car_sim

  包含小车的描述、基本控制和基础功能的功能包。

- nicsrobot_line_follower

  巡线任务功能包。

## Tutorials

```
roslaunch nics_bringup nics_gazebo_simple.launch # NICS赛道模拟，运行前请修改.world文件中的路径
roslaunch nics_bringup nics_gazebo_block.launch # NICS障碍赛道模拟，运行前请修改.world文件中的路径
roslaunch nics_bringup nics_gazebo_actor.launch # NICS动态障碍赛道模拟，运行前请修改.world文件中的路径
roslaunch akm_control keyboard_teleop.launch # 小车键盘控制
roslaunch nicsrobot_line_follower nicsrobot_line_follower.launch # 简单场景巡线任务
```

改变模型位置：http://gazebosim.org/tutorials?tut=ros_comm&cat=connect_ros

```
rosservice call /gazebo/set_model_state	'{model_state: {model_name: AKM_*, pose: {position: {x: **, y: **, z: **}}, reference_frame: world}}'
```

在本功能包中可以通过

```
rosrun akm_control set_model_state.py AKM_* <x> <y> <theta>
```

改变小车位置，传入的四个参数分别是小车名、相对世界坐标系的横纵坐标和 yaw 角（弧度制）

## Debug

What to do First: Install necessary packages according to README.md

### 1.

> CMake Error at /usr/local/share/cmake-3.19/Modules/FindQt4.cmake:1314 (message): Found unsuitable Qt version "" from NOTFOUND, this code requires Qt 4.x

```
sudo apt install cmake gcc g++ qt{4,5}-qmake libqt4-dev	
```

### 2.

> -- No package 'orocos-bfl' found

```
sudo apt-get install ros-melodic-bfl
```

### 3.

> Could not load controller 'left_rear_wheel_velocity_controller' because controller type 'effort_controllers/JointVelocityController' does not exist.

```
sudo apt-get install ros-melodic-joint-state-controller
sudo apt-get install ros-melodic-effort-controllers
sudo apt-get install ros-melodic-position-controllers
```

其余类似的 robot_state_publisher 也是如此

### 4.

> Could not find a package configuration file provided by "image_transport" with any of the following names:
>
> image_transportConfig.cmake image_transport-config.cmake

```
sudo apt-get install ros-melodic-compressed-image-transport
```

### 5.

> Could not find a package configuration file provided by "tf(eigen)_conversions"

```
sudo apt-get install ros-melodic-tf(eigen)-conversions
```

### 6.

All nodes are in the set namespace, so there is no need to add "/" before the topic name.

"/" means root directory and "~" means current namespace + node name.

### 7.

namespace 存在时在 rviz 中显示 "Frame does not exist"，建议检查 tf_tree 中所有的 Frame_id 前是否有 namespace，若没有可以通过以下方式传入：

```
<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher">
    <param name="tf_prefix" value="$(arg robot_name)"/>
</node>
```

### 8. 没有joint_states

 实际是因为模型的joint没有加载进来，可能有以下两个原因

- 加载param文件时命名空间重复，即外面有namespace，yaml文件中又在开头加了namespace;
- joint_states实际由/gazebo发出，依赖于controller_manager，同样如果<gruop>标签中已经定义了ns，则不需要在controller_manager传入的参数中再加入namespace

### 9. 无法控制小车

```shell
sudo apt-get install ros-melodic-rqt-controller-manager
```

之后可通过

```shell
rosrun rqt_controller_manager rqt_controller_manager
```


查看各controller状态确定其是否正常工作

## Notes

- 11.23: 合并巡线任务 
- 11.18: 给赛道加入围栏
- 11.30: 更新车体外观，加入静态障碍物
- 12.3: 加入AI小车作为动态障碍物，根据新的车体调整了巡线任务代码
- 12.7: 解决巡线任务不停止的问题

## Ref

https://blog.csdn.net/qq_36754438/article/details/109125320

