# nicsrobot_line_follower

## Tutorials

这是在gazebo的仿真环境中运行的版本，整个工程也更加简单

仿真环境参考：[https://github.com/ppppplus/NICS_GZB]()

#### 安装步骤：

（依赖库请自行安装）

```shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone -b simulation https://gitee.com/xiang-yunfei/nicsrobot_line_follower.git
chmod -R +x nicsrobot_line_follower/scripts
cd ..
catkin_make
source devel/setup.bash
```

#### 关卡一：视觉巡线

运行步骤的（首先应打开仿真环境）：

```
source ~/catkin_ws/devel/setup.bash
roslaunch nicsrobot_line_follower nicsrobot_line_follower.launch
```
