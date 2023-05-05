# Yolov8_ros

For English version: [English](./README.md) 

提供了一个基于PyTorch-YOLOv8的[PyTorch-YOLOv8](https://github.com/ultralytics/ultralytics)的ROS功能包。该功能包已在Ubuntu18.04和Ubuntu 20.04上进行了测试。

Authors: Zhitao Zheng (qq44642754@163.com)

<p>
   <img width = "1000" src="https://github.com/qq44642754a/Yolov8_ros/blob/master/yolov8_ros/media/image.png"></a>
</p>

# 开发环境：
- Ubuntu 18.04 / 20.04
- ROS melodic / Noetic
- Python>=3.7.0环境，PyTorch>=1.7

# 环境配置安装步骤：

## 安装符合对应的python和pytorch版本的环境（建议使用anaconda），然后安装以下依赖。

```
pip install ultralytics
pip install rospkg
```


## 安装Yolov8_ROS

```
cd /your/catkin_ws/src

git clone https://github.com/qq44642754a/Yolov8_ros.git

cd ..

catkin_make

```

## 基本用法

1. 首先，确保将您训练好的权重放在 [weights](https://github.com/qq44642754a/Yolov8_ros/tree/master/yolov8_ros/weights) 文件夹中。
2. `launch/yolo_v8.launch` 文件中的默认使用`yolov8s.pt这个权重文件，另外您需要在launch文件中额外修改您对应的摄像头话题名称以及是否使用Cpu选项：
```
roslaunch yolov8_ros yolo_v8.launch
```
