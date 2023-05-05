# Yolov8_ros

For simplified Chinese version: [简体中文版](./README_CN.md) 

This package provides a ROS wrapper for [PyTorch-YOLOv8](https://github.com/ultralytics/ultralytics) based on PyTorch-YOLOv8. The package has been tested with Ubuntu 18.04 and Ubuntu 20.04.

V1.0.1: Add device options(cpu or gpu).

**Authors**: Zhitao Zheng (<qq44642754@163.com>)

<p>
   <img width = "1000" src="https://github.com/qq44642754a/Yolov8_ros/blob/master/yolov8_ros/media/image.png"></a>
</p>


# develop environment：
- Ubuntu 18.04 / 20.04
- ROS  Melodic / Noetic
- Python>=3.7.0 environment, including PyTorch>=1.7

# Prerequisites:

Pip install the ultralytics package including all [requirements](https://github.com/ultralytics/ultralytics/blob/main/requirements.txt) in a [**Python>=3.7**](https://www.python.org/) environment with [**PyTorch>=1.7**](https://pytorch.org/get-started/locally/).

```
pip install ultralytics
pip install rospkg
```

## Installation yolov8_ros

```
cd /your/catkin_ws/src

git clone https://github.com/qq44642754a/Yolov8_ros.git

cd ..

catkin_make

```

## Basic Usage

1. First, make sure to put your weights in the [weights](https://github.com/qq44642754a/Yolov8_ros/tree/master/yolov8_ros/weights) folder. 
2.  The default settings (using `yolov8s.pt`) in the `launch/yolo_v8.launch` file should work, all you should have to do is change the image topic you would like to subscribe to:

```
roslaunch yolov8_ros yolo_v8.launch
```

  
  Alternatively you can modify the parameters in the [launch file](https://github.com/qq44642754a/Yolov8_ros/tree/master/yolov8_ros/launch), recompile and launch it that way so that no arguments need to be passed at runtime.

### Node parameters

* **`image_topic`** 

    Subscribed camera topic.

* **`weights_path`** 

    Path to weights file.

* **`pub_topic`** 

    Published topic with the detected bounding boxes.
    
* **`confidence`** 

    Confidence threshold for detected objects.
    


