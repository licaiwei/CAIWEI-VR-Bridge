# CAIWEI-VR-Bridge
ROS VR Bridge is a package for OpenVR developed by the caiwei. The main application of the package is to provide a communication interface between OpenVR and ROS. The aim is to control the VR devices using ROS messages and ROS services.

OpenVR在ROS下的package。该软件包的主要应用是提供OpenVR与ROS之间的通信接口。目的是使用ROS消息和ROS服务来控制VR设备。

![](C:\Users\lii\Documents\CAIWEI_Open_source\CAIWEI-VR-Bridge\assets\tf.gif)

## 设备与环境

ubuntu 18.04 ROS Melodic

Steam + SteamVR

[HTC-Vive](https://www.vive.com/cn/)

[OpenVR](https://github.com/ValveSoftware/openvr)

## [安装ROS](http://wiki.ros.org/melodic/Installation/Ubuntu)

## 安装SteamVR

[安装Steam](https://store.steampowered.com/about/)

在steam界面内打开 steamVR，首次打开要下载安装一段时间。

## 编译package

打开一个`termial`，定位到你的 `workspace/src` 下

```bash
git clone git@github.com:licaiwei/CAIWEI-VR-Bridge.git
cd ..
catkin_make
source devel/setup.bash
```

## 使用 ros_vr_bringup

打开SteamVR

#### 仅使用设备坐标

```bash
rosrun ros_vr_bringup tf_state_publisher
```

初始化并占用了OpenVR的状态机，发布所有VR设备的TF。打开`Rviz`，如第一张图所示。

#### 使用VR

```bash
rosrun ros_vr_bringup vr_bringup
```

初始化并占用了OpenVR的状态机，发布所有VR设备的TF，订阅 `/to_VR_bring/imagel`和`/to_VR_bring/imager` 两个`sensor_msgs::Image`类型的Topic，并将其分别显示在HMD（头戴式显示器）的左右眼上。

## 应用案例 ros_vr_gazebo

将`model`文件夹中的`vr_controller_r`，`vr_controller_r`，`vr_HMD`三个模型（文件夹）放在你的gazebo模型路径下，通常这个路径为`~/.gazebo/model/`。将`model`文件夹中的`vr_gazebo.world`文件放在gazebo的环境变量路径里面
```bash
env | grep GAZEBO_RESOURCE_PATH
```
一个典型的路径是`/usr/local/share/gazebo7`。在这个路径后面加上`/worlds`就可以了，里面有很多`.world`文件。

先运行`vrbringup`

```bash
rosrun ros_vr_bringup vr_bringup
```

再运行gazebo下的vr环境

```bash
roslaunch ros_vr_gazebo vr_gazebo_demo.launch
```

这将会启动gazebo环境，并加载`vr_controller_r`，`vr_controller_r`，`vr_HMD`三个模型，`vr_HMD`包含调教好的双目相机插件，发布`sensor_msgs::Image`类型的两个Topic`/to_VR_bring/imagel`和`/to_VR_bring/imager` ，时间戳一致。同时还会启动`vr_device_states_controller`节点，该节点订阅VR设备的TF，并将gazebo中的模型设置到对应的坐标。可以实现在gazebo中的VR效果，欢迎来到机器人的世界。
![](C:\Users\lii\Documents\CAIWEI_Open_source\CAIWEI-VR-Bridge\assets\gazebo_vr.gif)

