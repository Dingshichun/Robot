## ROS Robot APP导航

注：虚拟机需要与小车处在同一个局域网下，且ROS_DOMAIN_ID，需要一致，可以查看【使用前必看】来设置板子上的IP和ROS_DOMAIN_ID。

### 1、程序功能说明

小车连接上代理，运行程序，手机与小车连接通过一个网络。打开手机上下载的【ROS Robot】app，输入小车的IP地址，选择ROS2，点击连接，即可连接上小车。选择【导航】，点击App界面的【设定初始化点】设置小车起始位姿，点击App界面【设置导航点】，给定小车目标点，随后小车会规划路径移动到该点。

### 2、启动并连接代理

以配套虚拟机为例，输入以下指令启动代理，

```shell
#小车代理
sudo docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8090 -v4
#摄像头代理（先启动代理再打开小车开关）
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 9999 -v4
```

![image-20240102120030310](image-20240102120030310.png)

然后，打开小车开关，等待小车连接上代理，连接成功如下图所示，

![image-20231219145735869](image-20231219145735869.png)

### 3、启动程序

首先启动小车处理底层数据程序，终端输入，

```shell
ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py
```

![image-20231219145831295](image-20231219145831295.png)

启动APP导航命令，终端输入，

```shell
ros2 launch yahboomcar_nav navigation_dwb_app_launch.xml maps:=/home/yahboom/yahboomcar_ws/src/yahboomcar_nav/maps/testaa.yaml
```

加载地图参数：maps:=/home/yahboom/yahboomcar_ws/src/yahboomcar_nav/maps/testaa.yaml（可替换目标地图）

启动摄像头显示指令，终端输入，

```
#使摄像头舵机水平
ros2 run yahboom_esp32_mediapipe control_servo
#启动ESP32 摄像头
ros2 run yahboom_esp32_camera sub_img
```

手机APP显示如下图，输入小车的IP地址，【zh】表示中文，【en】表示英文；选择ROS2，下边的Video Tpoic选择：/usb_cam/image_raw/compressed，最后点击【连接】

![image-20231219145932465](image-20231219145932465.png)

成功连接上后，显示如下，

![image-20231219150550971](image-20231219150550971.png)

如下图所示，选择导航界面，

![image-20231219150703005](image-20231219150703005.png)

然后，结合小车在实际中的位姿，点击【设置初始化点】，给定小车一个初始的目标点，雷达扫描的区域与实际障碍物大致重合则表示位姿准确。如下图所示，

![image-20231219160712066](image-20231219160712066.png)

然后，点击【设置导航点】，给定小车一个目的地，小车会规划出路径并且按照路径运动到目的地。

### 4、代码解析

这里说明下开启APP导航的launch文件，

navigation_dwb_app_launch.xml

```xml
<launch>
    <include file="$(find-pkg-share rosbridge_server)/launch/rosbridge_websocket_launch.xml"/>
    <node name="laserscan_to_point_publisher" pkg="laserscan_to_point_publisher" exec="laserscan_to_point_publisher"/>
    <include file="$(find-pkg-share yahboomcar_nav)/launch/navigation_dwb_launch.py"/>
    <include file="$(find-pkg-share robot_pose_publisher_ros2)/launch/robot_pose_publisher_launch.py"/>
</launch>
```

这里运行了以下几个launch文件和节点Node：

- rosbridge_websocket_launch.xml：开启rosbridge服务相关节点，启动后，可以通过网络连接到ROS
- laserscan_to_point_publisher：把雷达的点云转换发布到APP上进行可视化
- navigation_dwb_launch.py：导航程序
- robot_pose_publisher_launch.py：小车位姿发布程序，小车位姿在APP进行可视化