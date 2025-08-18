## ROS Robot App建图

注：虚拟机需要与小车处在同一个局域网下，且ROS_DOMAIN_ID，需要一致，可以查看【使用前必看】来设置板子上的IP和ROS_DOMAIN_ID。

### 1、程序功能说明

小车连接上代理，运行程序，打开手机上下载的【ROS Robot】app，输入小车的IP地址，选择ROS2，点击连接，即可连接上小车。通过滑动界面的轮盘可以控制小车，缓慢控制小车走完建图的区域，最后点击保存地图，小车会保存当前建好的地图。

### 2、**启动并连接代理**

以配套虚拟机为例，输入以下指令启动代理（代理启动一次不关闭即可，无需重复启动），

```shell
#小车代理
sudo docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8090 -v4
#摄像头代理（先启动代理再打开小车开关）
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 9999 -v4
```

![image-20240102120120341](image-20240102120120341.png)

然后，打开小车开关，等待小车连接上代理，连接成功如下图所示，

![image-20231219100456522](image-20231219100456522.png)

### 3、启动程序

首先启动小车处理底层数据程序，终端输入，

```shell
ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py
```

![image-20231219101637748](image-20231219101637748.png)

启动APP建图命令，终端输入，

```shell
#以下建图二选一
ros2 launch yahboomcar_nav map_gmapping_app_launch.xml
ros2 launch yahboomcar_nav map_cartographer_app_launch.xml
#使摄像头舵机水平
ros2 run yahboom_esp32_mediapipe control_servo
#启动ESP32 摄像头
ros2 run yahboom_esp32_camera sub_img
```

手机APP显示如下图，输入小车的IP地址，【zh】表示中文，【en】表示英文；选择ROS2，下边的Video Tpoic选择/usb_cam/image_raw/compressed，最后点击【连接】

![image-20231219143224710](image-20231219143224710.png)

成功连接上后，显示如下，

![image-20231219143427091](image-20231219143427091.png)

通过滑动轮盘控制小车缓慢移动走完需要建图的区域，然后点击保存地图，输入地图名字点击提交，即可保存地图

![image-20231219143658951](image-20231219143658951.png)

地图保存的位置是，

```
/home/yahboom/yahboomcar_ws/src/yahboomcar_nav/maps
```

![image-20231219143847102](image-20231219143847102.png)

## 4、代码解析

这里说明下开启APP建图的launch文件，以gmapping建图为例，

map_gmapping_app_launch.xml

```xml
<launch>
    <include file="$(find-pkg-share rosbridge_server)/launch/rosbridge_websocket_launch.xml"/>
    <node name="laserscan_to_point_publisher" pkg="laserscan_to_point_publisher" exec="laserscan_to_point_publisher"/>
    <include file="$(find-pkg-share yahboomcar_nav)/launch/map_gmapping_launch.py"/>
    <include file="$(find-pkg-share robot_pose_publisher_ros2)/launch/robot_pose_publisher_launch.py"/>
    <include file="$(find-pkg-share yahboom_app_save_map)/yahboom_app_save_map.launch.py"/>
</launch>
```

这里运行了以下几个launch文件和节点Node：

- rosbridge_websocket_launch.xml：开启rosbridge服务相关节点，启动后，可以通过网络连接到ROS
- laserscan_to_point_publisher：把雷达的点云转换发布到APP上进行可视化
- map_gmapping_launch.py：gmapping建图程序
- robot_pose_publisher_launch.py：小车位姿发布程序，小车位姿在APP进行可视化
- yahboom_app_save_map.launch.py：保存地图的程序

