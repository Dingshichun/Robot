# 开发前准备
开发前的一些必要准备工作，包括安装虚拟机软件 VMware workstation player、安装虚拟机 ubuntu、WiFi 连接智能小车。
### (1) 软件安装和 MicroROS 控制板配置
1. **安装并打开虚拟机**。下载商家提供的 VM player 安装包，解压后直接安装，尽量不要自己安装其它的软件或版本，避免在打开虚拟机时出现一些奇怪的问题。然后就可以打开虚拟机，需要提前下载并解压商家提供的虚拟机镜像。    
2. **编辑配置文件**。打开虚拟机后，打开控制板上的电源开关，将小车和电脑使用 USB 数据线进行连接，并将数据线连接到虚拟机上（方法是打开虚拟机中的可移动设备，找到并连接）。  
按照自己的配置编辑虚拟机中的 config_robot.py 文件，我的配置如下，主要关注 '/dev/ttyUSB0'、 WiFi 配置和虚拟机的 IP 地址。  
ttyUSB0 是虚拟机连接的 USB 设备，在终端输入 `ls /dev/ttyUSB*` 可以查看虚拟机连接的所有 USB 设备。    
终端输入 `ifconfig` 查看虚拟机的 IP 地址，由于一些原因可能没有显示 ipv4 的地址，可能是主机打开了 WiFi ，会有影响，也能是 360、腾讯电脑管家之类的可能拦截了网桥流量，关闭 WiFi 或有影响的软件之后再尝试即可。**这里如果是将 WiFi 关闭，在获取到 IP 地址之后需要重新打开，以供 MicroROS 开发板连接**。
```python
if __name__ == '__main__':
    robot = MicroROS_Robot(port='/dev/ttyUSB0', debug=False) # port 一般不用改
    print("Rebooting Device, Please wait.")
    robot.reboot_device()

    robot.set_wifi_config("DSC1998", "987654321") # 输入主机的 WiFi 名称和密码
    robot.set_udp_config([192, 168, 1, 2], 8090) # 输入查询到的虚拟机 IP 地址
    robot.set_car_type(robot.CAR_TYPE_COMPUTER) # 小车类型，这里是虚拟机版
    # robot.set_car_type(robot.CAR_TYPE_RPI5)
    # robot.set_car_type(robot.CAR_TYPE_RISCV)
    robot.set_ros_domain_id(20)
    robot.set_ros_serial_baudrate(921600) # 串口通讯波特率
    robot.set_ros_namespace("") # 多机通讯时要设置小车名称以区分小车
```  
3. **配置参数**。短按 MicroROS控制板的复位键，在终端输入下面的命令配置机器人，若返回的数据和自己配置的一致，则配置成功。如果未连接成功，请检查确认机器人的配置参数、是否能够正常连接到局域网、代理 IP 地址和端口号是否对应。
```
sudo python3 config_rpbot.py
```
配置成功后，可以拔掉连接虚拟机和 MicroROS 控制板的 USB 数据线。  
然后**启动代理**。使用商家提供的虚拟机，在终端运行命令 `sh ~/start_agent_computer.sh` 连接代理，如果终端只输出两行内容，尝试按 MicroROS 控制板的复位键。  
4. **测试**。测试时不要停止步骤 3 中启动代理的命令，而是重新打开一个终端，输入 `ros2 node list` 查看节点列表。可以运行键盘控制小车的节点，终端输入下面命令，根据终端提示即可控制小车运动。
```ros2
ros2 run yahboomcar_ctrl yahboom_keyboard
```
**注意：每次小车关机之后，都要重新启动代理才能控制小车的运动**，即需要在一个终端运行命令 `sh ~/start_agent_computer.sh` 连接代理，然后打开另一个终端运行想要的控制命令。
5. **使用其它虚拟机连接代理**。如果不是使用商家配套的虚拟机，而是自己安装的其它的虚拟机，连接代理时需要先安装 docker 开发环境，并打开终端输入下面的命令
```
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host
microros/micro-ros-agent:humble udp4 --port 8090 -v4
```
### (2) 连接 WiFi 摄像头
1. **开启代理**   
需要确保前面的配置正确，打开小车电源，如果使用商家配套的虚拟机系统，在终端输入下面的命令，会自动连接摄像头代理，连接上代理后不要退出，保持命令运行状态。
```linux
sh ~/start_Camera_computer.sh
```
注意：如果未连接成功，请检查确认连接 ROS-wifi 图传模块，是否能够正常连接到局域网，代理 IP 地址是否对应。
如果使用第三方虚拟机系统，则需要先安装 docker 开发环境，并打开终端输入：
```
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 9999 -v4
```
2. **测试 ROS 节点**  
打开新终端，输入 `ros2 node list` 命令查看 /espRos/Esp32Node 节点名称。如果搜索不到 /espRos/Esp32Node 节点，请检查确认虚拟机上 .bashrc 文件的 ROS DOMAIN ID 必须是 20 才行。micros 小车的 ROS DOMAIN ID 也要是 20，否则小车无法进行 AI 视觉的玩法。  
3. **查看摄像头画面**  
打开新终端，执行下面命令就会显示摄像头画面
```
ros2 run yahboom_esp32_camera sub_img
```
如果摄像头朝下，不在中间（也就是没有和水平面呈几乎 90° 的状态），打开新的终端，输入下面命令。(此步骤需要虚拟机连接小车的代理，即打开新终端运行命令 `sh ~/start_agent_computer.sh`)
```
ros2 run yahboom_esp32_mediapipe control_servo
```
等待摄像头动作到中间，按 "Ctrl+C" 终止程序。
如果摄像头舵机动了，但没有恢复到中间，需要保持上面的程序运行，然后在不断电的情况下，重新安装舵机，使其处于中间位置。重新安装完成后，按 Crtl+C 来终止程序，结束舵机校准。  
如果摄像头出现倒置的情况，重新打开一个新的终端，输入命令
```
python3 ~/SET_Camera.py
```
然后输入一下连接 ROS-wifi 图传模块的 IP 地址，可以到连接 ROS-wifi 图传模块代理的终端查看，最后按回车，图像就还原了。

### (3) 主要命令
```
sudo python3 config_rpbot.py # 运行配置文件
sh ~/start_agent_computer.sh # 启动小车代理
sh ~/start_Camera_computer.sh # 启动 WiFi 摄像头代理

# 注意，需要在上面的代理运行的情况下才能正常打开节点
ros2 run yahboomcar_ctrl yahboom_keyboard # 打开键盘控制小车的节点
ros2 run yahboom_esp32_camera sub_img # 打开 WiFi 摄像头
```