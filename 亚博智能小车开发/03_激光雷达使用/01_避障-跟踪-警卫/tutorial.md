# 雷达避障、跟踪、警卫
### (1) 避障
注意：虚拟机需要与小车处在同一个局域网下，且 ROS_DOMAIN_ID 需要一致。  
功能：小车连接上代理，运行程序，小车上的雷达扫描设定范围内是否有障碍物，有障碍物则会根据障碍物的位置，自动调整速度，使其自身避开障碍物。通过动态参数调节器可以调整雷达检测的范围和避障检测的距离等参数。  
首先打开一个终端将小车连接上代理，终端输入 `sh ~/start_agent_computer.sh`  
新开一个终端，输入指令 `ros2 run yahboomcar_laser laser_Avoidance` 启动避障程序  
可以通过动态参数调节器去设置一些参数，终端输入 `ros2 run rqt_reconfigure rqt_reconfigure` ，需要点击 Refresh 刷新后才可以看到全部节点。显示的 laser_Avoidance 就是雷达避障的节点。 laser_Avoidance.py 代码见 source 文件夹。

### (2) 跟踪
注意：虚拟机需要与小车处在同一个局域网下，且 ROS_DOMAIN_ID 需要一致。  
功能：小车连接上代理，运行程序，小车上的雷达扫描设定范围内的最近的一个物体，并且根据设定的跟踪距离，调试自身的速度，与该物体保持一定的距离。通过动态参数调节器可以调整雷达检测的范围和避障检测的距离等参数。  
首先打开一个终端将小车连接上代理，终端输入 `sh ~/start_agent_computer.sh`  
新开一个终端，输入指令 `ros2 run yahboomcar_laser laser_Tracker` 启动跟踪程序  
可以通过动态参数调节器去设置一些参数，终端输入 `ros2 run rqt_reconfigure rqt_reconfigure` ，需要点击 Refresh 刷新后才可以看到全部节点。laser_Tracker 代码见 source 文件夹。  
### (3) 雷达警卫
注意：虚拟机需要与小车处在同一个局域网下，且 ROS_DOMAIN_ID 需要一致。  
功能：小车连接上代理，运行程序，小车上的雷达扫描设定范围内的最近的一个物体，并且会通过自转跟踪该物体，如果该物体靠近雷达小于设定的距离，则小车上的蜂鸣器会响以示警告。通过动态参数调节器可以调整雷达检测的范围和避障检测的距离等参数。  
首先打开一个终端将小车连接上代理，终端输入 `sh ~/start_agent_computer.sh`  
新开一个终端，输入指令 `ros2 run yahboomcar_laser laser_Warning` 启动警卫程序  
可以通过动态参数调节器去设置一些参数，终端输入 `ros2 run rqt_reconfigure rqt_reconfigure` ，需要点击 Refresh 刷新后才可以看到全部节点。laser_Warning 代码见 source 文件夹。