# UM982 Driver for ROS 1 Noetic

这是 Unicore UM982 双天线 RTK GNSS 接收机的 ROS 1 Noetic 驱动。节点从串口读取接收机输出的 PVTSLN 文本消息，解析位置、航向和定位状态，然后发布标准 ROS 消息。

本工程已从 ROS 2 版本改为 ROS 1 catkin + roscpp，适用于 Ubuntu 20.04 / ROS Noetic。

## 1. 工程目录

    ~/um982_for_ros1/um982_driver_ws/
    ├── src/um982-driver/unicore_um982_driver/
    │   ├── config/unicore_driver_params.yaml
    │   ├── launch/unicore.launch
    │   ├── src/unicore_um982_driver_node.cpp
    │   ├── src/pvtsln_parser.cpp
    │   ├── include/.../pvtsln_data.hpp
    │   └── README.md
    └── src/tools/99-serial.rules

config/unicore_driver_params.yaml 保存串口参数和 UM982 初始化命令，src/tools/99-serial.rules 用于把 USB 串口固定为 /dev/ttyGPS。

## 2. 运行环境和依赖

推荐环境：

- Ubuntu 20.04
- ROS 1 Noetic
- C++14 编译器（Ubuntu 20.04 默认 GCC 9）
- UM982 接收机和 USB 转串口设备

安装编译和运行依赖：

    sudo apt update
    sudo apt install -y \
      build-essential cmake git udev \
      ros-noetic-ros-base \
      ros-noetic-catkin \
      ros-noetic-roscpp \
      ros-noetic-sensor-msgs \
      ros-noetic-geometry-msgs \
      ros-noetic-diagnostic-msgs \
      ros-noetic-diagnostic-updater \
      ros-noetic-serial

验证 ROS 版本：

    source /opt/ros/noetic/setup.bash
    rosversion -d

输出应为 noetic。

## 3. 编译工程

    cd ~/um982_for_ros1/um982_driver_ws
    source /opt/ros/noetic/setup.bash
    catkin_make
    source devel/setup.bash

编译成功时会看到：

    [100%] Built target unicore_um982_driver_node

可选：让每次打开终端都自动加载环境：

    echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
    echo 'source ~/um982_for_ros1/um982_driver_ws/devel/setup.bash' >> ~/.bashrc
    source ~/.bashrc

## 4. 查找实际串口设备

先不要启动驱动，插拔 UM982 的 USB 线：

    ls -l /dev/ttyUSB* /dev/ttyACM* /dev/ttyGPS 2>/dev/null

查看插拔日志：

    dmesg --follow

常见设备名是 /dev/ttyUSB0。查看 USB 厂商和产品 ID：

    udevadm info -q property -n /dev/ttyUSB0 | grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL'

当前工程自带的规则针对 CH340 USB 转串口芯片：

    idVendor=1a86
    idProduct=7523

如果你的转换器 ID 不同，需要按查询结果修改规则文件。

## 5. 固定设备名称为 /dev/ttyGPS

固定名称后，USB 设备重新插拔时仍使用 /dev/ttyGPS，不会因为 ttyUSB0 变成 ttyUSB1 而改变配置。

    cd ~/um982_for_ros1/um982_driver_ws
    sudo cp src/tools/99-serial.rules /etc/udev/rules.d/99-serial.rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger

拔出并重新插入设备，然后确认：

    ls -l /dev/ttyGPS
    readlink -f /dev/ttyGPS

如果没有 /dev/ttyGPS，检查 idVendor 和 idProduct 是否与规则一致。规则模板：

    KERNEL=="ttyUSB*", ATTRS{idVendor}=="实际厂商ID", ATTRS{idProduct}=="实际产品ID", MODE:="0666", SYMLINK+="ttyGPS"

## 6. 串口权限

检查当前用户所属用户组：

    groups

如果没有 dialout：

    sudo usermod -aG dialout "$USER"

注销并重新登录后检查：

    ls -l /dev/ttyGPS

当前规则使用 MODE 0666，所有用户都可以读写该设备。如果希望权限更严格，可以改为 MODE 0660、GROUP dialout，然后重新加载 udev 规则。

## 7. 修改串口设备名和波特率

### 临时修改

使用实际设备 /dev/ttyUSB0：

    roslaunch unicore_um982_driver unicore.launch port:=/dev/ttyUSB0 baudrate:=115200

使用固定别名：

    roslaunch unicore_um982_driver unicore.launch port:=/dev/ttyGPS baudrate:=115200

### 永久修改

编辑配置文件：

    nano ~/um982_for_ros1/um982_driver_ws/src/um982-driver/unicore_um982_driver/config/unicore_driver_params.yaml

修改：

    port: "/dev/ttyGPS"
    baudrate: 115200

例如：

    port: "/dev/ttyUSB0"
    baudrate: 230400

启动命令中的 port:= 和 baudrate:= 会覆盖配置文件中的值。

注意：接收机实际波特率必须一致。配置文件中的 CONFIG COM3 115200 也会在节点启动时发送给 UM982；如果改波特率，应同步检查并修改该初始化命令。

## 8. 直接读取串口原始数据

读取之前必须停止 ROS 驱动，因为同一个串口不能被多个程序同时打开。

以 115200、8N1 为例设置串口：

    sudo stty -F /dev/ttyGPS 115200 cs8 -cstopb -parenb -ixon -ixoff -icanon

读取 10 秒原始数据：

    timeout 10s cat /dev/ttyGPS

也可以使用 screen：

    sudo apt install -y screen
    screen /dev/ttyGPS 115200

退出 screen：按 Ctrl+A，再按 K，确认退出。

正常情况下应看到以 #PVTSLN 开头的消息。完全没有输出时，优先检查串口名、波特率、USB 线和 UM982 输出配置。

## 9. 启动驱动

终端一启动 ROS Master：

    source /opt/ros/noetic/setup.bash
    roscore

终端二启动驱动：

    source /opt/ros/noetic/setup.bash
    source ~/um982_for_ros1/um982_driver_ws/devel/setup.bash
    roslaunch unicore_um982_driver unicore.launch

启动时节点会：

1. 打开配置中的串口；
2. 发送 config/unicore_driver_params.yaml 中的 UM982 初始化命令；
3. 读取并解析 PVTSLN；
4. 发布位置、航向和诊断消息。

没有连接真实 UM982 时不要启动节点，因为节点会尝试打开串口并发送初始化命令。

## 10. 查看 ROS 输出

    rosnode list
    rosnode info /unicore_um982_driver
    rostopic list
    rostopic hz /gps/fix
    rostopic echo /gps/fix
    rostopic echo /gps/imu
    rostopic echo /diagnostics

发布的话题：

| 话题 | 消息类型 | 内容 |
|---|---|---|
| /gps/fix | sensor_msgs/NavSatFix | 纬度、经度、高程、定位状态和协方差 |
| /gps/imu | sensor_msgs/Imu | 根据航向角生成的 Z 轴姿态 |
| /diagnostics | diagnostic_msgs/DiagnosticArray | 数据新鲜度、定位状态和卫星数 |

## 11. 未接天线时的启动验证

UM982 没有接天线时，驱动仍然应该能够打开串口、接收并解析 PVTSLN 消息。没有天线不会导致 ROS 节点无法启动；常见现象是：

- 终端持续打印 `Parsed OK`；
- `/gps/fix` 持续有消息；
- latitude、longitude、altitude 可能为 0；
- NavSatFix 的 status 为 `STATUS_NO_FIX`（数值通常为 -1）；
- 卫星数为 0；
- diagnostics 中会显示未定位或无效定位状态。

可以用以下命令确认数据链路：

    rostopic hz /gps/fix
    rostopic echo /gps/fix
    rostopic echo /diagnostics

如果消息持续发布、Data age 较小、原始 PVTSLN 能被解析，说明串口打开、数据读取、解析和 ROS 发布链路已经正常。没有天线时不能用经纬度是否为 0 来判断程序失败。

接上天线并等待接收机完成定位后，验收标准是：

- latitude 和 longitude 变为有效的非零值；
- Satellite count 大于 0；
- 状态变为 `SINGLE`、`RTK_FLOAT` 或 `RTK_FIXED`；
- diagnostics 不再报告 `NO_DATA` 或未定位状态。

接收机在无天线时可能返回 `NONE` 或 `NO_SOLUTION`。当前驱动会把这类未知状态显示为 `Unknown GPS status`，这表示“没有定位”，不表示串口或节点崩溃。后续可以将这些状态单独归类为未定位警告。

## 12. 常见问题

### Failed to open serial port 或 No such file or directory

    ls -l /dev/ttyGPS /dev/ttyUSB0
    roslaunch unicore_um982_driver unicore.launch port:=/dev/ttyUSB0

### Permission denied

确认用户在 dialout 组，并重新登录：

    groups
    sudo usermod -aG dialout "$USER"

### 串口能打开但没有 /gps/fix

依次检查：

1. baudrate 是否和 UM982 一致；
2. 停止驱动后，cat /dev/ttyGPS 是否能看到 PVTSLN；
3. UM982 是否启用了 PVTSLNA 输出；
4. 是否有其他程序占用串口：

       sudo lsof /dev/ttyGPS

不要同时运行 screen、cat、NTRIP 客户端或其他串口程序。

### 编译时找不到 ROS 头文件或包

    source /opt/ros/noetic/setup.bash
    cd ~/um982_for_ros1/um982_driver_ws
    catkin_make

### 诊断显示 NO_DATA

这表示节点超过 5 秒没有解析到有效的 PVTSLN 消息。停止驱动后，用第 8 节命令读取原始串口，确认 UM982 确实输出正确格式的数据。

## 13. NTRIP 注意事项

当前 ROS 1 版本负责串口读取、PVTSLN 解析和 ROS 话题发布。NTRIP/RTKLIB 不由默认 launch 文件自动启动。若使用 str2str，必须规划好串口读写方式，避免驱动节点和 str2str 同时独占同一个串口，否则会出现数据冲突或串口打开失败。

## 14. 当前适配范围

- ROS 2 ament_cmake 已替换为 ROS 1 catkin；
- rclcpp 已替换为 roscpp；
- ROS 2 serial_driver 已替换为 ROS 1 serial；
- ROS 2 参数、定时器、日志和发布器已替换为 ROS 1 API；
- PVTSLN 解析逻辑保持独立；
- 已通过 catkin_make 编译验证。

