# ROS_STM32F1
## 基于STM32F1平台的ROS机器人底盘控制程序


### 使用方法
1. 通过usb线直接连接到STM32F1平台的usb接口，和已经安装好ros的ubuntu系统的电脑。
//1. 通过usb转串口模块连接到STM32F1平台的USART3串口（可以在程序中配置为其他串口）和已安装好ros的Ubuntu系统的电脑。（弃用）
2. 启动终端，输入roscore 回车。
3. 另启动一个终端，输入rosrun rosserial_python serial_node.py /dev/ttyUSB0 _baud:=115200 回车 连接上后终端会打印连接成功的消息。
4. 在启动一个终端，输入rostopic list 可以获取当前的topic信息。输入rostopic echo /odom 回车 可以获取底盘上传的odom数据。

