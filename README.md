# ROS_STM32F1

## master branch

==**����STM32F1ƽ̨����ROS��STM32֮���ͨ��ģ��**==

## ros_balancecar branch

==**����STM32F1ƽ̨��ROS������ƽ����̿��Ƴ���**==

## ros_stablecar branch

==**����STM32F1ƽ̨��ROS�������ȶ����̿��Ƴ���**==


### ʹ�÷���
1. ͨ��usb��ֱ�����ӵ�STM32F1ƽ̨��usb�ӿڣ����Ѿ���װ��ros��ubuntuϵͳ�ĵ��ԡ�
    ~~//1. ͨ��usbת����ģ�����ӵ�STM32F1ƽ̨��USART3���ڣ������ڳ���������Ϊ�������ڣ����Ѱ�װ��ros��Ubuntuϵͳ�ĵ��ԡ�~~

2. �����նˣ�����roscore �س���

3. ~~������һ���նˣ�����rosrun rosserial_python serial_node.py /dev/ttyUSB0 _baud:=115200 �س� �����Ϻ��ն˻��ӡ���ӳɹ�����Ϣ��~~

   ������һ���նˣ�����rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200 �س� �����Ϻ��ն˻��ӡ���ӳɹ�����Ϣ��

4. ������һ���նˣ�����rostopic list ���Ի�ȡ��ǰ��topic��Ϣ������rostopic echo /odom �س� ���Ի�ȡ�����ϴ���odom���ݡ�

### ע�⣺

1. ��ǰ���򲻽�mpu6050Ӳ�����޷���ɲ��Եģ�mpu6050Ŀǰ�Ƿ���whileѭ���н��г�ʼ���ġ�

