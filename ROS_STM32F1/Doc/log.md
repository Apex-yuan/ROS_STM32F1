# ������־



-  **2019/4/3  V1.0** 
  ROS_STM32F1 �����˵��̳���������������棺
  1. ����������rosserial��
  2. ��ֲ��Turtlebot3 burger�Ĳ��ֿ��Ƴ���ͨ��topic����������Ϣ������imu��Ϣ������odom��Ϣ������joint_states��Ϣ������cmd_vel��Ϣ��
  3. ����mpu6050 dmp�⣬��ֱ�ӻ�ȡ ���ٶȼơ������ǵ����ݼ���Ԫ����Ϣ
  4. ����������������ĵײ���ƺ����͵����PID���ƺ�������֧��ros�·��ٶȣ�������ٴﵽĿ���ٶȡ�

-----

- **2019/4/4 V1.0**
    ����PID�����㷨��������2���Ѿ�ʵ��ɹ�������1��Ŀǰ���Խ����̫���ԣ�����Ҫ��һ��ʵ����֤��
  1.��������ƽ��С�����ٶȿ��ƺͷ�����ƣ������ٶȵ���ֱ�߷����ϵ��ٶȿ��ƣ��ٶ�PI���ƣ������ٶȵ���ת��ķ�����ƣ�����PD���ơ�
  2.��Ŀ�����ٶȺͽ��ٶ�ת��Ϊ�������ӷֱ���ٶ��ٶ���ʩ���ٶ�PI����

-----

- **2019/4/5 V1.0**

  1. �������������ļ��� B ����˵�����жϺ���û�д������C++�ļ����ʱ�жϺ���û�� extern ��C��

     ```c
     #ifdef __cplusplus
     extern "C" {
     #endif
      �жϷ����� 
     #ifdef __cplusplus
     }
     #endif
     ```

  2. ���ⲻ�����жϷ������з�����ԭ�������

     - ʵ�����������ⷢ�������жϺ�����ִ�У������жϵ����ڻ�䳤��ms����Ƶ�ʻ����������λ ������ʾ����ID����Ϣ��У�������

  3. TIM_TIMER_Init(uint16_t arr, uint16_t psc) �����ĳ�ʼ��λ���н�������������ڲ���TIM_Cmd(TIM1,ENABLE)Ϊʹ��״̬�����������̽����жϽ��̣�����һЩ��ֵ����⣨��������Ӳ���Ϸ��жϣ���

  -----

- **2019/4/9 V1.0**
  - topic ����Ƶ�����⣺
    1. ʹ��STM32�Դ��Ĵ���USARTͨ������ģ���ROSͨ��ʼ�մﲻ����Ҫ�ķ���Ƶ�ʣ�����Ҳ����һ�����ͻ�����Ч����û�л�����ʱЧ��һ������֪�������������˻���ķ�����Ƶ�ʡ���������֤һ��stm32����Ƶ�Է���Ƶ�ʵ�Ӱ�죩
    2. ʹ��STM32 ���⴮�ڿ������Խ�����ķ���Ƶ����ߣ���������������Ҫ�ķ���Ƶ�ʡ�
    3. �����б�Ҫ̽��һ�·���Ƶ�ʵ�������Щ����Ӱ�졣

/* 2019/4/10 V1.1 */
�������ӣ�

1. ������USB���⴮�ڵĿ⣬����ROS��STM32��ͨ����ԭ���Ĵ���ת��USB���⴮���ϣ���������˷���Ƶ�ʡ�
2.�Ż��˲��ִ��루��ҪʱUSB���⴮�ڿⲿ�֣�������USBSerial�ļ�����ros����ײ�usb���⴮�ڵ�ͨ�Žӿڣ�
3.����������װ̬����ʾ��ͨ��led�ƿ���ֱ�ӿ�����ǰ������״̬��connect: ��˸��disconnect���ر�
BUG:
1. ��Ӳ����δ��װmpu6050ģ�飬�����һֱ����mpu6050�ĳ�ʼ���ȴ����ڡ�

/* 2019/4/12  V1.2*/
1. ���ˣ�ros��STM32F1��ͨ�Ų�������ɣ�������Ŀ��������ͨ������Щ������ROSС����
2. stm32������rosʱ����� wrong checksum for topic id and msg ����Ϣ����Ҫ����Ϊͨ���ٶȲ������µģ�
   ��1����ʹ�ô����Ҳ������ͻ�����ʱ������ָ���Ϣ���������ͨ���ٶȷǳ�����
   ��2����ʹ�ô��ںͷ��ͻ�����ʱ������ָ���Ϣ�һ�һֱ���ͣ����Ӵ��ڻ������Ĵ�С��������ѭ������������ķ����ٶȣ�����delay_ms(20);�����Խ��һֱ���͵�        ���⵫�����еķ���Ƶ�ʻ��½���
   ��3����ʹ��USB���⴮�ڣ��ڽ����ͻ������Ĵ�С��Ϊ1024ʱҲ����֣���Ϣһֱ���͵���������ڽ����ͻ���������Ϊ��1024*2��ʱ�㲻����һֱ���ͣ����Խ����ն�
         ����ʾһ�еĸ���Ϣ�������Ľڵ㷢��Ƶ�ʿ��Դﵽ��
3. ���ڳ����б����˴����ϴ���usb���⴮���ϴ������ַ�ʽ����������ҵ�����ԭ��ʱ����ʹ��(Ŀǰ��Ϊ���ģ����ڷ�ʽ�������ã����ڷ�ʽֻΪ��������������ǰ���ٸ��ļ���������������)�� 
4. ROS��Ϣ�ķ����Ͷ��Ŀ��Էŵ��жϺ����н��У���������ʱ��ͬ������һ�¡�
5. �����������˷������������ݵ�ģ�庯�����������ӳ�����Ӳ��������ʹ�á�
6. ������mpu6050�ļ��к�����ȡ���ݵ����ͣ����¶������µ�IMU_Data�Ľṹ�����ͣ����������Ļ�ȡimu���ݡ�
7. Ŀǰ���еķ�������ȫ����ֲ���˶�ʱ��1�жϺ����У���δ�����������⣬���������������ٸ�Ϊ�������з�����
/******************* release V1.0 *******************************/

/**********************ros_balancecar branch*********************/
/* 2019/4/12 */
1.���������ó���ӵ�ƽ�⳵ƽ̨�ϡ�

/* 2019/4/18 */
1.�޸�led������Դ�ļ����޸�����������ʹ��led_toggle()����û�仯��bug��
2.����������λ�롰&�������ȼ�������ȡ�==�������ȼ���������led_toggle()���������������С�
if(LEDn & LED_ALL == LED_ALL)   ���Ϊ true 
 (1)��ִ��LED == LED_ALL������  ���Ϊ 1
 (2)��ִ��LEDn & 1������  �˴���LEDnΪ0x01 ���Ϊ 1
 (3)���ִ����þ��LED0���ٴη�ת�������ֺ�֮ǰ��״̬һ���ˣ����������˸��
����ʽ�޸�Ϊ��if((LEDn & LED_ALL) == LED_ALL)


/*2019/4/19*/
1. STM32�߼���ʱ����ʱʱ�䲻׼ȷ��������ʱ��ʱ����TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
   ����ϸ�ڲο���https://blog.csdn.net/qq_38087069/article/details/85029642
2.����pid�Ķ��ĺ���������ͨ����λ������pid��Ϣ��ʵ�ֶ�̬����pid������Ϣ��
��λ������������rostopic pub /pid [TAB] [TAB]    ��data: ��������ݸ�Ϊ: (�ո�)[kp,ki,kd]�ĸ�ʽ
    rostopic pub /pid std_msgs/Float32MultiArry 

/* 2019/4/22 */
1. ��ROS�˷���������ϢʱӦע��Ҫ��ǰ��������飬ֱ�ӽ�����ĵ�ַ��ֵ��ROS�˵����ݵ�ַ���磺
    ��ȷ��ʽ��
   float rpy_data[3];
   rpy_data[0] = 1;
   rpy_data[1] = 2;
   rpy_data[2] = 3;
   rpy_msg.data = rpy_data;
    ����ʽ��
    rpy_msg.data[1] = imu_data.rpy[1];
    rpy_msg.data[2] = imu_data.rpy[2]; 
/* 2019/4/24 */
1. ʵ���˷����PID���ƣ��·����ٶ�ͨ��pid����ʹƽ��С���ﵽ�ȶ����ٶ���ת��
    /* 2019/4/26 */
  1. �Ż�protocol.c�ļ����ݣ�
     ��1��ɾ��֮ǰ�����Ĳ��Դ���
     ��2���Ż��������ݽ��գ�������һ��������������Դ��4��ָ�
      (3) ��ʵ�ʵĴ���3�жϷ������ŵ�stm32f103_it.c������ά�����ڱ��ļ���ʵ���жϷ�������ԭ��usart3_irq()

    2.ƽ�⳵��ֱ��������������ڽ�С������ʱ���ͣת��
    
    ˼·1����ƽ���С�Ƕȷ�Χ�ر��ٶȿ��ƽ�����ֱ������

/* 2019/5/16 */
1. �Ż�user��Ĵ���
2. ��ԭ��sensor.c/h����Ϊvariable.c/h,���ļ����ڴ������ļ���Ҫ�õ���������Щ�ļ��ж��岻̫���ʵı�����
3. Ŀǰ��Ҫ�����·��ٶȺ�ʵ���ٶ�֮��Ĺ�ϵ��������ɣ�
4. ��ʵ���·��Ľ��ٶ���ʵ�ʽ��ٶ�һ�£����Էֱ�ͨ��imu��odom�������Ƚϣ� ������ɣ�

/* 2019/5/19 */
1. �޸ı��������ֵĴ��룬ʹ�ٶȿ��Ʋ���ת�ٲ��ֺ�ROS�Ϸ�������һ�¡�

/* 2019/6/26 */
1. ���������������ж���ִ�л���������rviz�м��ز���costmap�����⡣������ָ����odom�Ϸ����⣬���ж��еķ���Ƶ��Ҳ�������ģ�
   Ŀǰ���ɿ�����ʱ��������⡣
2. �޸ķ���������λ�ã��������������õ�whileѭ���н��С�

/* 2019/7/1 */
1.��֮ǰ��main.cpp�������뱣���Ĵ������ʱ�ȷŵ��⣺
/*���沿�����ε�����Ϊ����������õ����ȶ����̣��ò��ִ��빦������2019/4/14*/
// //Ŀǰ��PID���㺯��������һЩ���⣬�����޸ġ���2019/3/30
// //Ŀǰ�޸���������ӵ�pid����������֤�ɹ������޸��Ҳ����ӡ�2019/4/4
// //pid������֤�ɹ����ѽ��Ҳ�pid������Ϊ�����ʽ��Ŀǰ����������Ϊ static float speed_control_integral ��������ʹ����ͬ��PID�������������Ż� 2019/4/5 8.32
// float leftPIDCaculate(float goal_vel, float real_vel, float kp, float ki)
// {
//   float delta_vel;
//   float fP,fI;
//   float speed_control_output;
//   static float speed_control_integral;

//   delta_vel = goal_vel - real_vel;

//   fP = delta_vel * kp;
//   fI = delta_vel * ki;

//   speed_control_integral += fI;

//   speed_control_output = fP + speed_control_integral;
//   return speed_control_output;
// }

// float rightPIDCaculate(float goal_vel, float real_vel, float kp, float ki)
// {
//   float delta_vel;
//   float fP,fI;
//   float speed_control_output;
//   static float speed_control_integral;

//   delta_vel = goal_vel - real_vel;

//   fP = delta_vel * kp;
//   fI = delta_vel * ki;

//   speed_control_integral += fI;

//   speed_control_output = fP + speed_control_integral;
//   return speed_control_output;
// }

// float motor_output[WHEEL_NUM];
// // left motor:  gpio : PB13,PB12   PWM: PA3
// // right motor: gpio : PB14,PB15   PWM: PA2
// void motorControl(float linear_vel, float angular_vel)
// {
//   float goal_vel[WHEEL_NUM], current_vel[WHEEL_NUM];
//   //float motor_output[WHEEL_NUM];

//   //������������Ŀ���ٶ�ֵ
//   goal_vel[LEFT]  = linear_vel - (angular_vel * WHEEL_SEPARATION / 2);
//   goal_vel[RIGHT] = linear_vel + (angular_vel * WHEEL_SEPARATION / 2);

//   //�����������ӵ�ǰ�ٶ�ֵ
//   current_vel[LEFT] = last_velocity[LEFT] * WHEEL_RADIUS;
//   current_vel[RIGHT] = last_velocity[RIGHT] * WHEEL_RADIUS;

//   //���PID���㵱ǰ�ĵ�����
//   motor_output[LEFT] = leftPIDCaculate(goal_vel[LEFT], current_vel[LEFT], 1000, 800);
//   motor_output[RIGHT] = rightPIDCaculate(goal_vel[RIGHT], current_vel[RIGHT], 1000, 800);

//   if(motor_output[LEFT] > 0)
//   {
//     motor_setDirection(LEFT_MOTOR, FRONT);
//     motor_output[LEFT] = motor_output[LEFT] + LEFT_MOTOR_OUT_DEAD_ZONE; // +
//   }
//   else
//   {
//     motor_setDirection(LEFT_MOTOR, BACK);
//     motor_output[LEFT] = motor_output[LEFT] - LEFT_MOTOR_OUT_DEAD_ZONE; // +
//   }

//   if(motor_output[RIGHT] > 0)
//   {
//     motor_setDirection(RIGHT_MOTOR, FRONT);
//     motor_output[RIGHT] = motor_output[RIGHT] + RIGHT_MOTOR_OUT_DEAD_ZONE; // +
//   }
//   else
//   {
//     motor_setDirection(RIGHT_MOTOR, BACK);
//     motor_output[RIGHT] =  motor_output[RIGHT] - RIGHT_MOTOR_OUT_DEAD_ZONE; // +
//   }

//   //�·��ٶ�Ϊ��ʱ��ȷ���������ֹͣ״̬����ֹ��������Ϊ�ƶ��������Լ�������� 
//   //�ò��ּ��ϻ���ָ���ת�ķ���ʱ�����ӻ���ԭ������תһ�£�������ת��
// //  if (linear_vel == 0 && angular_vel == 0)
// //  {
// //    //����ͣת
// //    motor_setDirection(LEFT_MOTOR, STOP);
// //    //����ͣת
// //    motor_setDirection(RIGHT_MOTOR, STOP);
// //    //�������
// //    motor_output[LEFT] = motor_output[RIGHT] = 0;
// //  }

//   //�������޷�
//   motor_output[LEFT] = constrain(motor_output[LEFT], MIN_MOTOR_OUT, MAX_MOTOR_OUT);
//   motor_output[RIGHT] = constrain(motor_output[RIGHT], MIN_MOTOR_OUT, MAX_MOTOR_OUT);

//   motor_setPwm(LEFT_MOTOR, (uint16_t) abs(motor_output[LEFT]));
//   motor_setPwm(RIGHT_MOTOR, (uint16_t) abs(motor_output[RIGHT]));
// }

/**********************ros_stablecar branch*********************/
/* 2019/9/3 */
1. ������֧ros_stablecar,���ڿ����ȶ���˫�ֵ��̡�
2. ��ƽ�⳵���ֵĵ������������ε���Ȼ�������ȶ����̵ĵ��������ƴ��뼴����ɶ��ȶ����̵�Ӧ�ò��ԡ�

/* 2019/9/5 */
�������µĸ�����λ�������Ĳ���:
1. ͨ��ROS�Ŀͻ��˷�����ģ�͸��Ĳ�����
   ��1������Ƭ����ֻ��ʵ���޷���ֵ�Ļص������������Զ���Ҫʵ�ַ���ֵΪbool���͵ĺ�����
   ��2����������ʵ��ĳЩ�򵥵Ĺ���Ҳ�ǿ��еģ����ӵ��Զ˻ᱨһ��ERROR
   ʹ�÷�����
   a. ʵ�ַ���˵Ļص�����
   void pidCallback(const std_srvs::EmptyRequest &pid_req, std_srvs::EmptyResponse &pid_res)
    {
    led_toggle(LED0);
    nh.loginfo("pidCallback�ص���������");
    }
    b. �������������
    ros::ServiceServer<std_srvs::EmptyRequest, std_srvs::EmptyResponse> pid_server("pid", pidCallback);
    c. main�����е�������ķ�������
   nh.advertiseService(pid_server);

  �ڵ�Ƭ������λ������rosserial����֮�������µ��ն˵��� rosparam call /pid ���Կ�������rosserial���ն��д�ӡ����pidCallback�ص��������ԡ�����־��
2. ͨ��ROS�Ķ�̬����ʵ�ֲ����޸ĵ�Ŀ��
   ��Ƭ�����߱���̬������ʵ������

3. ͨ��ROS�Ĳ���������������λ������
ע�����⣺
   (1) ����ÿ�δ�������֮ǰ����nh.getParam("pid_p",&kp)��ȡ��������Ƚ�ռ��CPU��Դ��Ӱ���������ֵ�ִ���ٶȣ�ʵ�⣺֮ǰimu�ķ���Ƶ�ʿ��Դﵽ95hz�ڼ��ϸú����󷢲�Ƶ�ʻ��Ϊ60��hz��
�����������ԣ�
    ��1��������д�������ļ��У�.yaml�ļ���ͨ��lanunch�ļ����ظò����ļ���
    ��2����λ������ѭ����������ROS�����Ӻ󣬼����Ѿ������������������еĲ�����������δ���سɹ�ʹ��Ԥ������λ���еĲ�����
    ��3�����ò����޸ı�־λ����ʹ���˸ñ�־λ����ÿ2���ȡһ�β����������Ĳ���
    ��4���ȴ�����������ɣ����������µ����������ļ��С�

     /* ���沿�ִ��������ѭ����ִ�У�ʵ�ֵĹ����ǣ�����ROS��������֮�󣬻�ȡһ�β����������ϵĲ��� */
    static uint8_t get_parameter_flag = 1;
   
    if(nh.connected())
     {
       if(1 == get_parameter_flag)
     {
       get_parameter_flag = 0;
   
      if(!(nh.getParam("bcbot/pid/debug",&debug)))
      {
        debug = 0;
      }
      if(!(nh.getParam("bcbot/pid/p",&kp)))
      {
        kp = 0;
      }
      if(!(nh.getParam("bcbot/pid/i",&ki)))
      {
        ki = 0;
      }
      if(!(nh.getParam("bcbot/pid/d",&kd)))
      {
        kd = 0;
      }
      sprintf(aaa,"bcbot/pid/debug=%d",debug);
      nh.loginfo(aaa); 
      sprintf(aaa,"bcbot/pid/p=%d",kp);
      nh.loginfo(aaa);
      sprintf(aaa,"bcbot/pid/i=%d",ki);
      nh.loginfo(aaa);
      sprintf(aaa,"bcbot/pid/d=%d",kd);
      nh.loginfo(aaa);
     }
   } 

/* 2019/9/5 */
1.Ŀǰ�ô���ŵ��ȶ������ϻ��������ܣ����ܶ��������̫���ʣ���Ҫ��һ������
  ��1��odomУ׼
  ��2���ٶ�PID��������
  ��3����֤mpu6050���º�������whileѭ�����Ƿ������⡣

/* 2019/10/14 */
1. ֮ǰ�ٶȿ���һֱ�õ���λ��ʽPID������PID���ʺ���Ϊ������ٶȿ��ƣ��޸���pid���Ʋ��ֵĴ��룬�޸�Ϊֻ��p���ơ�
2. ��ǰ״̬�����λ���޸�dwa�ļ��ٶȲ�����������ʵ��rviz�������ơ�

- **2019/4/3  V1.0** 

  1. ɾ��֮ǰƽ�⳵������Ҫ�Ĵ����ļ���ֻ�����ȶ�������Ҫ�Ĵ����ļ������롣
  2. ����淶main.cpp�еĴ���

  