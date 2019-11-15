# 工程日志



-  **2019/4/3  V1.0** 
  ROS_STM32F1 机器人底盘程序组件基本完整版：
  1. 包含完整的rosserial库
  2. 移植了Turtlebot3 burger的部分控制程序（通过topic发布订阅消息：发布imu消息，发布odom消息，发布joint_states消息，订阅cmd_vel消息）
  3. 包含mpu6050 dmp库，可直接获取 加速度计、陀螺仪的数据及四元数信息
  4. 包含编码器及电机的底层控制函数和电机的PID控制函数，可支持ros下发速度，电机快速达到目标速度。

-----

- **2019/4/4 V1.0**
    轮子PID控制算法：（方法2：已经实验成功，方法1：目前测试结果不太可以，还需要进一步实验验证）
  1.（类似于平衡小车的速度控制和方向控制）将线速度当作直线方向上的速度控制：速度PI控制，将角速度当作转向的方向控制：方向PD控制。
  2.将目标线速度和角速度转化为两个轮子分别的速度再独立施加速度PI控制

-----

- **2019/4/5 V1.0**

  1. 程序卡死在启动文件的 B 处，说明有中断函数没有处理或在C++文件编程时中断函数没加 extern “C”

     ```c
     #ifdef __cplusplus
     extern "C" {
     #endif
      中断服务函数 
     #ifdef __cplusplus
     }
     #endif
     ```

  2. 话题不能在中断服务函数中发布（原因不清楚）

     - 实验结果：将话题发布放入中断函数中执行，整个中断的周期会变长（ms计数频率会变慢），上位 机端显示话题ID和消息的校验码错误。

  3. TIM_TIMER_Init(uint16_t arr, uint16_t psc) 函数的初始化位置有讲究，如果函数内部的TIM_Cmd(TIM1,ENABLE)为使能状态，函数会立刻进入中断进程，导致一些奇怪的问题（程序死在硬件上访中断）。

  -----

- **2019/4/9 V1.0**
  - topic 发布频率问题：
    1. 使用STM32自带的串口USART通过串口模块和ROS通信始终达不到想要的发布频率，发送也建立一个发送缓冲区效果和没有缓冲区时效果一样，不知道是哪里限制了话题的发布的频率。（后面验证一下stm32的主频对发布频率的影响）
    2. 使用STM32 虚拟串口可以明显将话题的发布频率提高，基本可以满足需要的发布频率。
    3. 后续有必要探究一下发布频率到底由那些因素影响。

/* 2019/4/10 V1.1 */
性能增加：

1. 增加了USB虚拟串口的库，并将ROS与STM32的通信由原来的串口转到USB虚拟串口上，明显提高了发布频率。
2.优化了部分代码（主要时USB虚拟串口库部分，增加了USBSerial文件建立ros层与底层usb虚拟串口的通信接口）
3.增加了连接装态的显示：通过led灯可以直接看到当前的连接状态，connect: 闪烁，disconnect：关闭
BUG:
1. 若硬件上未安装mpu6050模块，程序会一直卡在mpu6050的初始化等待环节。

/* 2019/4/12  V1.2*/
1. 至此，ros与STM32F1的通信层基本搭建完成，接下来目标放在如何通过，这些组件搭建出ROS小车。
2. stm32连接上ros时会出现 wrong checksum for topic id and msg 的信息，主要是因为通信速度不够导致的：
   （1）：使用串口且不带发送缓冲区时不会出现该信息，但整体的通信速度非常慢。
   （2）：使用串口和发送缓冲区时，会出现该信息且会一直发送，增加串口缓冲区的大小，且在主循环中限制整体的发布速度（加上delay_ms(20);）可以解决一直发送的        问题但是所有的发布频率会下降。
   （3）：使用USB虚拟串口，在将发送缓冲区的大小设为1024时也会出现，信息一直发送的情况，但在将发送缓冲区设置为（1024*2）时便不会再一直发送，但仍将在终端
         中显示一行的该信息。基本的节点发布频率可以达到。
3. 现在程序中保留了串口上传和usb虚拟串口上传的两种方式，方便后面找到真正原因时测试使用(目前因为更改，串口方式不可以用，串口方式只为后续做保留，当前不再更改及调整其的适用情况)。 
4. ROS消息的发布和订阅可以放到中断函数中进行，接下来将时间同步做好一下。
5. 程序中增加了发布超声波数据的模板函数，可在增加超声波硬件驱动后，使用。
6. 更新了mpu6050文件中函数获取数据的类型，重新定义了新的IMU_Data的结构体类型，用以完整的获取imu数据。
7. 目前所有的发布函数全部移植到了定时器1中断函数中，尚未发现明显问题，先做测试有问题再改为主函数中发布。
/******************* release V1.0 *******************************/

/**********************ros_balancecar branch*********************/
/* 2019/4/12 */
1.接下来将该程序加到平衡车平台上。

/* 2019/4/18 */
1.修改led的驱动源文件，修复了主程序中使用led_toggle()函数没变化的bug。
2.操作符：按位与“&”的优先级低于相等“==”的优先级，导致了led_toggle()函数不能正常运行。
if(LEDn & LED_ALL == LED_ALL)   结果为 true 
 (1)先执行LED == LED_ALL的运算  结果为 1
 (2)再执行LEDn & 1的运算  此处的LEDn为0x01 结果为 1
 (3)因此执行完该句后LED0会再次反转这样就又和之前的状态一致了，因而不会闪烁。
将上式修改为：if((LEDn & LED_ALL) == LED_ALL)


/*2019/4/19*/
1. STM32高级定时器定时时间不准确，在配置时基时增加TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
   具体细节参考：https://blog.csdn.net/qq_38087069/article/details/85029642
2.增加pid的订阅函数，可以通过上位机发布pid消息，实现动态调整pid参数信息。
上位机发布方法：rostopic pub /pid [TAB] [TAB]    将data: 后面的数据改为: (空格)[kp,ki,kd]的格式
    rostopic pub /pid std_msgs/Float32MultiArry 

/* 2019/4/22 */
1. 从ROS端发布数组消息时应注意要提前定义好数组，直接将数组的地址赋值给ROS端的数据地址。如：
    正确方式：
   float rpy_data[3];
   rpy_data[0] = 1;
   rpy_data[1] = 2;
   rpy_data[2] = 3;
   rpy_msg.data = rpy_data;
    错误方式：
    rpy_msg.data[1] = imu_data.rpy[1];
    rpy_msg.data[2] = imu_data.rpy[2]; 
/* 2019/4/24 */
1. 实现了方向的PID控制：下发角速度通过pid调节使平衡小车达到稳定角速度旋转。
    /* 2019/4/26 */
  1. 优化protocol.c文件内容：
     （1）删除之前保留的测试代码
     （2）优化串口数据接收：定义了一个命令缓冲区，可以存放4条指令。
      (3) 将实际的串口3中断服务函数放到stm32f103_it.c函数中维护，在本文件中实现中断服务函数的原型usart3_irq()

    2.平衡车的直立控制如何做到在将小车提起时电机停转？
    
    思路1：在平衡的小角度范围关闭速度控制仅保留直立控制

/* 2019/5/16 */
1. 优化user层的代码
2. 将原来sensor.c/h改名为variable.c/h,该文件用于存放许多文件需要用到的且在哪些文件中定义不太合适的变量。
3. 目前需要测试下发速度和实际速度之间的关系。（待完成）
4. 让实际下发的角速度与实际角速度一致（可以分别通过imu和odom计算来比较） （待完成）

/* 2019/5/19 */
1. 修改编码器部分的代码，使速度控制测量转速部分和ROS上发部分相一致。

/* 2019/6/26 */
1. 将发布函数放在中断中执行会遇到，在rviz中加载不出costmap的问题。（问题指向到了odom上发问题，但中断中的发布频率也是正常的）
   目前怀疑可能是时间戳的问题。
2. 修改发布函数的位置，将发布函数放置到while循环中进行。

/* 2019/7/1 */
1.将之前在main.cpp函数中想保留的代码块暂时先放到这：
/*下面部分屏蔽掉是因为下面代码适用的是稳定底盘，该部分代码功能正常2019/4/14*/
// //目前的PID计算函数还存在一些问题，后续修改。　2019/3/30
// //目前修改了左侧轮子的pid函数，待验证成功后在修改右侧轮子。2019/4/4
// //pid函数验证成功，已将右侧pid函数改为左侧样式，目前左右轮子因为 static float speed_control_integral 变量不能使用相同的PID函数，后续再优化 2019/4/5 8.32
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

//   //计算左右轮子目标速度值
//   goal_vel[LEFT]  = linear_vel - (angular_vel * WHEEL_SEPARATION / 2);
//   goal_vel[RIGHT] = linear_vel + (angular_vel * WHEEL_SEPARATION / 2);

//   //计算左右轮子当前速度值
//   current_vel[LEFT] = last_velocity[LEFT] * WHEEL_RADIUS;
//   current_vel[RIGHT] = last_velocity[RIGHT] * WHEEL_RADIUS;

//   //结合PID计算当前的电机输出
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

//   //下发速度为零时，确保电机处于停止状态，防止机器人因为推动，出现自己动的情况 
//   //该部分加上会出现更换转的方向时，轮子会沿原来方向转一下，再往回转。
// //  if (linear_vel == 0 && angular_vel == 0)
// //  {
// //    //左轮停转
// //    motor_setDirection(LEFT_MOTOR, STOP);
// //    //右轮停转
// //    motor_setDirection(RIGHT_MOTOR, STOP);
// //    //输出清零
// //    motor_output[LEFT] = motor_output[RIGHT] = 0;
// //  }

//   //电机输出限幅
//   motor_output[LEFT] = constrain(motor_output[LEFT], MIN_MOTOR_OUT, MAX_MOTOR_OUT);
//   motor_output[RIGHT] = constrain(motor_output[RIGHT], MIN_MOTOR_OUT, MAX_MOTOR_OUT);

//   motor_setPwm(LEFT_MOTOR, (uint16_t) abs(motor_output[LEFT]));
//   motor_setPwm(RIGHT_MOTOR, (uint16_t) abs(motor_output[RIGHT]));
// }

/**********************ros_stablecar branch*********************/
/* 2019/9/3 */
1. 新增分支ros_stablecar,用于控制稳定的双轮底盘。
2. 将平衡车部分的电机输出控制屏蔽掉，然后增加稳定地盘的电机输出控制代码即可完成对稳定地盘的应用测试。

/* 2019/9/5 */
发现了新的更改下位机参数的策略:
1. 通过ROS的客户端服务器模型更改参数：
   （1）：单片机端只能实现无返回值的回调函数，而电脑端需要实现返回值为bool类型的函数。
   （2）：若单纯实现某些简单的功能也是可行的，但从电脑端会报一个ERROR
   使用方法：
   a. 实现服务端的回调函数
   void pidCallback(const std_srvs::EmptyRequest &pid_req, std_srvs::EmptyResponse &pid_res)
    {
    led_toggle(LED0);
    nh.loginfo("pidCallback回调函数测试");
    }
    b. 定义服务器对象
    ros::ServiceServer<std_srvs::EmptyRequest, std_srvs::EmptyResponse> pid_server("pid", pidCallback);
    c. main函数中调用下面的发布函数
   nh.advertiseService(pid_server);

  在单片机和上位机建立rosserial连接之后，启用新的终端调用 rosparam call /pid 可以看到启动rosserial的终端中打印出“pidCallback回调函数测试”的日志。
2. 通过ROS的动态参数实现参数修改的目的
   单片机不具备动态参数的实现条件

3. 通过ROS的参数服务器更改下位机参数
注意问题：
   (1) 若在每次处理数据之前都用nh.getParam("pid_p",&kp)获取参数，会比较占用CPU资源，影响其他部分的执行速度（实测：之前imu的发布频率可以达到95hz在加上该函数后发布频率会变为60多hz）
参数整定策略：
    （1）将参数写到配置文件中（.yaml文件）通过lanunch文件加载该参数文件。
    （2）下位机在主循环建立了与ROS的连接后，加载已经发布到参数服务器中的参数，若参数未加载成功使用预设在下位机中的参数。
    （3）设置参数修改标志位，若使能了该标志位，则每2秒获取一次参数服务器的参数
    （4）等待参数更改完成，将参数更新到参数配置文件中。

     /* 下面部分代码放在主循环中执行，实现的功能是：在与ROS建立连接之后，获取一次参数服务器上的参数 */
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
1.目前该代码放到稳定底盘上基本可以跑，但很多参数还不太合适，需要进一步调参
  （1）odom校准
  （2）速度PID参数调整
  （3）验证mpu6050更新函数放在while循环中是否有问题。

/* 2019/10/14 */
1. 之前速度控制一直用的是位置式PID，这种PID不适合作为电机的速度控制，修改了pid控制部分的代码，修改为只有p控制。
2. 当前状态配合上位机修改dwa的加速度参数基本可以实现rviz导航控制。

- **2019/4/3  V1.0** 

  1. 删除之前平衡车部分需要的代码文件，只保留稳定底盘需要的代码文件及代码。
  2. 整理规范main.cpp中的代码

  