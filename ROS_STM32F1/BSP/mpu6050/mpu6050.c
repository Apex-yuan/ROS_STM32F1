
#include "mpu6050.h"
#include "i2c.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
//#include "usart1.h"
#include <math.h>

static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};
//float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
//float Pitch,Roll,Yaw;
//unsigned long sensor_timestamp;
//short gyro[3], accel[3], sensors;
//unsigned char more;
//long quat[4];


int8_t MPU_I2C_Write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
    int i;
    if (!SW_I2C_Start())
        return -1; //false;
    SW_I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!SW_I2C_WaitAck()) {
        SW_I2C_Stop();
        return -1;//false;
    }
    SW_I2C_SendByte(reg);
    SW_I2C_WaitAck();
    for (i = 0; i < len; i++) {
        SW_I2C_SendByte(data[i]);
        if (!SW_I2C_WaitAck()) {
            SW_I2C_Stop();
            return -1;//false;
        }
    }
    SW_I2C_Stop();
    return 0;//true;
}


int8_t MPU_I2C_Read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!SW_I2C_Start())
        return -1;//false;
    SW_I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!SW_I2C_WaitAck()) {
        SW_I2C_Stop();
        return -1;//false;
    }
    SW_I2C_SendByte(reg);
    SW_I2C_WaitAck();
    SW_I2C_Start();
    SW_I2C_SendByte(addr << 1 | I2C_Direction_Receiver);
    SW_I2C_WaitAck();
    while (len) {
        *buf = SW_I2C_ReceiveByte();
        if (len == 1)
            SW_I2C_NoAck();
        else
            SW_I2C_Ack();
        buf++;
        len--;
    }
    SW_I2C_Stop();
    return 0;//true;
}                                           
                 
                                           
                                           
/*add by miaowlabs*/
 /*-------------------------------------------------------------------*/
 /* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}
/*add by miaowlabs*/
unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

/*add by miaowlabs*/
uint8_t run_self_test(void)
{
    int result;
//    char test_packet[4] = {0};
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x3) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel_sens = 0; //修改原码，使重力加速度校准失效。防止小车只有在平衡位置才能开机。
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
		//printf("setting bias succesfully ......\n");
        return 0;
    }
	else
	{
		//printf("bias has not been modified ......\n");
    return 1;
	}


}
                                                                                    
 

uint8_t MPU_DMP_Init(void)
{
	uint8_t res = 0;
	SW_I2C_Init(); 	//初始化IIC总线
	if(mpu_init()==0)	//初始化MPU6050
	{	 
		res = mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL);//设置所需要的传感器
		if(res) return 1; 
		res = mpu_configure_fifo(INV_XYZ_GYRO|INV_XYZ_ACCEL);//设置FIFO
		if(res) return 2; 
		res = mpu_set_sample_rate(DEFAULT_MPU_HZ);	//设置采样率
		if(res) return 3; 
		res = dmp_load_motion_driver_firmware();		//加载dmp固件
		if(res) return 4; 
		res = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));//设置陀螺仪方向
		if(res) return 5; 
		res = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_TAP|	//设置dmp功能
		      DMP_FEATURE_ANDROID_ORIENT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|
		      DMP_FEATURE_GYRO_CAL);
		if(res) return 6; 
		res = dmp_set_fifo_rate(DEFAULT_MPU_HZ);	//设置DMP输出速率(最大不超过200Hz)
		if(res) return 7;   
		res = run_self_test();		//自检
		if(res) return 8;    
		res = mpu_set_dmp_state(1);	//使能DMP
		if(res) return 9;     
	}
  else
    return 10;
	return 0;
}


//得到dmp处理后的数据(注意,本函数需要比较多堆栈,局部变量有点多)
//rpy:欧拉角
//rpy[0]: pitch:俯仰角 精度:0.1°   范围:-90.0° <---> +90.0°
//rpy[1]: roll:横滚角  精度:0.1°   范围:-180.0°<---> +180.0°
//rpy[2]: yaw:航向角   精度:0.1°   范围:-180.0°<---> +180.0°
//gyro :角速度（rad/s）
//accel:加速度（m/s^2）
//quat :四元数（float格式）
//返回值:0,正常
//    其他,失败
//uint8_t MPU_DMP_ReadData(float *gyro, float *accel ,float *quat, float *rpy)
uint8_t MPU_DMP_ReadData(IMU_Data *imu)
{
	//float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	unsigned long sensor_timestamp;
	short sensors;
	unsigned char more;
  short gyro_raw[3]; //raw gyro 
  short accel_raw[3]; //raw accel
	long quat_raw[4]; //q30格式的四元数
	if(dmp_read_fifo(gyro_raw, accel_raw, quat_raw, &sensor_timestamp, &sensors, &more))return 1;	 
	/* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
	 * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
	**/
	if (sensors & INV_XYZ_GYRO )
  {
    for(int i = 0; i < 3; ++i)
    {
      imu->gyro[i] = gyro_raw[i] * GYRO_FACTOR;
    }
  }
  else
    return 2;
	//send_packet(PACKET_TYPE_GYRO, gyro);
	if (sensors & INV_XYZ_ACCEL)
  {
    for(int i = 0; i < 3; ++i)
    {
      imu->accel[i] = accel_raw[i] * ACCEL_FACTOR;
    }
  }
  else
    return 3;
	//send_packet(PACKET_TYPE_ACCEL, accel); 
	/* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
	 * The orientation is set by the scalar passed to dmp_set_orientation during initialization. 
	**/
	if(sensors&INV_WXYZ_QUAT) 
	{
    for(int i = 0; i < 4; i++)
    {
      imu->quat[i] = quat_raw[i] / q30;  //q30格式转换为浮点数
    }
		//计算得到欧拉角（俯仰角/横滚角/航向角）
		imu->rpy[0] = asin(-2 * imu->quat[1] * imu->quat[3] + 2 * imu->quat[0]* imu->quat[2]);	// pitch
		imu->rpy[1] = atan2(2 * imu->quat[2] * imu->quat[3] + 2 * imu->quat[0] * imu->quat[1], -2 * imu->quat[1] * imu->quat[1] - 2 * imu->quat[2] * imu->quat[2] + 1);	// roll
		imu->rpy[2] = atan2(2*(imu->quat[1] * imu->quat[2] + imu->quat[0] * imu->quat[3]), imu->quat[0] * imu->quat[0] + imu->quat[1] * imu->quat[1] - imu->quat[2] * imu->quat[2] - imu->quat[3] * imu->quat[3]);	//yaw
	}
  else 
    return 4;
	return 0;
}


////得到dmp处理后的数据(注意,本函数需要比较多堆栈,局部变量有点多)
////pitch:俯仰角 精度:0.1°   范围:-90.0° <---> +90.0°
////roll:横滚角  精度:0.1°   范围:-180.0°<---> +180.0°
////yaw:航向角   精度:0.1°   范围:-180.0°<---> +180.0°
////返回值:0,正常
////    其他,失败
//uint8_t MPU_DMP_GetData(short *gyro,short *accel ,float *pitch,float *roll,float *yaw)
//{
//	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
//	unsigned long sensor_timestamp;
//	short sensors;
//	unsigned char more;
//	long quat[4]; 
//	if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more))return 1;	 
//	/* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
//	 * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
//	**/
//	/*if (sensors & INV_XYZ_GYRO )
//	send_packet(PACKET_TYPE_GYRO, gyro);
//	if (sensors & INV_XYZ_ACCEL)
//	send_packet(PACKET_TYPE_ACCEL, accel); */
//	/* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
//	 * The orientation is set by the scalar passed to dmp_set_orientation during initialization. 
//	**/
//	if(sensors&INV_WXYZ_QUAT) 
//	{
//		q0 = quat[0] / q30;	//q30格式转换为浮点数
//		q1 = quat[1] / q30;
//		q2 = quat[2] / q30;
//		q3 = quat[3] / q30; 
//		//计算得到俯仰角/横滚角/航向角
//		*pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
//		*roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
//		*yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
//	}else return 2;
//	return 0;
//}
