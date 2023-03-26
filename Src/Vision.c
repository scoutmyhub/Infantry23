#include "referee_lib.h"
#include "Vision_Task.h"
#include "kalman.h"
#include "referee_lib.h"
#include "Vision.h"
#include "bsp_usart.h"
#include "stdlib.h"
#include "INS_task.h"
#include "usart.h"
#include "cmsis_os.h"
#include "math.h"

FloatUChar_t Qart[4];
FloatUChar_t Gyro[3];
FloatUChar_t Accel[3];
Vision_send_t Vision_send;
TX TX_0, TX_1, TX_2, TX_3;
TX TX_gyro1, TX_gyro2, TX_gyro3;
TX TX_acc1, TX_acc2, TX_acc3;
TX TX_yaw_ecd, TX_pitch_ecd;
extKalman_t kalman_Yaw, kalman_Pitch;
float kalman_targetYaw, kalman_targetPitch;
float targetYaw, targetPitch;
float YAW_R = 1.0, PITCH_R = 1.0, YAW_Q = 1.0, PITCH_Q = 1.0;
void vision_send(void);

extern bmi088_real_data_t bmi088_real_data;
extern fp32 gyro[3], accel[3], temp;
extern fp32 INS_quat[4];
VisionRecvData_t VisionRecvData;

// extern uint8_t Vision_Data[Vision_DataLength];
uint8_t Follow_Vision = 0;
float myyaw, mypitch, myroll;



uint8_t sv_buff[26];
FloatUChar_t Qart[4];
FloatUChar_t Gyro[3];
FloatUChar_t Accel[3];

extern fp32 gyro[3], accel[3], temp;
extern fp32 INS_quat[4];

void vision_send(void)
{
	uint8_t length = 26;
	uint8_t *x_delta, *y_delta;

	Vision_send.frame_head = 0xD4;
	Vision_send.a = 1;
	Vision_send.b = 0;
	Vision_send.c = 0;
	Vision_send.state = 0;
	Vision_send.mark = 28;
	Vision_send.anti_top = 0;
	Vision_send.color = 1;
	Vision_send.delta_x = 32;
	Vision_send.delta_y = 0;
	Vision_send.frame_tail = 0xD5;

	x_delta = (uint8_t *)&Vision_send.delta_x;
	y_delta = (uint8_t *)&Vision_send.delta_y;
	TX_yaw_ecd.f_data = (float)(motor_chassis[4].ecd);
	TX_pitch_ecd.f_data = (float)(motor_chassis[5].ecd);
	TX_0.f_data = (float)INS_quat[0];
	TX_1.f_data = (float)INS_quat[1];
	TX_2.f_data = (float)INS_quat[2];
	TX_3.f_data = (float)INS_quat[3];
	TX_gyro1.f_data = (float)bmi088_real_data.gyro[0];
	TX_gyro2.f_data = (float)bmi088_real_data.gyro[1];
	TX_gyro3.f_data = (float)bmi088_real_data.gyro[2];
	TX_acc1.f_data = (float)bmi088_real_data.accel[0];
	TX_acc2.f_data = (float)bmi088_real_data.accel[1];
	TX_acc3.f_data = (float)bmi088_real_data.accel[2];

	sv_buff[0] = Vision_send.frame_head;
	sv_buff[1] = Vision_send.a;
	sv_buff[2] = Vision_send.b;
	sv_buff[3] = Vision_send.c;
	sv_buff[4] = *TX_yaw_ecd.byte;
	sv_buff[5] = *(TX_yaw_ecd.byte + 1);
	sv_buff[6] = *(TX_yaw_ecd.byte + 2);
	sv_buff[7] = *(TX_yaw_ecd.byte + 3);
	sv_buff[8] = *TX_pitch_ecd.byte;
	sv_buff[9] = *(TX_pitch_ecd.byte + 1);
	sv_buff[10] = *(TX_pitch_ecd.byte + 2);
	sv_buff[11] = *(TX_pitch_ecd.byte + 3);
	sv_buff[12] = Vision_send.state;
	sv_buff[13] = Vision_send.mark;
	sv_buff[14] = Vision_send.anti_top;
	sv_buff[15] = Vision_send.color;
	sv_buff[16] = *x_delta;
	sv_buff[17] = *(x_delta + 1);
	sv_buff[18] = *(x_delta + 2);
	sv_buff[19] = *(x_delta + 3);
	sv_buff[20] = *y_delta;
	sv_buff[21] = *(y_delta + 1);
	sv_buff[22] = *(y_delta + 2);
	sv_buff[23] = *(y_delta + 3);
	sv_buff[24] = Vision_send.shoot;
	sv_buff[25] = Vision_send.frame_tail;

	//    sv_buff[24]=*TX_0.byte;
	//	  sv_buff[25]=*(TX_0.byte+1);
	//	  sv_buff[26]=*(TX_0.byte+2);
	//	  sv_buff[27]=*(TX_0.byte+3);
	//	  sv_buff[28]=*TX_1.byte;
	//	  sv_buff[29]=*(TX_1.byte+1);
	//	  sv_buff[30]=*(TX_1.byte+2);
	//	  sv_buff[31]=*(TX_1.byte+3);
	//		sv_buff[32]=*TX_2.byte;
	//		sv_buff[33]=*(TX_2.byte+1);
	//		sv_buff[34]=*(TX_2.byte+2);
	//		sv_buff[35]=*(TX_2.byte+3);
	//		sv_buff[36]=*TX_3.byte;
	//		sv_buff[37]=*(TX_3.byte+1);
	//		sv_buff[38]=*(TX_3.byte+2);
	//		sv_buff[39]=*(TX_3.byte+3);
	//		sv_buff[40]=*(TX_gyro1.byte);
	//		sv_buff[41]=*(TX_gyro1.byte+1);
	//		sv_buff[42]=*(TX_gyro1.byte+2);
	//		sv_buff[43]=*(TX_gyro1.byte+3);
	//		sv_buff[44]=*(TX_gyro2.byte);
	//		sv_buff[45]=*(TX_gyro2.byte+1);
	//		sv_buff[46]=*(TX_gyro2.byte+2);
	//		sv_buff[47]=*(TX_gyro2.byte+3);
	//		sv_buff[48]=*(TX_gyro3.byte);
	//		sv_buff[49]=*(TX_gyro3.byte+1);
	//		sv_buff[50]=*(TX_gyro3.byte+2);
	//		sv_buff[51]=*(TX_gyro3.byte+3);
	//		sv_buff[52]=*(TX_acc1.byte);
	//		sv_buff[53]=*(TX_acc1.byte+1);
	//		sv_buff[54]=*(TX_acc1.byte+2);
	//		sv_buff[55]=*(TX_acc1.byte+3);
	//		sv_buff[56]=*(TX_acc2.byte);
	//		sv_buff[57]=*(TX_acc2.byte+1);
	//		sv_buff[58]=*(TX_acc2.byte+2);
	//		sv_buff[59]=*(TX_acc2.byte+3);
	//		sv_buff[60]=*(TX_acc3.byte);
	//		sv_buff[61]=*(TX_acc3.byte+1);
	//		sv_buff[62]=*(TX_acc3.byte+2);
	//		sv_buff[63]=*(TX_acc3.byte+3);
	//		sv_buff[64]=Vision_send.shoot;
	//		sv_buff[65]=Vision_send.frame_tail;

	HAL_UART_Transmit(&VISION_HUART, (uint8_t *)sv_buff, length, 100);
}

void LanggoUartFrameIRQHandler(UART_HandleTypeDef *huart)
{

	if (huart == &huart1)
	{

		if (Vision_Data[0] == 0x73)
		{
			VisionRecvData.frame_head = Vision_Data[0];

			VisionRecvData.yaw_angle = (short)(Vision_Data[1] << 8 | Vision_Data[2]);

			VisionRecvData.pitch_angle = (short)(Vision_Data[3] << 8 | Vision_Data[4]);

			VisionRecvData.distance = (short)(Vision_Data[5] << 8 | Vision_Data[6]);

			VisionRecvData.time = (short)(Vision_Data[7] << 8 | Vision_Data[8]);

			VisionRecvData.shot_flag = Vision_Data[9];

			VisionRecvData.frame_tail = Vision_Data[10];  

			memset(Vision_Data, 0, 100);
		}
		if(UseMouseRight()){
			VisionRecvData.pitch_angle = (int)(((VisionRecvData.pitch_angle * 100) / 32767) * 100000);
			VisionRecvData.yaw_angle = (int)(((VisionRecvData.yaw_angle * 100) / 32767) * 100000);

			VisionRecvData.pitch_angle = (float)(VisionRecvData.pitch_angle / 1000) / 100.0f;
			VisionRecvData.yaw_angle = (float)(VisionRecvData.yaw_angle / 1000) / 100.0f;
			// targetYaw = VisionRecvData.yaw_angle; 
			// targetPitch = VisionRecvData.pitch_angle;
			kalman_targetYaw = KalmanFilter(&kalman_Yaw, VisionRecvData.yaw_angle);
			kalman_targetPitch = KalmanFilter(&kalman_Pitch, VisionRecvData.pitch_angle);			
		}
		else {
			VisionRecvData.pitch_angle = 0.0f;
			VisionRecvData.yaw_angle = 0.0f;
			VisionRecvData.pitch_angle = 0.0f;
			VisionRecvData.yaw_angle = 0.0f;
			// targetYaw = VisionRecvData.yaw_angle; 
			// targetPitch = VisionRecvData.pitch_angle;
			kalman_targetYaw = 0.0f;
			kalman_targetPitch = 0.0f;	
		}

		// if (VisionRecvData.shot_flag == 0x01)
		// 	Follow_Vision = 1;
		// else
		// 	Follow_Vision = 0;
	}

	MY_UART_callback(huart);
}

void USART1_IRQHandler(void)
{
	LanggoUartFrameIRQHandler(&huart1);
}

void Vision_Kalman_Init(void)
{
	KalmanCreate(&kalman_Yaw, YAW_Q, YAW_R);
	KalmanCreate(&kalman_Pitch, PITCH_Q, PITCH_R);
}

const VisionRecvData_t* AimDataUpdate(void)
{
	return &VisionRecvData;
}



