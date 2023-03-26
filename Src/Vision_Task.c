#include "Vision_Task.h"

VisionData_t VisionData;



void Vision_task(void const *pvParameters)
{
	MY_UART_DMA_Init();
	Vision_Kalman_Init();
	while (1)
	{
		vision_send();
		AutoDataUpdate();
		vTaskDelay(5);
	}
}


void AutoDataUpdate(void)
{
	VisionData.DataStruct = AimDataUpdate();

	VisionData.get_angle_point = get_INS_angle_point();
	VisionData.get_gyro_point  = get_gyro_data_point();
	VisionData.RcCtrl          = get_remote_control_point();

	LoopQueueYaw(50, VisionData.DataStruct->yaw_angle);
	LoopQueuePitch(50, VisionData.DataStruct->pitch_angle);
	
	// VisionData.PredictionData.PredictionPitch = 
	// VisionData.PredictionData.PredictionYaw  = 
	
}


