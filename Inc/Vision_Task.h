#ifndef VISION_TASK_H
#define VISION_TASK_H

#include "kalman.h"
#include "referee_lib.h"
#include "Vision.h"
#include "bsp_usart.h"
#include "stdlib.h"
#include "INS_task.h"
#include "usart.h"
#include "cmsis_os.h"
#include "math.h"
#include "remote_control.h"
#include "user_lib.h"

typedef struct
{
	float PredictionYaw;
	float PredictionPitch;

	float PredictionYawRpm;
	float PredictionPitchRpm;

	float YawOffset;
	float PitchOffset;
}PredictionData_t;



typedef struct 
{
	const VisionRecvData_t* DataStruct;
    const RC_ctrl_t *RcCtrl;
    const fp32 *get_angle_point;
    const fp32 *get_gyro_point;

	PredictionData_t PredictionData;
	// Queue VisionDataQueue[2];
//	struct queue 

}VisionData_t;




void AutoDataUpdate(void);

#endif

