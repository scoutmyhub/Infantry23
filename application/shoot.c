/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "shoot.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"
#include "my_pid.h"
#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "my_pid.h"
#include "referee.h"
#include "referee_lib.h"
#include "Filter.h"
#include "stdbool.h"
#define shoot_fric1_on(pwm) fric1_on((pwm)) // 摩擦轮1pwm宏定义
#define shoot_fric2_on(pwm) fric2_on((pwm)) // 摩擦轮2pwm宏定义
#define shoot_fric_off() fric_off()         // 关闭两个摩擦轮

#define shoot_laser_on() laser_on()   // 激光开启宏定义
#define shoot_laser_off() laser_off() // 激光关闭宏定义

/*射击总结构体*/
shoot_control_t ShootControl;
shoot_mode_e shoot_mode = SHOOT_STOP; // 射击状态机

pid_t trigger_motor_pid;           // 电机PID
pid_type_def Fire_left_motor_pid;  // 摩擦轮
pid_type_def Fire_right_motor_pid; // 摩擦轮
pid_type_def FireLeftMoment;
pid_type_def FireRightMoment;

int poke_spd_ref, poke_pos_ref;
float FireOffset =0.0f;

Low_Pass_Filter_t ShootMotor_Left, ShootMotor_Right;

float32_t Fire_left_speed_pid[3] = {13, 0, 10};
float32_t Fire_right_speed_pid[3] = {13,0,10};
float32_t FireLeftMomentPID[3] = {10, 0, 7};
float32_t FireRightMomentPID[3] = {10, 0, 7};

/*电机转一发所需要的编码器值（加减速箱）*/
int Single_Data = 36864;

int16_t shoot_CAN_Set_Current; // 返回的can值

int ShootFTime = 0;
int ShootFNum = 80;

/**
 * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
 * @param[in]      void
 * @retval         void
 */
static void shoot_set_mode(void);
/**
 * @brief          射击数据更新
 * @param[in]      void
 * @retval         void
 */
static void shoot_feedback_update(void);

/**
 * @brief          堵转倒转处理
 * @param[in]      void
 * @retval         void
 */
static void trigger_motor_turn_back(void);

/*数据更新*/
void Shoot_Data_Updating(void);
/*FLAG_Init*/
void FLAG_Init(void);

void FLAG_Init(void)
{
  static Control_SET_t Init = {0};
  ShootControl.Last_Control_SET = Init;
}

/**
 * @brief          射击初始化，初始化PID，遥控器指针，电机指针
 * @param[in]      void
 * @retval         返回空
 */
void shoot_init(void)
{
  /*刷新数据*/
  shoot_feedback_update();
  /*目标值初始化*/
  poke_pos_ref = ShootControl.shoot_motor_measure->total_angle;
  poke_spd_ref = ShootControl.shoot_motor_measure->total_angle % 36864;
  /*初始化PID*/
  // 拨弹盘
  PID_struct_init(&pid_poke, POSITION_PID, BULLET_ANG_PID_MAX_OUT, BULLET_ANG_PID_MAX_IOUT, TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD);         // 1.0.10
  PID_struct_init(&pid_poke_omg, POSITION_PID, BULLET_SPEED_PID_MAX_OUT, BUFFET_SPEED_PID_MAX_IOUT, TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD); // 3.0.10

  PID_init(&Fire_left_motor_pid, PID_POSITION, Fire_left_speed_pid, FIRE_left_SPEED_PID_MAX_OUT, FIRE_left_SPEED_PID_MAX_IOUT);
  PID_init(&Fire_right_motor_pid, PID_POSITION, Fire_right_speed_pid, FIRE_right_SPEED_PID_MAX_OUT, FIRE_right_SPEED_PID_MAX_IOUT);
  PID_init(&FireLeftMoment, PID_POSITION, FireLeftMomentPID, FIRE_MOMENT_PID_MAX_OUT, FIRE_MOMENT_PID_MAX_IOUT);
  PID_init(&FireRightMoment, PID_POSITION, FireRightMomentPID, FIRE_MOMENT_PID_MAX_OUT, FIRE_MOMENT_PID_MAX_IOUT);
  // 低通滤波器的初始化
  Low_Pass_Filter_Init(&ShootMotor_Left, 2, 0.45);
  Low_Pass_Filter_Init(&ShootMotor_Right, 2, 0.45);


  /*标志位清零*/
//  FLAG_Init();
}

int16_t Fire_Left;
int16_t Fire_Right;
fp32 Fire_Left_Low, Fire_Right_Low;
int16_t Fire_Left_OUT, Fire_Right_OUT;
/**
 * @brief          射击循环
 * @param[in]      void
 * @retval         返回can控制值
 */
void shoot_control_loop(void)
{
  shoot_set_mode();
  trigger_motor_turn_back();
  shoot_feedback_update();
  HeatLimitUpdate(&ShootControl);

  if (shoot_mode == SHOOT_STOP)
  {
    ShootControl.Fire_Data.Fire_right_speed_set = 0;
    ShootControl.Fire_Data.Fire_left_speed_set = 0;
    shoot_fric_off();
    shoot_laser_off();
    shoot_CAN_Set_Current = 0;
  }
  else
  {
    ShootControl.Fire_Data.Fire_right_speed_set = Get_SHOOT_RPM();
    ShootControl.Fire_Data.Fire_left_speed_set = -Get_SHOOT_RPM()+FireOffset;
    shoot_laser_on(); // 激光开启
  }
  
  
  
  Fire_Left = -motor_shoot[0].speed_rpm;
  Fire_Right = motor_shoot[1].speed_rpm;
  Low_Pass_Filter_OUT(&ShootMotor_Left, motor_shoot[0].speed_rpm);
  Low_Pass_Filter_OUT(&ShootMotor_Right, motor_shoot[1].speed_rpm);
  Fire_Left_Low = ShootMotor_Left.out;
  Fire_Right_Low = ShootMotor_Right.out;
  
  
#if Normal_Mode  
  PID_calc(&Fire_left_motor_pid, motor_shoot[0].speed_rpm , ShootControl.Fire_Data.Fire_left_speed_set);
  PID_calc(&Fire_right_motor_pid, motor_shoot[1].speed_rpm , ShootControl.Fire_Data.Fire_right_speed_set);
  PID_calc(&FireLeftMoment, motor_shoot[0].given_current, Fire_left_motor_pid.out);
  PID_calc(&FireRightMoment, motor_shoot[1].given_current, Fire_right_motor_pid.out);
#else
  PID_calc(&Fire_left_motor_pid, ShootMotor_Left.out, ShootControl.Fire_Data.Fire_left_speed_set);
  PID_calc(&Fire_right_motor_pid, ShootMotor_Right.out, ShootControl.Fire_Data.Fire_right_speed_set);
#endif
  // 计算拨弹轮电机PID
  pid_calc_old(&pid_poke, ShootControl.shoot_motor_measure->total_angle, poke_pos_ref);
  pid_calc_old(&pid_poke_omg, ShootControl.shoot_motor_measure->speed_rpm, pid_poke.pos_out);

  // 手动校准程序下档
  if (ShootControl.Control_Time.Manual_Reset_FLAG < 5000) // 等待5s
  {
    poke_pos_ref = ShootControl.shoot_motor_measure->total_angle;
    poke_spd_ref = ShootControl.shoot_motor_measure->total_angle % 36864; // 保持处于校准状态
    ShootControl.Control_Time.Manual_Reset_FLAG++;                        // 计时延时
    shoot_CAN_Set_Current = 0;                                           // 电流值为0
  }
  else if ((shoot_mode == SHOOT_STOP) && ((switch_is_down(ShootControl.RC_ctrl->rc.s[Shoot_RC_Channel_left])) || (switch_is_down(ShootControl.RC_ctrl->rc.s[Shoot_RC_Channel_right]))))
  {
    shoot_CAN_Set_Current = 0;
    poke_pos_ref = ShootControl.shoot_motor_measure->total_angle;
    poke_spd_ref = ShootControl.shoot_motor_measure->total_angle % 36864;
  }

  else 
  {
    shoot_CAN_Set_Current = (int16_t)(pid_poke_omg.pos_out);
  }

  //		 CAN_CMD_SHOOT(0,0,0,0);
  Fire_Left_OUT = (int16_t)FireLeftMoment.out;
  Fire_Right_OUT = (int16_t)FireRightMoment.out;
  CAN_cmd_shoot(shoot_CAN_Set_Current, (int16_t)Fire_left_motor_pid.out, (int16_t)Fire_right_motor_pid.out, 0);
}

/**
 * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
 * @param[in]      void
 * @retval         void
 */
 
int8_t LAST_SHOOT_OFF_KEYBOARD=0;
void shoot_set_mode(void)
{
  static int8_t last_s = RC_SW_UP;

  shoot_feedback_update();
  
  if ((switch_is_up(ShootControl.RC_ctrl->rc.s[Shoot_RC_Channel_left]) && !switch_is_up(last_s) && shoot_mode == SHOOT_STOP) || ((ShootControl.RC_ctrl->key.v & SHOOT_OPEN_FIRE) && (!(LAST_SHOOT_OFF_KEYBOARD & SHOOT_OPEN_FIRE)) && (shoot_mode == SHOOT_STOP))) 
  {
    shoot_mode = SHOOT_READY;
  }
  else if ((switch_is_up(ShootControl.RC_ctrl->rc.s[Shoot_RC_Channel_left]) && !switch_is_up(last_s) && shoot_mode != SHOOT_STOP) || ((ShootControl.RC_ctrl->key.v & SHOOT_OPEN_FIRE) && !(LAST_SHOOT_OFF_KEYBOARD & SHOOT_OPEN_FIRE) && shoot_mode != SHOOT_STOP)) 
  {
    shoot_mode = SHOOT_STOP;
    ShootFTime = 0;

  }
  
  LAST_SHOOT_OFF_KEYBOARD = ShootControl.RC_ctrl->key.v;
  if ((!switch_is_down(last_s) && (switch_is_down(ShootControl.RC_ctrl->rc.s[Shoot_RC_Channel_left]) && shoot_mode == SHOOT_READY)) || ((ShootControl.RC_ctrl->mouse.press_l && ShootControl.Last_Control_SET.LAST_PRESS_L == 0) && (shoot_mode == SHOOT_READY)))
  {
    poke_pos_ref -= Single_Data; 
    if(ShootControl.HeatLimitData.flag)
    {
      ShootControl.HeatLimitData.index++;
    }
  }
  if (ShootControl.RC_ctrl->mouse.press_l)
  {
    ShootControl.Control_Time.shoot_count_cnt++;
    if (ShootControl.Control_Time.shoot_count_cnt >= 100)
    {
      ShootControl.Control_Time.shoot_count_cnt = 101;
      ShootControl.Control_FALG.SHOOT_SWITCH_KEYBOARD = true;
    }
  }
  else
  {
    ShootControl.Control_FALG.SHOOT_SWITCH_KEYBOARD = false;
    ShootControl.Control_Time.shoot_count_cnt = 0;
  }
  if (ShootControl.Control_FALG.SHOOT_SWITCH_KEYBOARD && (shoot_mode == SHOOT_READY))
  {
    ShootFTime++;
    if (ShootFTime >= ShootFNum)
    {
      poke_pos_ref -= Single_Data;
      ShootFTime = 0;
    }
  }
  ShootControl.Last_Control_SET.LAST_PRESS_L = ShootControl.RC_ctrl->mouse.press_l;

  last_s = ShootControl.RC_ctrl->rc.s[Shoot_RC_Channel_left];
  ShootControl.Last_Control_SET.LAST_SHOOT_SWITCH_KEYBOARD = ShootControl.RC_ctrl->key.v;
}
/**
 * @brief          射击数据更新
 * @param[in]      void
 * @retval         void
 */
static void shoot_feedback_update(void)
{
  /*遥控器指针*/
  ShootControl.RC_ctrl = get_remote_control_point();
  /*电机数据更新*/
  ShootControl.shoot_motor_measure = get_shoot_trigger_measure_point();
}

static void trigger_motor_turn_back(void)
{

  if (poke_pos_ref < ShootControl.shoot_motor_measure->total_angle - 5000)
  {
    ShootControl.Turn_Back_Data.block_time++;
  }
  else
  {
    ShootControl.Turn_Back_Data.block_time = 0;
    ShootControl.HeatLimitData.flag = 1;
  }
  if (ShootControl.Turn_Back_Data.block_time > 900)
  {
    ShootControl.Turn_Back_Data.Reset_ECD = (ShootControl.shoot_motor_measure->total_angle - poke_spd_ref) / Single_Data;
    poke_pos_ref = ShootControl.Turn_Back_Data.Reset_ECD * Single_Data + poke_spd_ref;
    ShootControl.HeatLimitData.flag = 0;
    ShootControl.Turn_Back_Data.block_time = 0;
  }
}

float Get_SHOOT_RPM(void)
{
  float SHOOT_SET = 0;
  if (ID1_speed_limit() == 15)
  {
    SHOOT_SET = 4500.0;
  }
  else if (ID1_speed_limit() == 18)
  {
    SHOOT_SET = 5400.0;
  }
  else if (ID1_speed_limit() == 30)
  {
    SHOOT_SET = 7900.0;
  }
  else
  {
    SHOOT_SET = 4500.0;
  }

  return SHOOT_SET;
}

/*传入裁判系统的最大速度 单位：m/s*/
/*返回摩擦轮目标转速 单位：rpm*/
// float Get_fire_rate(uint16_t speed)
// {
// 	double n = speed * 30.0f / (PI*FIRE_RADIUS)+Reality();
// 	return  n;
// }



void HeatLimitUpdate(shoot_control_t *ShootHeatLimit)
{
  ShootHeatLimit->HeatLimitData.FireableBullet = (ID1_cooling_limit() - ID1CoolingHeat()) / 10;
  ShootHeatLimit->HeatLimitData.heat = ID1CoolingHeat();
  if(ShootControl.HeatLimitData.FireableBullet <= 1)
  {
    Single_Data = 0;
  }
  else 
  {
    Single_Data = 36864;
  }
  if(ID1CoolingHeat() <= ID1_cooling_limit()-20)
  {
    ShootFNum = 80;
  }
  else{
    ShootFNum = 150;
  }
}
