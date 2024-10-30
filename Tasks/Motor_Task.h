#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H
#include "struct_typedef.h"
#include "pid.h"


typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;

} motor_measure_t;

extern fp32 motor_rpm, motor_rpm_pitch;
extern int16_t target_rpm;
extern fp32 pitch_relative_ecd_angle, pitch_target_angle, pitch_target_rpm;


enum Gimbal_Mode
{
    Gimbal_No_Force = 0,
    Recenter_Yaw,
    Recenter_Pitch,
    Normal,
    Auto_Aim_Debug,
    No_Yaw,
    No_Pitch,
    Turn_round,
    Fly,
};

typedef struct
{
    // measurement
    fp32 last_encoder;
    fp32 encoder;
    fp32 rpm;
    
    // converted value
    fp32 relative_ecd_angle;
    fp32 ecd_velocity;

    fp32 imu_angle;
    fp32 relative_imu_angle;
    
    fp32 imu_velocity;

    // control variable
    fp32 target_angle;
    fp32 target_velocity;

    pid_type_def ecd_velocity_pid;
    pid_type_def imu_velocity_pid;
    pid_type_def ecd_angle_pid;
    pid_type_def imu_angle_pid;

    int16_t give_current;

    // auxiliary value
    fp32 recenter_imu_angle;

} gimbal_motor_t;

typedef struct
{
	gimbal_motor_t gimbal_m6020[2];
	enum Gimbal_Mode mode;
    uint8_t auto_aim;
    uint8_t Turn_round;

} gimbal_control_t;


extern gimbal_control_t gimbal_control;
extern gimbal_motor_t *pitch, *yaw;
extern uint8_t gimbal_chassis_switch;
extern uint8_t right_click, right_click_last;


void Gimbal_Task(void const * argument);
enum Gimbal_Mode get_gimbal_mode(void);

#endif
