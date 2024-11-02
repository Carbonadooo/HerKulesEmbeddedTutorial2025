#include "Chassis_Task.h"
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "pid.h"
#include "remote_control.h"
#include <string.h>
#include "bsp_can.h"

// Parameters
#define WHEEL_DIAMETER 0.1525f
#define REDUCTION_RATIO 19.203f
#define MOTOR_RPM_TO_LINEAR_VELOCITY (2*PI/60)*(WHEEL_DIAMETER/2)/REDUCTION_RATIO
#define K MOTOR_RPM_TO_LINEAR_VELOCITY
#define ROTATE_RADIUS 0.27f

// PID
#define WHEEL_VELOCITY_PID_MAX_OUT 15000.0f
#define WHEEL_VELOCITY_PID_MAX_IOUT 4000.0f
const static fp32 wheel_velocity_pid_param[3] = {5000.0f, 0.0f, 0.0f};
pid_type_def wheel_velocity_pid[4];

// Struct
chassis_control_t chassis_control;
chassis_motor_t *wheel;
enum Chassis_Mode *chassis_mode;
extern motor_measure_t chassis_motor_measure[4];

// RC control
const RC_ctrl_t *rc_ctrl_chassis;

// Variable
fp32 vx, vy, wz;

void Chassis_Init(void)
{
    memset(&chassis_control,0,sizeof(chassis_control_t));
	wheel = chassis_control.chassis_m3508;
	for (int i = 0; i < 4; i++)
	{
		PID_init(&wheel_velocity_pid[i],PID_POSITION,wheel_velocity_pid_param, WHEEL_VELOCITY_PID_MAX_OUT, WHEEL_VELOCITY_PID_MAX_IOUT);
	}
    rc_ctrl_chassis = get_remote_control_point();
    chassis_mode = &chassis_control.mode;
}

// Set chassis mode according to remote control input
void Chassis_Mode_Set(void)
{
    if (switch_is_down(rc_ctrl_chassis->rc.s[0])) *chassis_mode = Chassis_No_Force;
    switch (*chassis_mode)
    {
    case Chassis_No_Force:
        if (switch_is_mid(rc_ctrl_chassis->rc.s[0])) *chassis_mode = Chassis_Normal;
        break;
    case Chassis_Normal:
        if (switch_is_up(rc_ctrl_chassis->rc.s[0])) *chassis_mode = Chassis_Rotate;
        break;
    case Chassis_Rotate:
        if (switch_is_mid(rc_ctrl_chassis->rc.s[0])) *chassis_mode = Chassis_Normal;
        break;
    default:
        break;
    }
}

// Update chassis data
void Chassis_Data_Update(void)
{
    
}

// Get motor current
void Chassis_Control(void)
{
    // Get target chassis velocity

    // Calculate each target motor velocity

    // Calculate each motor current using PID

}

void Chassis_Task(void const * argument)
{
    Chassis_Init();
    while(1)
    {
        Chassis_Mode_Set();
        Chassis_Data_Update();
        Chassis_Control();
        CAN_CMD_3508(wheel[0].give_current, wheel[1].give_current, wheel[2].give_current, wheel[3].give_current);
        vTaskDelay(1);
    }
}
