#include "Chassis_Task.h"
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "pid.h"
#include "remote_control.h"
#include <string.h>
#include "bsp_can.h"

// Parameters
#define PI 3.14159265359f
#define WHEEL_DIAMETER 0.1525f
#define REDUCTION_RATIO 19.203f
#define MOTOR_RPM_TO_LINEAR_VELOCITY (2*PI/60)*(WHEEL_DIAMETER/2)/REDUCTION_RATIO
#define K MOTOR_RPM_TO_LINEAR_VELOCITY
#define ROTATE_RADIUS 0.27f
#define RC_RANGE 660.0f
#define sqrt2 1.41421356237f

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
fp32 chassis_sensitivity = 0.001f;

// Variable
fp32 vx, vy, wz;

// Magic number
fp32 rotate_speed = 0.2f;

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
    if (switch_is_down(rc_ctrl_chassis->rc.s[0]))
    {
        *chassis_mode = Chassis_No_Force;
        return;
    }    
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
    for (int i = 0; i < 4; i++)
    {
        wheel[i].velocity = chassis_motor_measure[i].speed_rpm * K; // rad / s
    }
}

// Get motor current
void Chassis_Control(void)
{
    switch (*chassis_mode)
    {
        case Chassis_No_Force:
            // for (int i = 0; i < 4; i++)
            // {
            //     wheel[i].give_current = 0;
            // }
        break;
        case Chassis_Normal:
            // Get target chassis velocity
            chassis_control.chassis_v_x = rc_ctrl_chassis->rc.ch[1] / RC_RANGE;
            chassis_control.chassis_v_y = - rc_ctrl_chassis->rc.ch[0] / RC_RANGE;
            chassis_control.chassis_v_rotate = - rc_ctrl_chassis->rc.ch[2] / RC_RANGE;
            // Calculate each target motor velocity
            // wheel[0].target_velocity = 2 * chassis_control.chassis_v_x + 2 * chassis_control.chassis_v_y - sqrt2 * chassis_control.chassis_v_rotate;
            // wheel[1].target_velocity = - 2 * chassis_control.chassis_v_x + 2 * chassis_control.chassis_v_y - sqrt2 * chassis_control.chassis_v_rotate;
            // wheel[2].target_velocity = - 2 * chassis_control.chassis_v_x - 2 * chassis_control.chassis_v_y - sqrt2 * chassis_control.chassis_v_rotate;
            // wheel[3].target_velocity = 2 * chassis_control.chassis_v_x - 2 * chassis_control.chassis_v_y - sqrt2 * chassis_control.chassis_v_rotate;
            // Calculate each motor current using PID
            // for (int i = 0; i < 4; i++)
            // {
            //     wheel[i].give_current = PID_calc(&wheel_velocity_pid[i], wheel[i].velocity, wheel[i].target_velocity);
            // }
        break;
        case Chassis_Rotate:
            chassis_control.chassis_v_rotate = rotate_speed;
            // wheel[0].target_velocity = - sqrt2 * chassis_control.chassis_v_rotate;
            // wheel[1].target_velocity = - sqrt2 * chassis_control.chassis_v_rotate;
            // wheel[2].target_velocity = - sqrt2 * chassis_control.chassis_v_rotate;
            // wheel[3].target_velocity = - sqrt2 * chassis_control.chassis_v_rotate;
            // for (int i = 0; i < 4; i++)
            // {
            //     wheel[i].give_current = PID_calc(&wheel_velocity_pid[i], wheel[i].velocity, wheel[i].target_velocity);
            // }
        break;
    }
}
uint8_t test_data[8];

void Chassis_Task(void const * argument)
{
    Chassis_Init();
    while(1)
    {
        Chassis_Mode_Set();
        // Chassis_Data_Update();
        Chassis_Control();
        
        // x velocity, y velocity, rotate velocity, chassis mode
        memset(test_data, 0, 8);
        test_data[0] = (chassis_control.chassis_v_x + 1.0f) * 127.5f;
        test_data[1] = (chassis_control.chassis_v_y + 1.0f) * 127.5f;
        test_data[2] = (chassis_control.chassis_v_rotate + 1.0f) * 127.5f;
        test_data[3] = *chassis_mode;
        CAN_GIMBAL_TO_CHASSIS(test_data);
        vTaskDelay(1);
    }
}
