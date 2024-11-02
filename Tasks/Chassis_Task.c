#include "Chassis_Task.h"
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "pid.h"
#include "remote_control.h"
#include <string.h>

// Parameters
#define WHEEL_DIAMETER 0.1525f
#define REDUCTION_RATIO 19.203f
#define MOTOR_RPM_TO_LINEAR_VELOCITY (2*PI/60)*(WHEEL_DIAMETER/2)/REDUCTION_RATIO
#define K MOTOR_RPM_TO_LINEAR_VELOCITY
#define ROTATE_RADIUS 0.27f

// PID
#define WHEEL_VELOCITY_PID_KP 5000.0f
#define WHEEL_VELOCITY_PID_KI 0.0f
#define WHEEL_VELOCITY_PID_KD 0.0f
#define WHEEL_VELOCITY_PID_MAX_OUT 15000.0f
#define WHEEL_VELOCITY_PID_MAX_IOUT 4000.0f
const static fp32 wheel_velocity_pid[3] = {5000.0f, 0.0f, 0.0f};
pid_type_def wheel_velocity_pid[4];

// Struct
chassis_control_t chassis_control;
chassis_motor_t wheel[4];

void Chassis_Init(void)
{
    memset(&chassis_control,0,sizeof(chassis_control_t));
	wheel = chassis_control.chassis_m3508;
	const static fp32 wheel_velocity_pid[3] = {WHEEL_VELOCITY_PID_KP, WHEEL_VELOCITY_PID_KI, WHEEL_VELOCITY_PID_KD};
	for (int i = 0; i < 4; i++)
	{
		PID_init(&chassis_control.wheel_velocity_pid[i],PID_POSITION,wheel_velocity_pid, WHEEL_VELOCITY_PID_MAX_OUT, WHEEL_VELOCITY_PID_MAX_IOUT);
	}
}

extern motor_measure_t chassis_motor_measure[4];
void Chassis_Task(void const * argument)
{
    Chassis_Init();
    while(1)
    {

        vTaskDelay(1);
    }
}