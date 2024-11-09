#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"

enum Chassis_Mode
{
  Chassis_No_Force = 0,
  Chassis_Normal,
  Chassis_Rotate,
};

typedef struct
{
  // rad/s
  fp32 velocity;
  fp32 target_velocity;
  int16_t give_current;
}chassis_motor_t;

typedef struct
{
  // control variable (player's input)
  fp32 chassis_v_x;
  fp32 chassis_v_y;
  fp32 chassis_v_rotate;

  enum Chassis_Mode mode;
  chassis_motor_t chassis_m3508[4];
}chassis_control_t;

void Chassis_Task(void const *argument);

extern fp32 test_data;

#endif
