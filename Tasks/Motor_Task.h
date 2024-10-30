#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H
#include "struct_typedef.h"


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

#endif
