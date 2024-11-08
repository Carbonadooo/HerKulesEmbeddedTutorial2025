#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "struct_typedef.h"

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;

} motor_measure_t;

extern void can_filter_init(void);
extern void CAN_CMD_6020(int16_t yaw_current, int16_t pitch_current);
extern void CAN_CMD_3508(int16_t motor1_current, int16_t motor2_current, int16_t motor3_current, int16_t motor4_current);
extern motor_measure_t gimbal_motor_measure[2];
extern motor_measure_t chassis_motor_measure[4];

#endif
