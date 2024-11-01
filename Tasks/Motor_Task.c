#include "Motor_Task.h"
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "pid.h"
#include "remote_control.h"

#define CAN_6020_ALL_ID 0x1FF
#define get_motor_measure(ptr, data)                                    \
{                                                                   \
    (ptr)->last_ecd = (ptr)->ecd;                                   \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
    (ptr)->temperate = (data)[6];                                   \
}

pid_type_def motor_pos_pid;
const fp32 motor_pos_pid_para[3] = {0.0f, 0.0f, 0.0f};
fp32 max_out_pos = 0.0f;
fp32 max_iout_pos = 0.0f;

pid_type_def motor_vel_pid;
const fp32 motor_vel_pid_para[3] = {50.0f, 0.0f, 0.0f};
fp32 max_out_vel = 1000.0f;
fp32 max_iout_vel = 0.0f;

//pitch
pid_type_def motor_pos_pid_pitch;
const fp32 motor_pos_pid_para_pitch[3] = {60.0f, 0.0f, 0.0f};
fp32 max_out_pos_pitch = 200.0f;
fp32 max_iout_pos_pitch = 0.0f;

pid_type_def motor_vel_pid_pitch;
const fp32 motor_vel_pid_para_pitch[3] = {30.0f, 0.01f, 5.0f};
fp32 max_out_vel_pitch = 5000.0f;
fp32 max_iout_vel_pitch = 1000.0f;

const RC_ctrl_t *rc_ctrl_motor;
void Motor_Init()
{
    rc_ctrl_motor = get_remote_control_point();
    PID_init(&motor_vel_pid, PID_POSITION, motor_vel_pid_para, max_out_vel, max_iout_vel);
    PID_init(&motor_pos_pid, PID_POSITION, motor_pos_pid_para, max_out_pos, max_iout_pos);

    PID_init(&motor_vel_pid_pitch, PID_POSITION, motor_vel_pid_para_pitch, max_out_vel_pitch, max_iout_vel_pitch);
    PID_init(&motor_pos_pid_pitch, PID_POSITION, motor_pos_pid_para_pitch, max_out_pos_pitch, max_iout_pos_pitch);
}

motor_measure_t motor_measure, motor_measure_pitch;
fp32 motor_rpm, motor_rpm_pitch;
fp32 pitch_relative_angle = 0.0f;
uint16_t pitch_init_encoder = 7500;
int relative_encoder;
fp32 pitch_relative_ecd_angle = 0.0f;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        case 0x205:
        {
            get_motor_measure(&motor_measure, rx_data);
            static fp32 speed_filter_1 = 0.0f;
            static fp32 speed_filter_2 = 0.0f;
            static fp32 speed_filter_3 = 0.0f;
            
            static const fp32 filter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

            speed_filter_1 = speed_filter_2;
            speed_filter_2 = speed_filter_3;
            speed_filter_3 = speed_filter_2 * filter_num[0] + speed_filter_1 * filter_num[1] + (motor_measure.speed_rpm) * filter_num[2];
            motor_rpm = speed_filter_3;


            break;
        }
        case 0x206:
        {
            get_motor_measure(&motor_measure_pitch, rx_data);
            static fp32 speed_filter_1_pitch = 0.0f;
            static fp32 speed_filter_2_pitch = 0.0f;
            static fp32 speed_filter_3_pitch = 0.0f;
            
            static const fp32 filter_num_pitch[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

            speed_filter_1_pitch = speed_filter_2_pitch;
            speed_filter_2_pitch = speed_filter_3_pitch;
            speed_filter_3_pitch = speed_filter_2_pitch * filter_num_pitch[0] + speed_filter_1_pitch * filter_num_pitch[1] + (motor_measure_pitch.speed_rpm) * filter_num_pitch[2];
            motor_rpm_pitch = speed_filter_3_pitch;

            // encoder: 0-8191
            relative_encoder = motor_measure_pitch.ecd - pitch_init_encoder;
            pitch_relative_ecd_angle = (fp32)relative_encoder / 8192.0f * 360.0f; // [0, 360)

            break;
        }

        default:
        {
            break;
        }
    }
}


CAN_TxHeaderTypeDef  gimbal_tx_message;
extern CAN_HandleTypeDef hcan1;
uint8_t gimbal_can_send_data[8];
void CAN_cmd_6020(int16_t yaw_current, int16_t pitch_current)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_6020_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw_current >> 8);
    gimbal_can_send_data[1] = yaw_current;
    gimbal_can_send_data[2] = (pitch_current >> 8);
    gimbal_can_send_data[3] = pitch_current;
    gimbal_can_send_data[4] = 0;
    gimbal_can_send_data[5] = 0;
    gimbal_can_send_data[6] = 0;
    gimbal_can_send_data[7] = 0;
    HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

int16_t yaw_current = 0, pitch_current = 0;
int16_t target_rpm = 0, target_ecd = 0;
fp32 pitch_target_rpm = 0.0f, pitch_target_angle = 0.0f;
fp32 pitch_sensitivity = 0.0001f;
fp32 pitch_min_angle = -90.0f, pitch_max_angle = 10.0f;

void MotorTask(void const * argument)
{
    Motor_Init();
    while (1)
    {

        pitch_target_angle += -(fp32)rc_ctrl_motor->rc.ch[1] * pitch_sensitivity;
        if (pitch_target_angle < pitch_min_angle)
            pitch_target_angle = pitch_min_angle;
        if (pitch_target_angle > pitch_max_angle)
            pitch_target_angle = pitch_max_angle;


        // target_rpm = PID_calc(&motor_pos_pid, motor_measure.ecd, target_ecd);
        yaw_current = PID_calc(&motor_vel_pid, motor_rpm, (fp32)target_rpm);

        pitch_target_rpm = PID_calc(&motor_pos_pid_pitch, pitch_relative_ecd_angle, pitch_target_angle);
        pitch_current = PID_calc(&motor_vel_pid_pitch, motor_rpm_pitch, pitch_target_rpm);

        CAN_cmd_6020(yaw_current, pitch_current);
        osDelay(1); // freq > 300Hz
    }
    
}
