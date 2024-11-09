#include "bsp_can.h"
#include "main.h"
#include "Chassis_Task.h"
#include <string.h>
#define CAN_6020_ALL_ID 0x1FF
#define CAN_3508_ALL_ID 0x200
#define scale 1000

#define get_motor_measure(ptr, data)                                    \
{                                                                   \
    (ptr)->last_ecd = (ptr)->ecd;                                   \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
    (ptr)->temperate = (data)[6];                                   \
}

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}


uint8_t rx_data[8];
motor_measure_t gimbal_motor_measure[2];
motor_measure_t chassis_motor_measure[4];
extern fp32 test_data;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (hcan == &hcan1)
    {
        switch (rx_header.StdId)
        {
            case 0x205:
            {
                // Gimbal Yaw 6020
                get_motor_measure(&gimbal_motor_measure[0], rx_data);
                break;
            }
            case 0x206:
            {
                // Gimbal Pitch 6020
                get_motor_measure(&gimbal_motor_measure[1], rx_data);
                break;
            }
            case 0x201:
            case 0x202:
            case 0x203:
            case 0x204:
            {
                // Chassis 3508
                get_motor_measure(&chassis_motor_measure[rx_header.StdId - 0x201], rx_data);
                break;
            }
            default:
            {
                break;
            }
        }

    }

    if (hcan == &hcan2)
    {
        switch (rx_header.StdId)
        {
            case 0x130:
            {
                // send: memcpy(data, &tx_data, sizeof(tx_data));
                memcpy(&test_data, rx_data, sizeof(test_data));
                break;
            }
            default:
            {
                break;
            }
        }
    }
}

void CAN_CMD_6020(int16_t yaw_current, int16_t pitch_current)
{
    CAN_TxHeaderTypeDef  gimbal_tx_message;
    uint8_t gimbal_can_send_data[8];
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

void CAN_CMD_3508(int16_t motor1_current, int16_t motor2_current, int16_t motor3_current, int16_t motor4_current)
{
    CAN_TxHeaderTypeDef  chassis_tx_message;
    uint8_t chassis_can_send_data[8];
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_3508_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = (motor1_current >> 8);
    chassis_can_send_data[1] = motor1_current;
    chassis_can_send_data[2] = (motor2_current >> 8);
    chassis_can_send_data[3] = motor2_current;
    chassis_can_send_data[4] = (motor3_current >> 8);
    chassis_can_send_data[5] = motor3_current;
    chassis_can_send_data[6] = (motor4_current >> 8);
    chassis_can_send_data[7] = motor4_current;
    HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
