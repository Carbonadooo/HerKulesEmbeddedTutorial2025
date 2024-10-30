#include "LED_Task.h"
#include "main.h"
#include "cmsis_os.h"
#include "Motor_Task.h"
#include <stdio.h>

fp32 target_rpm_fp, target_rpm_fp_pitch;

void LEDTask(void const * argument){

    while (1){
        #ifdef SERIAL_PLOT_DEBUG
            target_rpm_fp = (fp32)target_rpm;
            target_rpm_fp_pitch = (fp32)pitch_target_rpm;
            printf("%.3f,%.3f,%.3f,%.3f\r\n", motor_rpm_pitch, target_rpm_fp_pitch, pitch_relative_ecd_angle, pitch_target_angle);
            osDelay(1);
        #else
            // Turn on the green LED
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET); 
            osDelay(125); // Delay for 125ms
            // Turn off the green LED
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET); 
            osDelay(125); // Delay for 125ms
        #endif
    }
}
