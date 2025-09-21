#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "control_cmd.h"
#include "stdint.h"
#include "ble_remote.h"

void servo_set_angle(uint8_t id, float angle)
{
    if (angle < 0)
        angle = 0;
    else if (angle > 180)
        angle = 180;
        
    float pulse = (angle / 180) * 200 + 50;

    switch (id)
    {
    case 0:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
        break;
    case 1:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse);
        break;
    case 2:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse);
        break;
    case 3:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse);
        break;
    default:
        break;
    }
}

void Start_Servo_Control(void const *argument)
{
    for (;;)
    {
        if (g_cmd.mode == 1)
        { // 仅在舵机模式下控制
            for (int s = 0; s < 4; s++)
                servo_set_angle(s, g_cmd.servo_angle[s]);

            // 使用 Switch2 控制吸盘，g_remote.Switch[1] 对应 Switch2
            if (g_remote.Switch[1])
            {
                // 开启吸盘
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
            }
            else
            {
                // 关闭吸盘
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
            }
        }
        osDelay(20);
    }
}