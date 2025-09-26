#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "control_cmd.h"
#include "ble_remote.h"
#include <stdint.h>

// -------- 状态变量 --------
static uint8_t last_switch2_state = 0; // Switch2 上次状态
static uint8_t last_switch3_state = 0; // Switch3 上次状态
static uint32_t release_start_time = 0;
static uint8_t is_releasing = 0;

extern volatile uint8_t servo_reset_lock; // 在 main.c 定义
extern volatile uint32_t servo_lock_time;

void servo_set_angle(uint8_t id, float angle)
{
    if (angle < 0)
        angle = 0;
    else if (angle > 180)
        angle = 180;

    float pulse = (angle / 180.0f) * 200.0f + 50.0f;

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

// 真空泵控制函数  (PB0)
static void vacuum_pump_control(uint8_t state)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void solenoid_valve_control(uint8_t state)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void start_vacuum(void)
{
    solenoid_valve_control(0); // 关闭电磁阀
    vacuum_pump_control(1);    // 开启真空泵
    is_releasing = 0;
}

static void start_release(void)
{
    vacuum_pump_control(0);    // 关闭真空泵
    solenoid_valve_control(1); // 打开电磁阀
    is_releasing = 1;
    release_start_time = osKernelSysTick();
}

static void stop_release(void)
{
    solenoid_valve_control(0); // 关闭电磁阀
    is_releasing = 0;
}

// -------- 舵机任务 --------
void Start_Servo_Control(void const *argument)
{
    vacuum_pump_control(0);
    solenoid_valve_control(0);

    for (;;)
    {
        /* 1. 检测 Switch3 上升沿 -> 四舵机回初始位并锁定 1 秒 */
        uint8_t current_switch3 = g_remote.Switch[2];
        if (current_switch3 && !last_switch3_state)
        {
            g_cmd.servo_angle[0] = 90;
            g_cmd.servo_angle[1] = 35;
            g_cmd.servo_angle[2] = 55;
            g_cmd.servo_angle[3] = 90;

            servo_reset_lock = 1;                // 进入锁定
            servo_lock_time = osKernelSysTick(); // 记录时间
        }
        last_switch3_state = current_switch3;

        /* 2. 自动解锁：1 秒后恢复蓝牙写入 */
        if (servo_reset_lock &&
            (osKernelSysTick() - servo_lock_time > 1000))
        {
            servo_reset_lock = 0;
        }

        /* 3. 舵机角度输出（锁定与否都维持当前 g_cmd 值） */
        for (int s = 0; s < 4; s++)
        {
            servo_set_angle(s, g_cmd.servo_angle[s]);
        }

        /* 4. 吸盘控制（Switch2） */
        uint8_t current_switch2 = g_remote.Switch[1];
        if (current_switch2 != last_switch2_state)
        {
            if (current_switch2)
                start_vacuum(); // 开启吸气
            else                // Switch2 关闭
                start_release(); // 开始放气
            last_switch2_state = current_switch2;
        }

        if (is_releasing &&
            (osKernelSysTick() - release_start_time > 500))
        {
            stop_release(); // 放气完成后自动关阀
        }

        osDelay(20);
    }
}
