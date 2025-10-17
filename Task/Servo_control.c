#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "control_cmd.h"
#include "ble_remote.h"
#include <stdint.h>
#include <math.h>

// 状态变量
static uint8_t last_switch_state[4] = {0}; // 记录四个开关的上次状态
static uint32_t release_start_time = 0;
static uint8_t is_releasing = 0;

// 舵机初始位置
static const float INITIAL_ANGLES[4] = {90.0f, 31.0f, 64.0f, 95.0f};

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

// 真空泵控制函数 (PB1)
static void vacuum_pump_control(uint8_t state)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// 电磁阀控制函数 (PB0)
static void solenoid_valve_control(uint8_t state)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
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

// 处理开关状态变化
static void handle_switch_states(void)
{
    // 检测 Switch3 上升沿 -> 设置舵机为初始位置
    uint8_t current_switch3 = g_cmd.switch_state[2];
    if (current_switch3 && !last_switch_state[2])
    {
        for (int i = 0; i < 4; i++)
        {
            g_cmd.servo_angle[i] = INITIAL_ANGLES[i];
            // 直接设置舵机角度
            servo_set_angle(i, INITIAL_ANGLES[i]);
        }
    }
    last_switch_state[2] = current_switch3;

    // 检测按钮状态，更新选中的舵机
    int selected = -1;
    for (int i = 0; i < 4; i++)
    {
        if (g_cmd.button_state[i])
        {
            selected = i;
            break;
        }
    }
    g_cmd.selected_servo = selected;
}

// 更新舵机角度 - 直接控制模式
static void update_servo_angles(void)
{
    // 如果有选中的舵机，根据摇杆直接更新角度
    if (g_cmd.mode == 1 && g_cmd.selected_servo >= 0)
    {
        // 使用较大的控制系数，提高响应速度
        float delta = g_cmd.vw * 0.003f;

        if (g_cmd.selected_servo == 2)
        { 
            delta = -delta;
        }

        // 直接更新舵机角度
        g_cmd.servo_angle[g_cmd.selected_servo] += delta;

        // 角度限制
        if (g_cmd.servo_angle[g_cmd.selected_servo] < 0)
            g_cmd.servo_angle[g_cmd.selected_servo] = 0;
        if (g_cmd.servo_angle[g_cmd.selected_servo] > 180)
            g_cmd.servo_angle[g_cmd.selected_servo] = 180;

        // 直接设置舵机角度
        servo_set_angle(g_cmd.selected_servo, g_cmd.servo_angle[g_cmd.selected_servo]);
    }
    else if (g_cmd.mode == 0)
    {
        // 在底盘模式下，保持当前角度（可选）
        for (int i = 0; i < 4; i++)
        {
            servo_set_angle(i, g_cmd.servo_angle[i]);
        }
    }
}

// 处理吸盘控制
static void handle_vacuum_control(void)
{
    uint8_t current_switch2 = g_cmd.switch_state[1];

    if (current_switch2 != last_switch_state[1])
    {
        if (current_switch2)
        {
            start_vacuum(); // 开启吸气
            g_cmd.suction_cup = 1;
        }
        else
        {
            start_release(); // 开始放气
            g_cmd.suction_cup = 0;
        }
        last_switch_state[1] = current_switch2;
    }

    if (is_releasing && (osKernelSysTick() - release_start_time > 500))
    {
        stop_release(); // 放气完成后自动关阀
    }
}

void Start_Servo_Control(void const *argument)
{
    // 初始状态：泵和阀关闭
    vacuum_pump_control(0);
    solenoid_valve_control(0);

    // 初始化舵机角度
    for (int i = 0; i < 4; i++)
    {
        servo_set_angle(i, 90);
        g_cmd.servo_angle[i] = 90;
    }

    // 初始化状态变量
    for (int i = 0; i < 4; i++)
    {
        last_switch_state[i] = g_cmd.switch_state[i];
    }
    g_cmd.selected_servo = -1;
    g_cmd.suction_cup = 0;

    for (;;)
    {
        // 统一处理所有控制逻辑
        handle_switch_states();
        update_servo_angles();
        handle_vacuum_control();

        osDelay(10); // 较短的延迟，提高响应速度
    }
}