#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "control_cmd.h"
#include <stdint.h>
#include "motor.h"
#include "pid.h"

extern Motor motors[4]; //  引用全局电机对象
#define JOY_MAX 300 // 摇杆保险值
#define REAL_MAX_RPM 331
#define RPM_SCALE 1.10333 //  根据你的摇杆值与rpm对应关系调节

/* 计算四个麦轮速度: LF, RF, LB, RB */
void mecanum_calc(int16_t vx, int16_t vy, int16_t vw, int32_t wheel[4])
{
    wheel[0] = vx - vy - vw; // 左前
    wheel[1] = vx + vy + vw; // 右前
    wheel[2] = vx + vy - vw; // 左后
    wheel[3] = vx - vy + vw; // 右后
}

void Start_Chassis_Control(void const *argument)
{
    int32_t wheel[4];
    for (;;)
    {
        if (g_cmd.mode == 0) // 底盘模式
        {
            mecanum_calc(g_cmd.vx, g_cmd.vy, g_cmd.vw, wheel);

            for (int i = 0; i < 4; i++)
            {
                // 1. 更新编码器
                motors[i].EncoderUpdate(&motors[i]);
                // 2. 获取当前转速 (rpm)
                motors[i].SpeedGet(&motors[i]);
                // 3. 设定目标转速，需映射成rpm
                motors[i].speed_set = wheel[i] * RPM_SCALE;
                // 4. PID计算
                motors[i].Calc(&motors[i]);
                // 5. 驱动电机
                motors[i].Driver(&motors[i], (int16_t)motors[i].pid.out);
            }
        }
        else // 舵机模式
        {
            for (int i = 0; i < 4; i++)
            {
                motors[i].Driver(&motors[i], 0);
            }
        }
        osDelay(10); // 与 Motor_Speed_Get 中的采样周期保持一致
    }
}

