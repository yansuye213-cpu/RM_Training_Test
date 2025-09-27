#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "control_cmd.h"
#include <stdint.h>
#include "motor.h"
#include "pid.h"

extern Motor motors[4]; //  引用全局电机对象

// === 统一配置 ===
#define JOY_RANGE 300.0f     // 取摇杆四方向测得的最小绝对值
#define MOTOR_MAX_RPM 331.0f // 电机安全最大转速（rpm）
#define ROTATE_GAIN 1.5f     // 旋转速度放大系数

/* 计算四个麦轮速度: LF, RF, LB, RB */
static void mecanum_calc(float vx, float vy, float vw, float wheel[4])
{
    wheel[0] = vx - vy - vw; // 左前
    wheel[1] = vx + vy + vw; // 右前
    wheel[2] = vx + vy - vw; // 左后
    wheel[3] = vx - vy + vw; // 右后
}

void Start_Chassis_Control(void const *argument)
{
    float wheel[4];
    for (;;)
    {
        if (g_cmd.mode == 0) // 底盘模式
        {
            /* ---------- 1. 摇杆归一化到 [-1,1] ---------- */
            float vx = (float)g_cmd.vx / JOY_RANGE;
            float vy = (float)g_cmd.vy / JOY_RANGE;
            float vw = ((float)g_cmd.vw / JOY_RANGE) * ROTATE_GAIN;
            if (vx > 1.0f)
                vx = 1.0f;
            if (vx < -1.0f)
                vx = -1.0f;
            if (vy > 1.0f)
                vy = 1.0f;
            if (vy < -1.0f)
                vy = -1.0f;
            if (vw > 1.0f)
                vw = 1.0f;
            if (vw < -1.0f)
                vw = -1.0f;

            /* ---------- 2. 麦轮速度解算 ---------- */
            mecanum_calc(vx, vy, vw, wheel);

            /* ---------- 3. 归一化结果转成 rpm 并限幅 ---------- */
            for (int i = 0; i < 4; i++)
            {
                float target_rpm = wheel[i] * MOTOR_MAX_RPM;
                if (target_rpm > MOTOR_MAX_RPM)
                    target_rpm = MOTOR_MAX_RPM;
                if (target_rpm < -MOTOR_MAX_RPM)
                    target_rpm = -MOTOR_MAX_RPM;

                // 1. 更新编码器
                motors[i].EncoderUpdate(&motors[i]);
                // 2. 获取当前转速 (rpm)
                motors[i].SpeedGet(&motors[i]);
                // 3. 设定目标转速
                motors[i].speed_set = target_rpm;
                // 4. PID计算
                motors[i].Calc(&motors[i]);
                // 5. 驱动电机//motors[1]
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

