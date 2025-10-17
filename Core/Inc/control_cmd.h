#ifndef __CONTROL_CMD_H
#define __CONTROL_CMD_H

#include <stdint.h>

typedef struct
{
    int16_t vx, vy, vw;   // 底盘速度
    float servo_angle[4]; // 舵机角度
    int selected_servo;   // 当前选中舵机
    uint8_t mode;         // 0=底盘,1=舵机
    uint8_t suction_cup;  // 吸盘控制，0:关闭，1:开启

    // 新增字段用于统一状态管理
    uint8_t switch_state[4]; // 四个开关的当前状态
    uint8_t button_state[4]; // 四个按钮的当前状态
} control_cmd_t;

/* 在这里声明全局变量 */
extern volatile control_cmd_t g_cmd;

#endif
