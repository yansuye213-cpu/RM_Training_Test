#ifndef __BLE_REMOTE_H
#define __BLE_REMOTE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include <stdint.h>

#define BLE_REMOTE_LENGTH 11
#define BLE_REMOTE_HEAD 0xFE
#define BLE_REMOTE_TAIL 0xFD
#define REMOTE_ERROR 1
#define REMOTE_OK 0
#define ROCKER_DISTANCE_MAX 500
#define ROCKER_DISTANCE_MIN 0
#define ROCKER_ANGLE_MAX 360
#define ROCKER_ANGLE_MIN 0
#define PI 3.1415926535898f

    typedef struct
    {
        int16_t distance;
        int16_t angle;
        float x_position;
        float y_position;
    } rocker_t;

    typedef struct
    {
        rocker_t rocker[2];
        uint8_t Button[4];
        uint8_t Switch[4];
    } remote_t;

    // 使用 volatile 保证 DMA/中断修改可见
    extern volatile remote_t g_remote;

    uint8_t uart_to_remote(uint8_t *buff);
    uint8_t uart_to_rocker(uint8_t *buff, rocker_t *rocker);
    const volatile remote_t *get_remote_control_point(void);

#ifdef __cplusplus
}
#endif

#endif
