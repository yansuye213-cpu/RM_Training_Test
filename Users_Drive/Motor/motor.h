#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "gpio.h"
#include "pid.h"
#include "tim.h"

#include "ble_remote.h"


#define MOTOR_FILTER 0.2
#define POLL_INTERVAL_MS 9      // 轮询间隔
#define MOTOR_SPEED_RERATIO 31u  // 电机减速比
#define PULSE_PRE_ROUND 11       // 一圈多少个脉冲
#define RADIUS_OF_TYRE 30        // 轮胎半径，单位毫米
#define MULTIPLE_ENCODER 4       // 编码器倍频,采用T1&T2计数，倍数*4
#define LINE_SPEED_C RADIUS_OF_TYRE * 2 * 3.1415926535897

typedef struct _ENCODER {
  int32_t lastCount;     // 上一次计数值
  int32_t currentCount;  // 当前计数值
  int32_t deltaCount;    // 两次计数值之差
  float speed;           // 电机转速
  float lineSpeed;       // 线速度
  uint8_t direct;        // 旋转方向
} Encoder;

typedef struct _MOTOR {
  TIM_HandleTypeDef *htim_encoder;
  TIM_HandleTypeDef *htim_pwm;
  uint32_t channel;
  GPIO_TypeDef *port1;
  uint16_t pin1;
  GPIO_TypeDef *port2;
  uint16_t pin2;

  Encoder encoder;
  float last_speed;
  float speed;
  float speed_filter;
  float speed_set;
  PID pid;

  void (*PidInit)(struct _MOTOR *motor, uint8_t mode, float maxout, float max_iout, float kp, float ki, float kd);
  void (*EncoderUpdate)(struct _MOTOR *motor);
  void (*SpeedGet)(struct _MOTOR *motor);
  void (*Calc)(struct _MOTOR *motor);
  void (*Driver)(struct _MOTOR *motor, int16_t pwm);
} Motor;

extern Motor motors[4];

void Motor_Init(Motor *motor, TIM_HandleTypeDef *htim_encoder, TIM_HandleTypeDef *htim_pwm, uint32_t channel,
                GPIO_TypeDef *port1, uint16_t pin1, GPIO_TypeDef *port2, uint16_t pin2);

#endif /* __MOTOR_H__ */
