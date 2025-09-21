#include "motor.h"

#include "math.h"
Motor motors[4];
/**
 * @brief 函数定义
 *
 */

static void Motor_Pid_Init(Motor *motor, uint8_t mode, float maxout, float max_iout, float kp, float ki, float kd);
static void Motor_Encoder_Update(Motor *motor);
static void Motor_Speed_Get(Motor *motor);
static void Motor_Calc(Motor *motor);
static void Motor_Driver(Motor *motor, int16_t pwm);

/**
 * @brief  电机对象一站式初始化
 * @author 韦将业 && Sereden
 * @date   2024-8-15
 * @param  htim_encoder  编码器模式定时器句柄
 * @param  htim_pwm      PWM 输出定时器句柄
 * @param  channel       PWM 通道（如 TIM_CHANNEL_1）
 * @param  port1/pin1    方向控制引脚1
 * @param  port2/pin2    方向控制引脚2
 * @retval None
 *
 */
void Motor_Init(Motor *motor, TIM_HandleTypeDef *htim_encoder, TIM_HandleTypeDef *htim_pwm, uint32_t channel,
                GPIO_TypeDef *port1, uint16_t pin1, GPIO_TypeDef *port2, uint16_t pin2) {
  // 绑定函数指针，类似于面向对象的设计，将电机方法放到结构体里
  motor->PidInit= Motor_Pid_Init;
  motor->EncoderUpdate = Motor_Encoder_Update;
  motor->SpeedGet = Motor_Speed_Get;
  motor->Calc = Motor_Calc;
  motor->Driver = Motor_Driver;

  // 初始化变量
  motor->htim_encoder = htim_encoder;//绑定编码器模式定时器，用于获取编码值
  motor->htim_pwm = htim_pwm;//绑定pwm定时器
  motor->channel = channel;//绑定pwm定时器通道
  motor->port1 = port1;//
  motor->pin1 = pin1;
  motor->port2 = port2;
  motor->pin2 = pin2;


  // 开启定时器
  HAL_TIM_PWM_Start(motor->htim_pwm, motor->channel);           // pwm输出
  HAL_TIM_Encoder_Start(motor->htim_encoder, TIM_CHANNEL_ALL);  // 开启正交解码
}

/*===================== PID 参数初始化 =====================*/
static void Motor_Pid_Init(Motor *motor, uint8_t mode, float maxout, float max_iout, float kp, float ki, float kd) {
  PID_Init(&motor->pid, mode, maxout, max_iout, kp, ki, kd);
}

/*===================== 核心闭环：计算 PWM =====================*/
static void Motor_Calc(Motor *motor) { motor->pid.out = PID_Calculate(&motor->pid, motor->speed_filter, motor->speed_set); }


/**
 * @brief 获取编码器的脉冲，并计算与上次的差值
 * @author Sereden
 * @date 2024-8-17
 * @retval None
 */
static void Motor_Encoder_Update(Motor *motor) {
  motor->encoder.lastCount = motor->encoder.currentCount;//保留旧值
  motor->encoder.currentCount = __HAL_TIM_GET_COUNTER(motor->htim_encoder);//获取当前值
  motor->encoder.deltaCount = motor->encoder.currentCount - motor->encoder.lastCount;//计算差值
  //  16 位溢出/下溢补偿
  if (motor->encoder.deltaCount > 32768) {
    motor->encoder.deltaCount -= 65536;
  } else if (motor->encoder.deltaCount < -32768) {
    motor->encoder.deltaCount += 65536;
  }
}

/**
 * @brief 读取电机速度，单位rpm
 * @author Sereden
 * @date 2024-8-14
 * @details 编码器读取解算数据不够准确，加上一阶互补滤波，系数瞎填的
 */
static void Motor_Speed_Get(Motor *motor) {
  motor->encoder.speed = (float)motor->encoder.deltaCount / (MOTOR_SPEED_RERATIO * PULSE_PRE_ROUND * MULTIPLE_ENCODER) *(60000.0f / (POLL_INTERVAL_MS + 1));
  motor->encoder.lineSpeed = motor->encoder.speed * LINE_SPEED_C / 60.0f;
  motor->last_speed = motor->speed;
  motor->speed = motor->encoder.speed;
  motor->speed_filter = (1 - MOTOR_FILTER) * motor->speed + MOTOR_FILTER * motor->last_speed;
}
/**
 * @brief 驱动电机
 * @author 韦将业 && Serede
 * @date 2024-8-15
 * @param value 电机速度
 * @retval None
 *
 */
static void Motor_Driver(Motor *motor, int16_t pwm_value) {
  if (pwm_value > 0)
  {
    // 正转
    HAL_GPIO_WritePin(motor->port1, motor->pin1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor->port2, motor->pin2, GPIO_PIN_RESET);
  }
  else if (pwm_value < 0)
  {
    // 反转
    HAL_GPIO_WritePin(motor->port1, motor->pin1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->port2, motor->pin2, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(motor->port1, motor->pin1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->port2, motor->pin2, GPIO_PIN_RESET);
  }
  __HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->channel, fabs(pwm_value));
}
