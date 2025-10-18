/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "pid.h"
#include "string.h"
#include "ble_remote.h"
#include "stdlib.h"
#include "control_cmd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RX_BUF_LEN 128
#define BLE_FRAME_LEN 11

/* 全局控制数据，两个任务都直接读取 */
volatile control_cmd_t g_cmd;
volatile uint8_t vacuum_on = 0; // 0=关闭；1=开启

/* 蓝牙解包的变量 */
static uint8_t rx_buf[RX_BUF_LEN];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
static void ProcessBleData(uint8_t *buf, uint16_t size)
{
  if (size < BLE_REMOTE_LENGTH)
  {
    // 处理单个字符命令
    if (size == 1 && buf[0] >= '1' && buf[0] <= '9' && g_cmd.switch_state[0])
    {
      // 设置舵机角度到预设位置（每个舵机不同角度）
      switch (buf[0])
      {
      case '1': // 预设1
        g_cmd.servo_angle[0] = 55.512f;
        g_cmd.servo_angle[1] = 51.289f;
        g_cmd.servo_angle[2] = 76.723f;
        g_cmd.servo_angle[3] = 101.663f;
        break;
      case '2': // 预设2
        g_cmd.servo_angle[0] = 29.907f;
        g_cmd.servo_angle[1] = 67.807f;
        g_cmd.servo_angle[2] = 88.690f;
        g_cmd.servo_angle[3] = 102.047f;
        break;
      case '3': // 预设3
        g_cmd.servo_angle[0] = 2.028f;
        g_cmd.servo_angle[1] = 71.326f;
        g_cmd.servo_angle[2] = 81.532f;
        g_cmd.servo_angle[3] = 99.407f;
        break;
      case '4': // 预设4
        g_cmd.servo_angle[0] = 123.201f;
        g_cmd.servo_angle[1] = 97.498f;
        g_cmd.servo_angle[2] = 76.150f;
        g_cmd.servo_angle[3] = 54.548f;
        break;
      case '5': // 预设5
        g_cmd.servo_angle[0] = 119.937f;
        g_cmd.servo_angle[1] = 113.380f;
        g_cmd.servo_angle[2] = 88.999f;
        g_cmd.servo_angle[3] = 34.496f;
        break;
      }
    }
    return;
  }

  // 蓝牙帧处理
  for (uint16_t i = 0; i + BLE_REMOTE_LENGTH <= size; i++)
  {
    // 使用头文件中定义的帧头尾
    if (buf[i] == BLE_REMOTE_HEAD && buf[i + BLE_REMOTE_LENGTH - 1] == BLE_REMOTE_TAIL)
    {
      if (uart_to_remote(&buf[i]) == REMOTE_OK)
      {
        // 只更新原始数据，不处理业务逻辑
        control_cmd_t cmd;

        // 更新开关状态
        for (int j = 0; j < 4; j++)
        {
          cmd.switch_state[j] = g_remote.Switch[j];
        }

        // 更新按钮状态
        for (int j = 0; j < 4; j++)
        {
          cmd.button_state[j] = g_remote.Button[j];
        }

        // 更新摇杆数据
        cmd.vx = (int16_t)g_remote.rocker[0].y_position;
        cmd.vy = (int16_t)g_remote.rocker[0].x_position;
        cmd.vw = (int16_t)g_remote.rocker[1].x_position;

        // 模式直接使用Switch0
        cmd.mode = g_remote.Switch[0];

        // 保持原有的舵机角度（业务逻辑在Servo_control.c中处理）
        for (int j = 0; j < 4; j++)
        {
          cmd.servo_angle[j] = g_cmd.servo_angle[j];
        }

        // 选中的舵机（业务逻辑在Servo_control.c中处理）
        cmd.selected_servo = g_cmd.selected_servo;

        // 吸盘状态（业务逻辑在Servo_control.c中处理）
        cmd.suction_cup = g_cmd.suction_cup;

        // 一次性更新全局控制命令
        g_cmd = cmd;
      }
    }
  }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* DMA空闲中断回调：接收蓝牙*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart2)
  {
    ProcessBleData(rx_buf, Size); // 处理数据
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buf, RX_BUF_LEN);
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buf, RX_BUF_LEN); // 开启蓝牙DMA接收

  // ===== 电机对象初始化 =====
  Motor_Init(&motors[0], &htim2, &htim8, TIM_CHANNEL_1,
             GPIOA, GPIO_PIN_4,
             GPIOA, GPIO_PIN_5);

  Motor_Init(&motors[1], &htim3, &htim8, TIM_CHANNEL_2,
             GPIOC, GPIO_PIN_5,
             GPIOC, GPIO_PIN_4);

  Motor_Init(&motors[2], &htim4, &htim8, TIM_CHANNEL_3,
             GPIOB, GPIO_PIN_12,
             GPIOB, GPIO_PIN_13);

  Motor_Init(&motors[3], &htim5, &htim8, TIM_CHANNEL_4,
             GPIOB, GPIO_PIN_15,
             GPIOB, GPIO_PIN_14);

  // ===== PID 初始化，解决每个电机差异=====
  motors[0].PidInit(&motors[0],
                    POSITION,
                    3600 - 1,          // max_out
                    1800 - 1,          // max_iout
                    5.0f, 2.0f, 1.0f); // 5.0f, 2.0f, 1.0f

  motors[1].PidInit(&motors[1],
                    POSITION,
                    3600 - 1,          // max_out
                    1800 - 1,          // max_iout
                    5.0f, 2.0f, 1.0f); // 25.0f, 5.0f, 0.0f

  motors[2].PidInit(&motors[2],
                    POSITION,
                    3600 - 1,          // max_out
                    1800 - 1,          // max_iout
                    5.0f, 2.0f, 1.0f); // 25.0f, 5.0f, 0.0f

  motors[3].PidInit(&motors[3],
                    POSITION,
                    3600 - 1,          // max_out
                    1800 - 1,          // max_iout
                    5.0f, 2.0f, 1.0f); // 25.0f, 5.0f, 0.0f

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 while (1)
 {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
 }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
