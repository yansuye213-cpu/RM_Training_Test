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
static float servo_angle[4] = {90, 90, 90, 90};
static int selected_servo = -1;
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
  if (size < BLE_FRAME_LEN)
    return;

  for (uint16_t i = 0; i + BLE_FRAME_LEN <= size; i++)
  {
    if (buf[i] == 0xFE && buf[i + BLE_FRAME_LEN - 1] == 0xFD)
    {
      if (uart_to_remote(&buf[i]) == REMOTE_OK)
      {
        control_cmd_t cmd;
        cmd.mode = g_remote.Switch[0]; // 使用统一 volatile g_remote

        int selected_servo = -1;
        for (int j = 0; j < 4; j++)
        {
          if (g_remote.Button[j])
          {
            selected_servo = j;
            cmd.selected_servo = j;
          }
        }

        for (int j = 0; j < 4; j++)
          cmd.servo_angle[j] = servo_angle[j];

        if (cmd.mode == 0)
        { // 底盘模式
          cmd.vx = (int16_t)g_remote.rocker[0].y_position;
          cmd.vy = (int16_t)g_remote.rocker[0].x_position;
          cmd.vw = (int16_t)g_remote.rocker[1].y_position;
        }
        else
        { // 舵机模式
          cmd.vx = cmd.vy = cmd.vw = 0;
          if (selected_servo >= 0)
          {
            float delta = g_remote.rocker[1].x_position * 0.05f;
            servo_angle[selected_servo] += delta;
            if (servo_angle[selected_servo] < 0)
              servo_angle[selected_servo] = 0;
            if (servo_angle[selected_servo] > 180)
              servo_angle[selected_servo] = 180;
            cmd.servo_angle[selected_servo] = servo_angle[selected_servo];
          }
        }

        g_cmd = cmd; // 更新全局控制结构
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
  if (huart == &huart3)
  {
    ProcessBleData(rx_buf, Size); // 处理数据
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, RX_BUF_LEN);
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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, RX_BUF_LEN); // 开启蓝牙DMA接收

  // ===== 电机对象初始化 =====
  Motor_Init(&motors[0], &htim2, &htim8, TIM_CHANNEL_1,
             GPIOA, GPIO_PIN_4,
             GPIOA, GPIO_PIN_5);

  Motor_Init(&motors[1], &htim3, &htim8, TIM_CHANNEL_2,
             GPIOC, GPIO_PIN_4,
             GPIOC, GPIO_PIN_5);

  Motor_Init(&motors[2], &htim4, &htim8, TIM_CHANNEL_3,
             GPIOB, GPIO_PIN_12,
             GPIOB, GPIO_PIN_13);

  Motor_Init(&motors[3], &htim5, &htim8, TIM_CHANNEL_4,
             GPIOB, GPIO_PIN_14,
             GPIOB, GPIO_PIN_15);

  // ===== PID 初始化=====
  for (int i = 0; i < 4; i++)
  {
    motors[i].PidInit(&motors[i],
                      POSITION,
                      3600 - 1,           // max_out
                      1800 - 1,           // max_iout
                      5.0f, 2.0f, 0.0f); // 5.0f, 2.0f, 0.0f
  }
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
