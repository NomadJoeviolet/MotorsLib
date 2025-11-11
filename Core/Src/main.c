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
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void CAN_Config(void)
{
  CAN_FilterTypeDef  sFilterConfig;

  /* Configure the CAN Filter */
  sFilterConfig.FilterBank = 0;                      // 过滤器编号，使用一个CAN，则可选0-13；使用两个CAN可选0-27
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // 过滤器模式，掩码模式或列表模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 过滤器位宽
  sFilterConfig.FilterIdHigh = 0x0000;               // 过滤器验证码ID高16位，0-0xFFFF
  sFilterConfig.FilterIdLow = 0x0000;                // 过滤器验证码ID低16位，0-0xFFFF
  sFilterConfig.FilterMaskIdHigh = 0x0000;           // 过滤器掩码ID高16位，0-0xFFFF
  sFilterConfig.FilterMaskIdLow = 0x0000;            // 过滤器掩码ID低16位，0-0xFFFF
  sFilterConfig.FilterFIFOAssignment = CAN_FilterFIFO0; // FIFOx，0或1
  sFilterConfig.FilterActivation = ENABLE;           // 使能过滤器
  sFilterConfig.SlaveStartFilterBank = 14;           // 从过滤器编号，0-27，对于单CAN实例该参数没有意义

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
  }

  /* Start the CAN peripheral */
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    /* Start Error */
  }

  /* Activate CAN RX notification */
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Notification Error */
  }
  //__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  //__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
}

// struct PID {
//   float kp;
//   float ki;
//   float kd;
//   float integral;
//   float last_error;
//   float error;
//   float output;
//   float i_max;
//   float output_max;
// };

// struct DJIMotor {
//   enum {
//     MOTOR_2006,
//     MOTOR_3508,
//     MOTOR_GM6020,
//   } motor_id;
//   int16_t motor_speed;//单位rpm
//   uint16_t motor_raw_angle;
//   float motor_angle;//角度制
//   int16_t motor_current;
//   uint8_t  temp;


//   int16_t ref_motor_speed;
//   uint16_t ref_motor_raw_angle;
//   float ref_motor_angle;
//   int16_t ref_motor_current;

//   int16_t output_value;

//   struct PID pid;
  
// };

// struct DJIMotor test_motor = {
//   .motor_id = MOTOR_3508,
//   .motor_speed = 0,
//   .motor_raw_angle = 0,
//   .motor_angle = 0.0f,
//   .motor_current = 0,
//   .temp = 0,

//   .ref_motor_speed = 0,
//   .ref_motor_raw_angle = 0,
//   .ref_motor_angle = 0.0f,
//   .ref_motor_current = 0,
//   .output_value = 0
// };


// uint8_t CAN_Send_Msg(uint16_t output , uint8_t len)
// {	
//   uint8_t message[8];
//   uint32_t TxMailbox;
//   CAN_TxHeaderTypeDef CAN_TxHeader;

//   // 设置发送参数
//   CAN_TxHeader.StdId = 0x200;                 // 标准标识符(12bit)
//   //CAN_TxHeader.ExtId = 0x00;                 // 扩展标识符(29bit)
//   CAN_TxHeader.IDE = CAN_ID_STD;             // 使用标准帧
//   CAN_TxHeader.RTR = CAN_RTR_DATA;           // 数据帧
//   CAN_TxHeader.DLC = 0x08;                    // 发送长度      
//   //CAN_TxHeader.TransmitGlobalTime = DISABLE;//关闭can的时间触发通信模式

//   // 装载数据
//   memset(message, 0, sizeof(message));
//   message[0] = (output >> 8) & 0xFF;
//   message[1] = output & 0xFF;

//   // 发送CAN消息
//   if(HAL_CAN_AddTxMessage(&hcan1, &CAN_TxHeader, message, &TxMailbox) != HAL_OK) 
//   {
//     return 1;
//   }
  
//   return 0;	
// }

// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
// {// 收到CAN数据会触发接收中断，进入该回调函数
//   uint8_t RxData[8];
//   CAN_RxHeaderTypeDef CAN_RxHeader; 
  
//   if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_RxHeader, RxData) != HAL_OK)
//   {
//     while(1) {};
//   }

//   if(hcan==&hcan1) // 只处理CAN1
//   {
//     // 处理CAN1接收到的数据
//     switch (CAN_RxHeader.StdId) // 标准标识符
//     {
//       case 0x201:
//         // 处理ID为0x201的消息
//         test_motor.motor_raw_angle = (RxData[0] << 8) | RxData[1];
//         test_motor.motor_angle = (float)test_motor.motor_raw_angle/8191.f*360.f;
//         test_motor.motor_speed = (RxData[2] << 8) | RxData[3];
//         test_motor.motor_current = (RxData[4] << 8) | RxData[5];
//         test_motor.temp = RxData[6];
//         break;
//       default:
//         break;
//     }
//   }
// }

// uint16_t PID_Calc(struct PID *pid) {
//   float derror = pid->error - pid->last_error;
//   pid->integral += pid->error;

//   if (pid->integral > pid->i_max) {
//     pid->integral = pid->i_max;
//   } else if (pid->integral < -pid->i_max) {
//     pid->integral = -pid->i_max;
//   }

//   pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * derror;
//   if (pid->output > pid->output_max) {
//     pid->output = pid->output_max;
//   } else if (pid->output < -pid->output_max) {
//     pid->output = -pid->output_max;
//   }

//   pid->last_error = pid->error;
//   return (int16_t)pid->output;
// }

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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  CAN_Config(); // 配置CAN
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14)
  {
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
#ifdef USE_FULL_ASSERT
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
