/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
#include <stdio.h>
#include "pid.h"
#include "led.h"
#include "medicine.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define velocity_set 2000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//Motor motor1;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Motor motor1,motor2;
uint8_t receive_buff[4] = {0,0,0,0};
float foward_distance = 0;
float left_angle = 0;
float right_angle = 0;
float error = 0;
int medicine_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
  return ch;
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  Motor_Init();
  PID_Init();
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //printf("OK");//测试串口
	  //Target_Speed_1 = 5*velocity_set;
	  motor_foward(100,velocity_set/3);//测试电机
	  //Target_Speed_1 = 0;
      HAL_Delay(1000);
	  medicine_flag = get_medicine();
	 HAL_UART_Receive(&huart1,receive_buff,sizeof(receive_buff),0xff);
	  //HAL_Delay(100);
//	  if(-medicine_flag)
//	  {
//		printf("2");
//		led1_off();
//	  }
	  
	  if(receive_buff[0] != 0)
      {
		  //HAL_UART_Transmit(&huart1,(uint8_t*)receive_buff,sizeof(receive_buff),0xffff);
		if((receive_buff[0]=='O')&&(receive_buff[1]=='K')&&(receive_buff[2]=='O')&&(receive_buff[3]=='K'))//OKOK
		  {
			led1_on();
			while(medicine_flag){medicine_flag = get_medicine();}
			printf("0");
			led1_off();
		  }
		else
		{
		  switch(receive_buff[0])
			  {
			  case 'G': //前进
				  if(receive_buff[1] == '+')
					  {
					  foward_distance = (receive_buff[2]-'0')*10 + (receive_buff[3]-'0');
					  motor_foward(foward_distance,velocity_set);
					  }
				  else if(receive_buff[1] == '-')
					  {
					  foward_distance = - (receive_buff[2]-'0')*10 - (receive_buff[3]-'0');
					  motor_foward(foward_distance,-velocity_set);
					  }
				  
				  printf("1");
				  break;
			  case 'L': //左转
				  if(receive_buff[1] == '+'){left_angle = (receive_buff[2]-'0')*10 + (receive_buff[3]-'0');motor_turnleft(left_angle,velocity_set);}
				  else if(receive_buff[1] == '-'){left_angle = - (receive_buff[2]-'0')*10 - (receive_buff[3]-'0');motor_turnleft(left_angle,-velocity_set);}
			      printf("1");
				  break;
			  case 'R': //右转
				  if(receive_buff[1] == '+'){right_angle = (receive_buff[2]-'0')*10 + (receive_buff[3]-'0');motor_turnright(right_angle,velocity_set);}
				  else if(receive_buff[1] == '-'){right_angle = - (receive_buff[2]-'0')*10 - (receive_buff[3]-'0');motor_turnright(right_angle,-velocity_set);}
			      printf("1");
				  break;
			  case 'E': //纠正
				  if(receive_buff[1] == '+'){error = (receive_buff[2]-'0')*10 + (receive_buff[3]-'0');}
				  else if(receive_buff[1] == '-'){error = - (receive_buff[2]-'0')*10 - (receive_buff[3]-'0');}
				  fix_error(error,velocity_set);
			      break;
			  }
		}
		memset(receive_buff,0,20);
	  }
//	//HAL_UART_Transmit(&huart1, (uint8_t *)(int)motor1.speed, sizeof((int)motor1.speed), 0xffff);//传输速度信号
//	  HAL_Delay(100);
	  //Speed_PID_Realize()
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

