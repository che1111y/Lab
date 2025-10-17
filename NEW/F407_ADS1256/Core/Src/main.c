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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ads1256.h"
#include "stdio.h"
#include "delay.h"
#include "fuzzyPID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DOF 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f)
{
  uint8_t temp[1] = {ch};
  HAL_UART_Transmit(&huart1, temp, 1, 2);
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
  float temp[8], R[8], V[8];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  ADS1256_GPIO_Init();												//ads1256引脚初始�??
	// delay_ms (500);
	ADS1256_CfgADC(PGA_1, DATARATE_100);	// 配置ADC参数�?? 增益1:1, 数据输出速率 15Hz  	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  // MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_RESET);

  // Default fuzzy rule base of delta kp/ki/kd
  int rule_base[][qf_default] = {
          //delta kp rule base
          {PB, PB, PM, PM, PS, ZO, ZO},
          {PB, PB, PM, PS, PS, ZO, NS},
          {PM, PM, PM, PS, ZO, NS, NS},
          {PM, PM, PS, ZO, NS, NM, NM},
          {PS, PS, ZO, NS, NS, NM, NM},
          {PS, ZO, NS, NM, NM, NM, NB},
          {ZO, ZO, NM, NM, NM, NB, NB},
          //delta ki rule base
          {NB, NB, NM, NM, NS, ZO, ZO},
          {NB, NB, NM, NS, NS, ZO, ZO},
          {NB, NM, NS, NS, ZO, PS, PS},
          {NM, NM, NS, ZO, PS, PM, PM},
          {NM, NS, ZO, PS, PS, PM, PB},
          {ZO, ZO, PS, PS, PM, PB, PB},
          {ZO, ZO, PS, PM, PM, PB, PB},
          //delta kd rule base
          {PS, NS, NB, NB, NB, NM, PS},
          {PS, NS, NB, NM, NM, NS, ZO},
          {ZO, NS, NM, NM, NS, NS, ZO},
          {ZO, NS, NS, NS, NS, NS, ZO},
          {ZO, ZO, ZO, ZO, ZO, ZO, ZO},
          {PB, PS, PS, PS, PS, PS, PB},
          {PB, PM, PM, PM, PS, PS, PB}};
  // Default parameters of membership function
  int mf_params[4 * qf_default] = {-8, -8, -4,  0,
                                   -8, -4, -1,  0,
                                   -4, -1,  0,  0,
                                   -1,  0,  1,  0,
                                    0,  1,  4,  0,
                                    1,  4,  8,  0,
                                    4,  8,  8,  0};
  // Default parameters of pid controller
  float fuzzy_pid_params[DOF][pid_params_count] = {{20.4f,   0,     0,    5,     0.1f, 0, 1},//kp, ki, kd, integral_limit, dead_zone, feed_forward,
                                                   {4.0f,   0,     0,    5,     0.1f, 0, 1},
                                                   {1.1f,   0,     0,    0,     0,    0, 1},
                                                   {2.4f,   0,     0,    0,     0,    0, 1},
                                                   {1.2f,   0,     0,    0,     0,    0, 1},
                                                   {1.2f,   0,     0,    0,     0,    0, 1}};
  float pid_params[DOF][pid_params_count] = {{10.2f,   0,     0,    5,     0.1f, 0, 1}};
  // Obtain the PID controller vector according to the parameters
  struct PID **fuzzy_pid_vector = fuzzy_pid_vector_init(
                    fuzzy_pid_params, 
                    1.0f, 
                    4, 
                    1, 
                    0, 
                    mf_params, 
                    rule_base, 
                    DOF
                    );//delta_kPID参数的调整幅度比例, mf_type隶属度函数类型, fo_type模糊化方法, df_type解模糊化方法
  
  struct PID **fan_pid_vector = pid_vector_init(
                    pid_params, 
                    DOF
                    );

  printf("output:\n");
  // int control_id = 5;
  int control_fan = 0, control_teg = 1, fan_out, teg_out;
  // float real = 0.0;
  float idea_temp_cold = 15.0f, idea_temp_hot = idea_temp_cold*1.2f;
  printf("cold_idea_value: %f, hot_idea_value: %f\n\r", idea_temp_cold, idea_temp_hot);
  bool direct[DOF] = {false, false, true, true, true, true};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // float err = idea - real;
    // int out = fuzzy_pid_motor_pwd_output(real, idea, direct[control_id], pid_vector[control_id]);
    // real += (float) (out - middle_pwm_output) / (float) middle_pwm_output * (float) max_error * 0.1f;
    // printf("idea:%f,out:%d,real:%f,err:%f\n", idea, out, real, err);

    for(int i=0;i<8;i++)
    {
      // V[i] = ADS1256_GetVolt(i, 0);
      // printf("Voltage%d: %4.3fV\r\n",i, V[i]);
      // R[i] = Get_Rntc(i, 10);
      // printf("Rntc%d: %4.3fO\r\n",i, R[i]);
      temp[i] = GetAccuraryTemperature(Get_Rntc(i, 10));
		  // printf("Temperature%d: %4.5fC\r\n",i, temp[i]);
      // printf("\r\n");
      delay_ms(200);
    }
        teg_out = fuzzy_pid_motor_pwd_output(temp[0], idea_temp_cold, direct[control_teg], fuzzy_pid_vector[control_teg]);
        fan_out = pid_motor_pwd_output(temp[1], idea_temp_hot, direct[control_fan], fan_pid_vector[control_fan]);

        printf("fanOutput:%d,Real:%f\n\r", (int)(fan_out), temp[1]);
        printf("tegOutput:%d,Real:%f\n\r", (int)(teg_out), temp[0]);

    // delete_pid_vector(pid_vector, DOF);

    __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1, fan_out);//min:14  max:50
    __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2, teg_out);//    
    // __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3, 40);//
    // __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4, 40);//

		// delay_ms(500);
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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  if (htim->Instance == TIM14) {
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
