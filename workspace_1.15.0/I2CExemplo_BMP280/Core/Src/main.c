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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

float tfine; //Variavel  global para BMP280

int16_t temperatura; //Temperatura em graus celsius com precisão de duas casas decimais
float temperaturaf; //Armazena Temperatura Final
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

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int BMP280_Temp()
{
	  uint8_t Calib0_reg=0x88; // registrador de calibração C0
	  uint8_t digi_t1[2]; //Variável para receber dois bytes de calibração 0x88 e 0x89

	  HAL_I2C_Master_Transmit(&hi2c1, 0xEC, &Calib0_reg, 1, 20); //Envia o endereço do dispositivo e do registrador
	  HAL_I2C_Master_Receive(&hi2c1, 0xEC, digi_t1, 2, 20); //Recebe os dois bytes de calibração

	  uint8_t Calib2_reg=0x8A; // registrador de calibração C3
	  uint8_t digi_t2[2]; //Variável para receber dois bytes de calibração 0x8A e 0x8B

	  HAL_I2C_Master_Transmit(&hi2c1, 0xEC, &Calib2_reg, 1, 20); //Envia o endereço do dispositivo e do registrador
	  HAL_I2C_Master_Receive(&hi2c1, 0xEC, digi_t2, 2, 20); //Recebe os dois bytes de calibração

	  uint8_t Calib3_reg=0x8C; // registrador de calibração C4
	  uint8_t digi_t3[2]; //Variável para receber dois bytes de calibração 0x8C e 0x8D

	  HAL_I2C_Master_Transmit(&hi2c1, 0xEC, &Calib3_reg, 1, 20); //Envia o endereço do dispositivo e do registrador
	  HAL_I2C_Master_Receive(&hi2c1, 0xEC, digi_t3, 2, 20); //Recebe os dois bytes de calibração

	  //Recebe os calores de calibração
	  uint16_t digit1=digi_t1[1]; //atribui os digitos mais significativos ao final dos 16 bits
	  digit1=(digit1<<8)+digi_t1[0]; //desloca os digitos mais significativos 8 bits para a esquerda e adiciona os 8 bits menos significativos para formar o primeiro valor
	  int16_t digit2=digi_t2[1]; //atribui os digitos mais significativos ao final dos 16 bits
	  digit2=(digit2<<8)+digi_t2[0]; //desloca os digitos mais significativos 8 bits para a esquerda e adiciona os 8 bits menos significativos para formar o segundo valor
	  int16_t digit3=digi_t3[1]; //atribui os digitos mais significativos ao final dos 16 bits
	  digit3=(digit3<<8)+digi_t3[0]; //desloca os digitos mais significativos 8 bits para a esquerda e adiciona os 8 bits menos significativos para formar o t erceiro valor

	  uint8_t Temp_reg_start=0xFA;
	  uint8_t Temp_data[3];

	  HAL_I2C_Master_Transmit(&hi2c1, 0xEC, &Temp_reg_start, 1, 20);
	  HAL_I2C_Master_Receive(&hi2c1, 0xEC, Temp_data, 3, 20);

	  uint8_t Pressure_reg_start=0xF7;
	  uint8_t Pressure_data[3];

	  HAL_I2C_Master_Transmit(&hi2c1, 0xEC, &Pressure_reg_start, 1, 20);
	  HAL_I2C_Master_Receive(&hi2c1, 0xEC, Pressure_data, 3, 20);

	  int32_t utc; //Variavel de leitura de temperatura vindo dos dados
	  utc=(Temp_data[0]<<12)+(Temp_data[1]<<4)+Temp_data[2]; //Totalizando os 20 bits (8 bits em Temp_data[0], 8 bits em  Temp_data[1] e 4 bits em  Temp_data[2] coforme inficado no datasheet do sensor BMP280

	  //Seguindo o  algoritmo de cálculo de temperatura indicado na planilha da BOSCH

	  float ut=utc;
	  float dig_t1 = digit1;
	  float dig_t2 = digit2;
	  float dig_t3 = digit3;

	  //Calculo datasheet pg. 23
	  float var1,var2, tempbmp280;
	  int16_t tempbmp280i;
	  var1 = ((ut)/16384-(dig_t1)/1024)*(dig_t2);
	  var2 = (((ut)/131072-(dig_t1)/8192)*((ut)/131072-(dig_t1)/8192))*(dig_t3);
	  tfine=var1+var2;
	  tempbmp280 = (tfine/5120);
	  tempbmp280i = (tempbmp280*100); // Passando para inteiro com duas casas decimais
	  HAL_Delay(10); // Aguarda 10ms
	  return tempbmp280i;
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  if(HAL_I2C_IsDeviceReady(&hi2c1, 0xEC, 2,10)== HAL_OK)
  {
  	HAL_GPIO_WritePin(GPIOB,LD1_Pin,1);
  	HAL_Delay(1);
  }

  //Reset do dispositivo
  uint8_t dreset[2];
  dreset[0]=0xE0;
  dreset[1]=0xB6;
  HAL_I2C_Master_Transmit(&hi2c1, 0xEC, dreset, 2, 20);
  HAL_Delay(1);

  //Ler o ID do dspositivo

  uint8_t ID_reg=0xD0;
  uint8_t ID_BMP280;

  HAL_I2C_Master_Transmit(&hi2c1, 0xEC, &ID_reg, 1, 20);
  HAL_I2C_Master_Receive(&hi2c1, 0xEC, &ID_BMP280, 1, 20);

  //Configuraçao Basica BMP280
  uint8_t Config_reg[2];
  Config_reg[0]=0xF4;
  Config_reg[1]=0b00100111;

  HAL_I2C_Master_Transmit(&hi2c1, 0xEC, Config_reg, 2, 20);
  HAL_Delay(1);

  //Ler o temperatura do dspositivo



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  temperatura = BMP280_Temp();
	  temperaturaf=temperatura;
	  temperaturaf = temperaturaf/100;

	  HAL_Delay(1000);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0010061A;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD1_Pin */
  GPIO_InitStruct.Pin = LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
