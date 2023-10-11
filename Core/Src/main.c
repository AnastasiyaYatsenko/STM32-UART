/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BUFFER_SIZE 16

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

union ReadRequestDatagram {
    struct {
      uint32_t sync :4;
      uint32_t reserved :4;
      uint32_t serial_address :8;
      uint32_t register_address :7;
      uint32_t rw :1;
      uint32_t crc :8;
    };
    uint32_t bytes;
} req;

union WriteReadReplyDatagram {
    struct {
      uint64_t sync :4;
      uint64_t reserved :4;
      uint64_t serial_address :8;
      uint64_t register_address :7;
      uint64_t rw :1;
      uint64_t data :32;
      uint64_t crc :8;
    };
    uint64_t bytes;
  } ans[24];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t CalcCRC(uint64_t ddd)
{
    uint8_t crc = 0;
    uint8_t byte;
    for (uint8_t i = 0; i < 3; ++i) {
        byte = (ddd >> (i * 8)) & 255;
        for (uint8_t j = 0; j < 8; ++j) {
            if ((crc >> 7) ^ (byte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = crc << 1;
            }
            byte = byte >> 1;
        }
    }
    return crc;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t transmitBuffer[BUFFER_SIZE];
  uint8_t receiveBuffer[BUFFER_SIZE];
  uint8_t chr[2];
  uint8_t str[255],cc[24];
  HAL_StatusTypeDef status[24],status1[24];




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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

/*
  sprintf(str, "Ква! Печатаем!\r\n\0");
  HAL_UART_Transmit(&huart1, str, strlen(str), 30);

  for (unsigned char i = 0; i < BUFFER_SIZE; i++) {
      transmitBuffer[i] = i + 65;
      receiveBuffer[i] = 0;
  }
*/
//  HAL_UART_Receive_IT(&huart1, receiveBuffer, BUFFER_SIZE);
//  HAL_UART_Transmit_IT(&huart2, transmitBuffer, BUFFER_SIZE);
//  HAL_Delay(10); //задержка в м�?
//  for (unsigned char i = 0; i < BUFFER_SIZE; i++)
//      receiveBuffer[i] = 0;
/*  HAL_UART_Transmit_IT(&huart1, transmitBuffer, BUFFER_SIZE);
//  HAL_Delay(2); //задержка в м�?
  uint32_t sr = huart2.Instance->SR;
  uint32_t dr = huart2.Instance->DR;
  uint32_t brr = huart2.Instance->BRR;

  HAL_UART_Receive_IT(&huart2, receiveBuffer, BUFFER_SIZE);
  HAL_Delay(2); //задержка в м�?
  sr = huart2.Instance->SR & 0x20;

  brr = huart2.Instance->BRR*115200;
*/
//  HAL_Delay(100); //задержка в м�?

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
/*  uint16_t i = 0;
  uint16_t cnt=0;
  HAL_StatusTypeDef status;
  while (1)
  {
      if (cnt++>10) {
	    HAL_UART_Transmit(&huart1, ".", 1, 1);
	    cnt=0;
      }
      status = HAL_UART_Receive(&huart1, chr, 1, 500);
//	  if( HAL_UART_Receive(&huart1, chr, 1, 500) == HAL_OK ) {
	  if( status == HAL_OK ) {
		  sprintf(str, "[%c] [%d] %d\r\n\0", chr[0],chr[0],i++);
		  HAL_UART_Transmit(&huart1, str, strlen(str), 30);
	  } else {
		  sprintf(str, "\r\n TIMEOUT! status: %d\r\n\0", status);
		  HAL_UART_Transmit(&huart1, str, strlen(str), 30);
	  }
//	  sprintf(str, "Значение переменной i=[%d]\r\n\0", i++);
//	  HAL_UART_Transmit(&huart1, str, strlen(str), 30);
	  HAL_Delay(0);
  }
*/

  req.bytes            = 0;
  req.sync             = 5;
  req.reserved         = 0;
  req.serial_address   = 0;
  req.register_address = 0x6c;
  req.rw               = 0;
  req.crc              = CalcCRC(req.bytes);

  uint32_t regs[] = { 0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
		              0x10,0x11,0x12,0x13,0x14,0x22,0x40,0x41,
					  0x42,0x68,0x6a,0x6c,0x6f,0x70,0x71,0x72};
  uint32_t sr[24][5],dr[5];

  sprintf(str, "\x1b[2H\x1b[2J\0");
  HAL_UART_Transmit(&huart1, str, strlen(str), 30);

  for (int i=0; i<sizeof(regs)/4; i++) {
  	  req.register_address = regs[i];
  	  req.crc              = CalcCRC(req.bytes);

  //	  status[i] = HAL_UART_Receive(&huart2, (uint8_t *)(& ans[i].bytes), 8, 2);
  	  sr[i][0] = huart2.Instance->SR;

  	  HAL_HalfDuplex_EnableTransmitter(&huart2);
  	  HAL_UART_Transmit(&huart2, (uint8_t *)(& req), 4, 1);
  	  sr[i][1] = huart2.Instance->SR;

  //	  status1[i] = HAL_UART_AbortReceive(&huart2); // Прерываем прием данных
  	  sr[i][2] = huart2.Instance->SR;

  	  cc[i]=0;
  	  HAL_HalfDuplex_EnableReceiver(&huart2);
  //	  status1[i] = HAL_UART_Receive(&huart2, cc+i, 1, 1); // == HAL_OK;
  	  sr[i][3] = huart2.Instance->SR;

  	  status[i] = HAL_UART_Receive(&huart2, (uint8_t *)(& ans[i].bytes), 8, 12);
  	  sr[i][4] = huart2.Instance->SR;
    }

  for (int i=0; i<sizeof(regs)/4; i++) {
      sprintf(str, "Статусы: %02x %02x %02x %02x %02x  CC:[%d] %02x  \0",sr[i][0],sr[i][1],sr[i][2],sr[i][3],sr[i][4],status1[i],cc[i]);
      HAL_UART_Transmit(&huart1, str, strlen(str), 30);
      if (status[i]==0) {
	      sprintf(str, "Чтение регистра %02x: %08x\r\n\0",regs[i],ans[i].data);
      } else if (status[i]==3) {
	      sprintf(str, "Ошибка чтения регистра %02x: timeout\r\n\0",req.register_address);
      } else {
	      sprintf(str, "Ошибка чтения регистра %02x: %d\r\n\0",req.register_address,status[i]);
      }
	  HAL_UART_Transmit(&huart1, str, strlen(str), 30);
	  //	  HAL_Delay(1000);
  }

  HAL_Delay(100000);
/////  HAL_StatusTypeDef HAL_HalfDuplex_Init (UART_HandleTypeDef * huart)
////  HAL_StatusTypeDef HAL_HalfDuplex_EnableTransmitter (UART_HandleTypeDef * huart)
////  HAL_StatusTypeDef HAL_HalfDuplex_EnableReceiver (UART_HandleTypeDef * huart)



  while (1)
  {
//	 HAL_UART_Transmit(&huart2, (uint8_t *)(& req), 4, 1);
//	 HAL_Delay(0); //задержка в м�?

//	 status1 = HAL_UART_AbortReceive(&huart2); // Прерываем прием данных
//     HAL_UART_AbortTransmit(&huart2); // Прерываем передачу данных

	    // ожидание паузы в передаче данных
//    if(
//    		HAL_UART_Receive(&huart2, str, 1, 1); // == HAL_OK;
//			) continue;

    // ожидание 1го байта команды
//    while ( HAL_UART_Receive(&huart2, (uint8_t *)(& ans), 1, 1) != HAL_OK ) ;

    // прием о�?тавших�?�? 2х байтов команды
//    if(
//    status = HAL_UART_Receive(&huart2, (uint8_t *)(& ans.bytes), 8, 1);
//    != HAL_OK ) continue;

    HAL_Delay(100);
    // проверка команды
//    if ( (str[0] == 0x10) && ((str[0] ^ str[1] ^ 0xe5) != str[2]) ) continue;

    // под�?чет контрольного кода ответа
//    uint16_t sum= 0;
//    for (uint16_t i=0; i<10; i++) sum += * ((uint8_t *)(& par) + i);
//    par.s = sum ^ 0xa1e3;

    // ответ на компьютер
//    HAL_UART_Transmit(&huart2, (uint8_t *)(& par), 12, 30);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
