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
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CHECK_STATUS(x)  if(x!=0)return(x)

#define MAX_RETRY 5


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

bool TEST_ALL_FLAG = 0;
bool CAN_COMMUNICATION_FLAG = 0;
bool SLAVE_COMMUNICATION_FLAG = 0;
bool LTC2949_COMMUNCIATION_FLAG = 0;
bool EEPROM_TEST_FLAG = 0;
bool CELL_BALANCING_FLAG = 0;
bool CONTACTOR_MOSFETS_CONTROL = 0;

TestType currentTest = WAIT;
// TestType currentTest = 10;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

uint8_t CAN_CommunicationTest(void);
uint8_t slaveCommunicationTest(void);
uint8_t LTC2949_CommunicationTest(void);
uint8_t eepromTest(void);
uint8_t cellBalancingMosfetsTest(void);
uint8_t ContactorMOSFETSTests(void);
uint8_t TestAll(void);
uint8_t sendOnCAN(uint32_t CANIDValue, uint8_t *txCANData);
uint8_t checkGPIOInputs(uint8_t test);
uint8_t canData[8] = {0};

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  uint8_t status = 0;
	  uint8_t retry = 0;
	  char *msg  = "WAITING FOR TEST INPUT!";



	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
	HAL_Delay(250);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_Delay(250);



	  switch (currentTest) {

	  	  	  	  case WAIT:
	  	  	    	status = HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, strlen(msg), 0x700);
	  	  	    	HAL_Delay(200);
					uint8_t canDataFrame[8] = {0};
					canDataFrame[1] = 1;

    				status = sendOnCAN(TEST_ID1, canDataFrame);
	  	  	    	break;


	              case TEST_ALL:
	                  if (retry < MAX_RETRY){
	                      status = TestAll();
	                      HAL_Delay(10);
	                      CHECK_STATUS(status);
	                  }
	                  break;

	              case CAN_COMMUNICATION:
	                  if (retry < MAX_RETRY){
	                      status = CAN_CommunicationTest();
	                      CHECK_STATUS(status);
	                      retry++;
	                  }
	                  break;

	              case SLAVE_COMMUNICATION:
	                  if (retry < MAX_RETRY){
	                      status = slaveCommunicationTest();
	                      CHECK_STATUS(status);
	                      retry++;
	                  }
	                  break;

	              case LTC2949_COMMUNICATION:
	                  if (retry < MAX_RETRY){
	                      status = LTC2949_CommunicationTest();
	                      CHECK_STATUS(status);
	                      retry++;
	                  }
	                  break;

	              case CELL_BALANCING:
	                  if (retry < MAX_RETRY){
	                      status = cellBalancingMosfetsTest();
	                      CHECK_STATUS(status);
	                      retry++;
	                  }
	                  break;

	              case EEPROM_TEST:
	                  if (retry < MAX_RETRY){
	                      status = eepromTest();
	                      CHECK_STATUS(status);
	                      retry++;
	                  }
	                  break;

	              case CONTACTOR_MOSFETS:
	                  if (retry < MAX_RETRY){
	                      status = ContactorMOSFETSTests();
	                      CHECK_STATUS(status);
	                      retry++;
	                  }
	                  break;

				 case TEMPERATURE_ACCURACY:
	                  if (retry < MAX_RETRY){
	                      status = temperatureAccuracyTests();
	                      CHECK_STATUS(status);
	                      retry++;
	                  }
	                  break;
	          }

  }

  return 0;
}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_LPUART1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}
/* USER CODE BEGIN 4 */

uint8_t CAN_CommunicationTest(void)
{
	uint8_t status = 0;
	uint8_t canDataFrame[8] = {0};
	canDataFrame[0] = 1;

	status = sendOnCAN(TEST_ID1, canDataFrame);
	HAL_Delay(200);
    if(CAN_COMMUNICATION_FLAG)
    {
    	//CAN Handshake successful
    	char *msg  = "CAN HANDSHAKE SUCCESSFUL!";
    	status = HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, strlen(msg), 0x700);

    }

    return status;
}

uint8_t slaveCommunicationTest(void)
{
	//Will sent a query on CAN and Marvel will perform the test and send a response
	uint8_t status = 0;
	uint8_t canDataFrame[8] = {0};
	canDataFrame[1] = 1;

    status = sendOnCAN(TEST_ID1, canDataFrame);
	HAL_Delay(200);
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
    if(SLAVE_COMMUNICATION_FLAG)
    {
    	//SLAVE COMMUNICATION successful
    	char *msg  = "SLAVE COMMUNICATION SUCCESSFUL!";
    	status = HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, strlen(msg), 0x700);
    }

    return status;
}

uint8_t LTC2949_CommunicationTest(void)
{
	uint8_t status = 0;
	uint8_t canDataFrame[8] = {0};
	canDataFrame[2] = 1;

	status = sendOnCAN(TEST_ID1, canDataFrame);		
	HAL_Delay(200);

    if(LTC2949_COMMUNCIATION_FLAG)
    {
    	//SLAVE COMMUNICATION successful
    	char *msg  = "LTC2949 COMMUNICATION SUCCESSFUL!";
    	status = HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, strlen(msg), 0x700);
    }

    return status;

}
//
uint8_t cellBalancingMosfetsTest(void)
{
	uint8_t status = 1;
	uint8_t canDataFrame[8] = {0};
	canDataFrame[3] = 1;
	//Hardware Flags will get when GPIO interrupt is triggered

	status = sendOnCAN(TEST_ID1, canDataFrame);
	HAL_Delay(200);

	checkGPIOInputs(CELL_BALANCING);
    if(CELL_BALANCING_FLAG) //will get set if the corresponding GPIOs connected to the TPs are getting positive signal
    {
    	//Cell Balancing Mosfet Switching successful
    	char *msg  = "CellBalancing TEST SUCCESSFUL!";
    	status = HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, strlen(msg), 0x700);
    }

    return status;
}

uint8_t eepromTest(void)
{
	uint8_t status = 1;
	uint8_t canDataFrame[8] = {0};
	canDataFrame[4] = 1;

	status = sendOnCAN(TEST_ID1, canDataFrame);
	HAL_Delay(200);

    if(EEPROM_TEST_FLAG)
    {
    	//Cell Balancing Mosfet Switching successful
    	char *msg  = "EEPROM TEST SUCCESSFUL!";
    	status = HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, strlen(msg), 0x700);
    }

    return status;

}

uint8_t ContactorMOSFETSTests(void)
{
	uint8_t status = 1;
	uint8_t canDataFrame[8] = {0};
	canDataFrame[5] = 1;

    status = sendOnCAN(TEST_ID1, canDataFrame);
	HAL_Delay(200);

    if(CONTACTOR_MOSFETS_CONTROL)
    {
    	//Cell Balancing Mosfet Switching successful
    	char *msg  = "CONTACTOR MOSFETS TEST SUCCESSFUL!";
    	status = HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, strlen(msg), 0x700);
    }

    return status;

}

uint8_t TestAll(void)
{
	uint8_t status = 0;
	CAN_CommunicationTest();
	HAL_Delay(100);
	slaveCommunicationTest();
	HAL_Delay(100);
	LTC2949_CommunicationTest();
	HAL_Delay(100);
	eepromTest();
	cellBalancingMosfetsTest();
	HAL_Delay(100);
	ContactorMOSFETSTests();

    if(CAN_COMMUNICATION_FLAG && SLAVE_COMMUNICATION_FLAG && LTC2949_COMMUNCIATION_FLAG && EEPROM_TEST_FLAG && CELL_BALANCING_FLAG)
    {
    	//SLAVE COMMUNICATION successful
    	char *msg  = "HARDWARE TEST SUCCESSFUL!";
    	status = HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, strlen(msg), 0x700);
    } 

	return status;
}

uint8_t temperatureAccuracyTests(void){
	uint8_t status = 0;
	uint8_t canDataFrame[8] = {0};
	canDataFrame[0] = 1;

	status = sendOnCAN(TEST_ID1, canDataFrame);
	HAL_Delay(200);
    if(CAN_COMMUNICATION_FLAG)
    {
    	//CAN Handshake successful
    	char *msg  = "CAN HANDSHAKE SUCCESSFUL!";
    	status = HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, strlen(msg), 0x700);

    }

    return status;
}

uint8_t checkGPIOInputs(uint8_t test)
{
	uint8_t status = 0;
	uint8_t MOSFET1, MOSFET2, MOSFET3, MOSFET4, MOSFET5, MOSFET6, MOSFET7, MOSFET8, MOSFET9, MOSFET10,
	        MOSFET11, MOSFET12, MOSFET13, MOSFET14, MOSFET15, MOSFET16, MOSFET17, MOSFET18, MOSFET19, MOSFET20, contactorPos, contactorNeg, ContactorPreChr = 0;


	switch(test)
	{
	case CELL_BALANCING:
		{
			//cell Balancing MOSFETs
			MOSFET1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
			MOSFET2 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);
			MOSFET3 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4);
			MOSFET4 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5);
			MOSFET5 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
			MOSFET6 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
			MOSFET7 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);
			MOSFET8 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9);
			MOSFET9 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10);
			MOSFET10 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11);
			MOSFET11 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12);
			MOSFET12 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
			MOSFET13 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14);
			MOSFET14 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15);
			MOSFET15 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
			MOSFET16 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
			MOSFET17 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
			MOSFET18 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
			MOSFET19 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
			MOSFET20 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);

		if(MOSFET1 && MOSFET2 && MOSFET3 && MOSFET4 && MOSFET5 && MOSFET6 && MOSFET7 && MOSFET8 && MOSFET9 && MOSFET10 &&
		   MOSFET11 && MOSFET12 && MOSFET13 && MOSFET14 && MOSFET15 && MOSFET16 && MOSFET17 && MOSFET18 && MOSFET19 && MOSFET20)
		{
			CELL_BALANCING_FLAG = 1;
		}
		else
		{
			//mention the MOSFETS not working and publish over UART
			uint8_t MosfetsCode = (MOSFET1 | MOSFET2 | MOSFET3 | MOSFET4 | MOSFET5 | MOSFET6 | MOSFET7 | MOSFET8 | MOSFET9 | MOSFET10 |
			                        MOSFET11 | MOSFET12 | MOSFET13 | MOSFET14 | MOSFET15 | MOSFET16 | MOSFET17 | MOSFET18 | MOSFET19 | MOSFET20);

			char message[100];
			// Formatting the message
			sprintf(message, "Faulty Cell Balancing Mosfets: %d", MosfetsCode);
			status = HAL_UART_Transmit(&hlpuart1, (uint8_t *)message, strlen(message), 0x700);
		}
		break;
	}
	case CONTACTOR_MOSFETS:
	{
		//Contactor MOSFETS
		contactorPos = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
		contactorNeg = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
		ContactorPreChr = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);

		if(contactorPos && contactorNeg && ContactorPreChr)
		{
			CONTACTOR_MOSFETS_CONTROL = 1;
		}
		else
		{
			//mention the MOSFETS not working and publish over UART
			uint8_t MosfetsCode = (contactorPos | contactorNeg | ContactorPreChr);
			char message[100];
			// Formatting the message
			sprintf(message, "Faulty Cell Balancing Mosfets: %d", MosfetsCode);
			status = HAL_UART_Transmit(&hlpuart1, (uint8_t *)message, strlen(message), 0x700);
		}
		break;
	}

}

	return status;
}

//Send On CAN
uint8_t sendOnCAN(uint32_t CANIDValue, uint8_t *txCANData)
{
	uint8_t status = 1;
	CAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.DLC   = 8;
	pTxHeader.RTR   = CAN_RTR_DATA;
    pTxHeader.IDE   = CAN_ID_STD;
    pTxHeader.StdId = CANIDValue;


    status = HAL_CAN_AddTxMessage(&hcan1, &pTxHeader, (uint8_t *)txCANData, (uint32_t *)CAN_TX_MAILBOX0);
    return status;
}

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
