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
  *****************************************************************************
  test comment
  test comment #2
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void CAN1_CAN2_broadcast(uint32_t can_id, uint8_t* x, uint8_t length) {

    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[8];

    // Преобразование входящего сообщения от системы управления
    // id = 0x150 широковещательный запуск/остановка
    if (can_id == 0x150) {
        TxHeader.DLC = 8;
        TxHeader.IDE = CAN_ID_EXT;
        TxHeader.ExtId = 0x00;
        TxHeader.RTR = CAN_RTR_DATA;

        if (x[0] == 0x01) {
            TxData[0] = 0b00010000;
            TxData[1] = 0x04;
            TxData[2] = 0x00;
            TxData[3] = 0x00;
            TxData[4] = 0x00;
            TxData[5] = 0x00;
            TxData[6] = 0x00;
            TxData[7] = 0x00;
        } else if (x[0] == 0x00) {
            TxData[0] = 0b00010000;
            TxData[1] = 0x04;
            TxData[2] = 0x00;
            TxData[3] = 0x00;
            TxData[4] = 0x00;
            TxData[5] = 0x00;
            TxData[6] = 0x00;
            TxData[7] = 0x01;
        }

        if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
            Error_Handler();
        }

    }


}
void CAN1_CAN2_individ(uint32_t can_id, uint8_t* x, uint8_t length) {

    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[8];

    // Преобразование входящего сообщения от системы управления
     // id = 0x160 + адрес: индивидуальный запуск/остановка
     //if (can_id >= 0x160 && can_id <= 0x18F) {
    if ((can_id >= 0x160) & (can_id <= 0x18F)) {//не работает с &&
         TxHeader.DLC = 8;
         TxHeader.IDE = CAN_ID_EXT;
         TxHeader.ExtId = can_id - 0x160 + 1;
         TxHeader.RTR = CAN_RTR_DATA;

         if (x[0] == 0x01) {
             TxData[0] = 0b00010000;
             TxData[1] = 0x04;
             TxData[2] = 0x00;
             TxData[3] = 0x00;
             TxData[4] = 0x00;
             TxData[5] = 0x00;
             TxData[6] = 0x00;
             TxData[7] = 0x00;
         } else if (x[0] == 0x00) {
             TxData[0] = 0b00010000;
             TxData[1] = 0x04;
             TxData[2] = 0x00;
             TxData[3] = 0x00;
             TxData[4] = 0x00;
             TxData[5] = 0x00;
             TxData[6] = 0x00;
             TxData[7] = 0x01;
         }

         if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
             Error_Handler();
         }
    }
}

void CAN1_CAN2_current(uint32_t can_id, uint8_t* x, uint8_t length) {

    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[8];

    // Преобразование входящего сообщения от системы управления
     // id = 0x160 + адрес: индивидуальный запуск/остановка
     //if (can_id >= 0x160 && can_id <= 0x18F) {
    if ((can_id >= 0x160) & (can_id <= 0x18F)) {//не работает с &&
         TxHeader.DLC = 8;
         TxHeader.IDE = CAN_ID_EXT;
         TxHeader.ExtId = can_id - 0x160 + 1;
         TxHeader.RTR = CAN_RTR_DATA;

         // Преобразование входящего сообщения от системы управления
         // Выставка значения тока, мА
         uint64_t I = (uint64_t)((x[2] << 8) | x[1]);// только грубо
         uint64_t I_ma = I * 1000 * 0.0146628;//15A - 1023 - FF 03
         unsigned char byte1 = (I_ma >> 24) & 0xFF;
         unsigned char byte2 = (I_ma >> 16) & 0xFF;
         unsigned char byte3 = (I_ma >> 8) & 0xFF;
         unsigned char byte4 =  I_ma & 0xFF;

         TxData[0] = 0x10;
         TxData[1] = 0x03;
         TxData[2] = 0x00;
         TxData[3] = 0x00;

         TxData[4] = byte1;
         TxData[5] = byte2;
         TxData[6] = byte3;
         TxData[7] = byte4;

         if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
             Error_Handler();
         }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};  // Example data
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
         HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);
         CAN1_CAN2_broadcast(RxHeader.StdId, RxData, RxHeader.DLC);// StdId = 0x150
         CAN1_CAN2_individ(RxHeader.StdId, RxData, RxHeader.DLC);// (can_id >= 0x160 && can_id <= 0x18F)
         CAN1_CAN2_current(RxHeader.StdId, RxData, RxHeader.DLC);// (can_id >= 0x160 && can_id <= 0x18F)
    }
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};  // Example data
    if(HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK)
    {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_14);
    }
}
//https://www.waveshare.com/wiki/STM32CubeMX_Tutorial_Series:_USART
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_10)
  {
	  printf("\n\r Кнопка S1 нажата\n\r");
    //Код обработки прерывания для PE10
	  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_15);

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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  //const char *message = "Hello, UART!\n";

  // Отправка сообщения через UART3
  //HAL_UART_Transmit(&huart3, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);


  printf("\n\r !UART Printf Example: retarget the C library printf function to the UART\n\r");
  printf("\n\r Работает\n\r");
  /* USER CODE END 2 */

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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

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
