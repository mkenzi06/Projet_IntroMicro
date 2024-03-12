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
#include "ctype.h"

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE {
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART2 and Loop until the end of transmission */
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

void receiveString(char *buffer, int maxLength) {
    int index = 0;
    char receivedChar;

    do {
        HAL_UART_Receive(&huart2, (uint8_t *)&receivedChar, 1, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart2, (uint8_t *)&receivedChar, 1, 0xFFFF); // Écho du caractère reçu
        buffer[index] = receivedChar;
        index++;
    } while (receivedChar != '\n' && index < maxLength);

    buffer[index - 1] = '\0'; // Remplace le caractère de nouvelle ligne par le terminateur de chaîne
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

const char *char_sounds[] = {
    "._",    // A
    "_...",  // B
    "_._.",  // C
    "_..",   // D
    ".",     // E
    ".._.",  // F
    "__.",   // G
    "....",  // H
    "..",    // I
    ".___",  // J
    "_._",   // K
    "._..",  // L
    "__",    // M
    "_.",    // N
    "___",   // O
    ".__.",  // P
    "__._",  // Q
    "._.",   // R
    "...",   // S
    "_",     // T
    ".._",   // U
    "..._",  // V
    ".__",   // W
    "_.._",  // X
    "_.__",  // Y
    "__..",  // Z
    " ",     // Espace
    " ",     // Autres caractères
};

void generateMorseSound(char c) {
  // Recherche de l'indice du caractère dans le tableau char_sounds
	c = toupper(c);
  int index = c - 'A';
  if (index >= 0 && index < 26) {
    // Récupération du son correspondant au caractère
    const char *sound = char_sounds[index];

    // Parcours du son caractère par caractère
    for (int i = 0; sound[i] != '\0'; i++) {
      // Génération du son en fonction du caractère
      if (sound[i] == '.') {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   // Allume le buzzer
        HAL_Delay(100);   // Durée du son court
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); // Éteint le buzzer
        HAL_Delay(100);   // Pause entre les sons
      } else if (sound[i] == '_') {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   // Allume le buzzer
        HAL_Delay(400);   // Durée du son long
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); // Éteint le buzzer
        HAL_Delay(200);   // Pause entre les sons
      }
    }
  } else if (c == ' ') {
    // Pause entre les mots
    HAL_Delay(600);
  }
}

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
  MX_USART2_UART_Init();
  //test de la fonction scanf
while(1){
  char *receivedString; // Assuming max length of received string is 100 characters

      // Receive a string
      receiveString(receivedString, sizeof(receivedString));

      // Print the received string
//      printf("Received string: %s\n", receivedString);

      for (int i = 0; receivedString[i] != '\0'; i++) {
    	      generateMorseSound(receivedString[i]);
    	    }
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
}





//  char receivedMessage[] = "salut";


  /* USER CODE END 2 */

  /* USER CODE BEGIN 2 */

  /* Infinite loop */

  /* USER CODE BEGIN WHILE */

//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);

while (1){


//	if (HAL_UART_Receive(&huart2, (uint8_t *)receivedMessage, sizeof(receivedMessage), HAL_MAX_DELAY) == HAL_OK) {
//	      // Traitement du message reçu
//	      // ...
//
//	const char *message = "Hello WORLD";  // Message à convertir en code Morse

	    // Parcours du message caractère par caractère
//	    for (int i = 0; message[i] != '\0'; i++) {
//	      generateMorseSound(message[i]);
//	    }
	//	HAL_Delay(1000);
	//  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
	//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
//	      // Parcours du message caractère par caractère
//	      for (int i = 0; receivedMessage[i] != '\0'; i++) {
//	        generateMorseSound(receivedMessage[i]);
//	      }
	    }
//	  }
//    const char *message = "HELLO WORLD";  // Message à convertir en code Morse
//
//    // Parcours du message caractère par caractère
//    for (int i = 0; message[i] != '\0'; i++) {
//      generateMorseSound(message[i]);
//    }
//	HAL_Delay(1000);
//  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
