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
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
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
IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define USART_TXBUF_LEN 1512
#define USART_RXBUF_LEN 128
uint8_t USART_TxBuf[USART_TXBUF_LEN];
uint8_t USART_RxBuf[USART_RXBUF_LEN];


__IO int USART_TX_Empty=0;
__IO int USART_TX_Busy=0;
__IO int USART_RX_Empty=0;
__IO int USART_RX_Busy=0;

int blink_enabled=0;
int myFrequency;
int timeToWait;
int timeCounter;
int delayFlag=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM10_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

uint8_t USART_kbhit(){
	if(USART_RX_Empty==USART_RX_Busy){
		return 0; //jeśli puste
	}else{
		return 1; //jeśli coś jest odebrane
	}
}

//pozyskanie znaku
int16_t USART_getchar()
{
int16_t tmp;
	if(USART_RX_Empty!=USART_RX_Busy)
	{
		 tmp=USART_RxBuf[USART_RX_Busy]; //tmp- tymczasowa zmienna, która odczytuje pierwszy znak zajęty z bufora odbioru
		 USART_RX_Busy++;
		 if(USART_RX_Busy >= USART_RXBUF_LEN)USART_RX_Busy=0; //modyfikuje Busy jeśli wyszedł po za tablicę
		 return tmp; //zwraca znak
	}else return -1;
}

//pozyskanie linii
uint8_t USART_getline(char *buf) //*char to wskaźnik na skopiowanie wartości zmiennej
{
	static uint8_t bf[128];
	static uint8_t idx=0;
	int i;
	uint8_t ret;
	while(USART_kbhit())
	{
		bf[idx]=USART_getchar();
		if(((bf[idx]==10)||(bf[idx]==13)||(bf[idx]==59)))
		{
			bf[idx]=0;
			for(i=0;i<=idx;i++){
				buf[i]=bf[i];
			}
			ret=idx;
			idx=0;
			return ret;
		}
		else
		{
			idx++;
			if(idx>=128)idx=0;
		}
	}
	return 0;
}

//wyslanie komunikatu
void USART_fsend(char* format,...){
	char tmp_rs[128];
	int i;
	__IO int idx;
	va_list arglist;
	va_start(arglist,format);
	vsprintf(tmp_rs,format,arglist);
	va_end(arglist);
	idx=USART_TX_Empty; //identyfikacja idx jako pusty bufor nadawania
	for(i=0;i<strlen(tmp_rs);i++){
		USART_TxBuf[idx]=tmp_rs[i];
		idx++;
		if(idx >= USART_TXBUF_LEN)idx=0;
	}
	__disable_irq(); //blokowanie przerwania
	if((USART_TX_Empty==USART_TX_Busy)&&(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TXE)==SET)){
		//sprawdzic dodatkowo zajetosc bufora nadajnika
		USART_TX_Empty=idx;
		uint8_t tmp=USART_TxBuf[USART_TX_Busy]; //pierwszy znak z danymi do zmiennej tymczasowej
		USART_TX_Busy++;
		if(USART_TX_Busy >= USART_TXBUF_LEN)USART_TX_Busy=0;
		HAL_UART_Transmit_IT(&huart2, &tmp, 1); //wpisanie znaku do wysłania
	}
	else{
		USART_TX_Empty=idx;
	}
	__enable_irq(); //odblokowanie przerwania
}

//wysylanie znaku - komunkacja z PC
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){ //uchwyt do wywołania przerwania
	if(huart==&huart2){ //czy nasz intrefejs
		if(USART_TX_Empty!=USART_TX_Busy){ //czy coś jest do wysłania
			uint8_t tmp=USART_TxBuf[USART_TX_Busy]; //pobieranie znaku
			USART_TX_Busy++; //korygazja wskaźnika
			if(USART_TX_Busy >= USART_TXBUF_LEN)USART_TX_Busy=0;
			HAL_UART_Transmit_IT(&huart2, &tmp, 1); //wysyłanie znaku
		}
	}
}

//odbieranie znaku - komunkacja z PC
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart==&huart2){
		USART_RX_Empty++; //empty na pierwszy wolny
		if(USART_RX_Empty>=USART_RXBUF_LEN)USART_RX_Empty=0;
		HAL_UART_Receive_IT(&huart2,&USART_RxBuf[USART_RX_Empty],1);
	}
}

void mruganie(){
	if(blink_enabled!=0){
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
}

//zmiana czestotliwosci mrugania diody - dzialania TIM11
void zmiana(int newFrequency){
	uint16_t newPeriod = 9999/newFrequency;
	__HAL_TIM_SET_AUTORELOAD(&htim11, newPeriod);
	HAL_TIM_Base_Init(&htim11);
}

//resetowanie wartosci "doliczonej" przez licznik na 0
void resetTimer(TIM_HandleTypeDef *htim) {
    HAL_TIM_Base_Stop(htim);
    __HAL_TIM_SET_COUNTER(htim, 0);
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start(htim);
}

void czekaj(int delayValue){
	timeToWait=delayValue;
	timeCounter=0;
	delayFlag=0;
	resetTimer(TIM10);
	while(delayFlag!=1){}
}

void mrugajMorse(char* slowo) {
    for (int i = 0; i < strlen(slowo); i++) {
        char litera = slowo[i];
        switch (litera) {
            case 'E':
            case 'e':
//            	kropka
            	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
                czekaj(300);
                HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
                czekaj(300);
                break;
            case 'S':
            case 's':
//            	trzy kropki
                for (int j = 0; j < 3; j++) {
                	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
                	czekaj(300);
                	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
                	czekaj(300);
                }
                break;
            case 'T':
            case 't':
//              kreska
                HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
                czekaj(1000);
                HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
                czekaj(300);
                break;
            default:
                break;
        }
        HAL_Delay(700);
    }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if(htim->Instance == TIM11){
		mruganie();
	}

	if(htim->Instance == TIM10){
		if(delayFlag==0){
			timeCounter+=1;
			if(timeCounter==timeToWait){
				delayFlag=1;
			}
		}
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
  MX_USART2_UART_Init();
  MX_TIM11_Init();
  MX_TIM10_Init();

  //inicjalizacja WatchDoga
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Receive_IT(&huart2,&USART_RxBuf[0],1); //odbior pierwszego znaku
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_TIM_Base_Start_IT(&htim11);
  USART_fsend("Witaj, jestem Nucleo STM32!!!");
  int len=0;
  char bx[200]; //bufor do odbierania stringów
  while (1)
  {
    /* USER CODE BEGIN 3 */
	  HAL_IWDG_Refresh(&hiwdg);

	  if((len = USART_getline(bx)) > 0) {

	      USART_fsend("REC[%d]> %s\r\n", len, bx);

      	  char command[128];
      	  strncpy(command, bx, 128); // Kopiuj linię do zmiennej command

       	  if (strncmp(command, "LED[ON]", 7) == 0) {

       		  blink_enabled=0;
       		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
       		  USART_fsend("LED[ON] - Dioda zapalona\r\n");
       	  }

       	  else if (strncmp(command, "LED[OFF]", 8) == 0) {
       		  blink_enabled=0;
       		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
       		  USART_fsend("LED[OFF] - Dioda zgaszona\r\n");
       	  }

       	  else if (strncmp(command, "LED[BLINK,", 10) == 0) {
       		  myFrequency = atoi(command + 10);
       		  if(myFrequency!=0)
       		  {
       			  zmiana(myFrequency);
       			  blink_enabled=1;
       			  USART_fsend("LED[BLINK,%d] - Mruganie dioda z czestotliwoscia %d Hz\r\n", myFrequency, myFrequency);
       		  }
       		  else
       		  {
       			  USART_fsend("LED[BLINK,%d] - Podaj inny parametr niz 0! %d Hz\r\n", myFrequency, myFrequency);
       		  }
       	  }

       	  else if (strncmp(command, "INSERT[Delay,", 13) == 0) {
       		  int delay_ms = atoi(command + 13);
       		  czekaj(delay_ms);
       		  USART_fsend("INSERT[Delay,%d] - Opoznienie o %d ms\r\n", delay_ms, delay_ms);
       	  }

          else if (strncmp(command, "MORSE[", 6) == 0) {
              int i = 6;
              char slowo[128];
              int j = 0;
              while (command[i] != ']' && command[i] != '\0') {
                  slowo[j++] = command[i++];
              }
              slowo[j] = '\0';

              mrugajMorse(slowo);
              USART_fsend("MORSE[%s] - Sygnalizacja Morse'a dla sekwencji liter %s\r\n", slowo, slowo);
          }

       	  else {
       		  USART_fsend("Blad: Nie rozpoznano polecenia.\r\n");
       	  }
	  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Reload = 2499;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 8399;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 9;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 8399;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 9999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
