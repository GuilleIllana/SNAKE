/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nokia5110_LCD.h"
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
	 typedef enum{inicio, pausa, juego, muerto} Game;
	 	 
	 typedef struct{
	   uint8_t fila;
		 uint8_t columna;
	 }Posicion;
	 
	 typedef struct {
	  uint8_t size;
	  uint8_t dir; //0 arriba, 1 derecha, 2 abajo, 3 izquierda
	  Posicion Pos[150]; //No se puede poner más
	 }Snake;
	 
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_X 81
#define MAX_Y 45 //quitando el score
#define MIN_X 2
#define MIN_Y 10
#define MAX_COLUMNA 40 
#define MAX_FILA 18 
#define MAX_SNAKE 760 //Longitud máxima de la serpiente (ocupando todo el tablero)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
	bool ISR = false;
  bool ISADC = false;
	bool ISTIMER = false;
  uint8_t ADC_X = 0;
  uint8_t ADC_Y = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM4_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
void drawFood(uint8_t pixelX, uint8_t pixelY); //Dibujo de la comida
void drawTablero(uint8_t Tab[MAX_FILA][MAX_COLUMNA], uint8_t score);
void SnakePos(uint8_t Tab[MAX_FILA][MAX_COLUMNA], Snake* snake, Game *estado);
void Dibujo(Game *estado, uint8_t Tab[MAX_FILA][MAX_COLUMNA], Snake* snake);
void JuegoInit(Snake *Serpiente, uint8_t Tab[MAX_FILA][MAX_COLUMNA]);
void Estado(Game *estado, Snake *Serpiente, uint8_t Tab[MAX_FILA][MAX_COLUMNA]);
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
  Snake Serpiente;
	uint8_t Tablero[MAX_FILA][MAX_COLUMNA]; //0 si vacio, 1 si serpiente, 2 si comida

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
   Game estado = inicio;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  LCD_setRST(GPIOE, GPIO_PIN_7);
  LCD_setCE(GPIOE, GPIO_PIN_8);
  LCD_setDC(GPIOE, GPIO_PIN_9);
  LCD_setDIN(GPIOE, GPIO_PIN_10);
  LCD_setCLK(GPIOE, GPIO_PIN_11);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  LCD_init();
	HAL_ADC_Start_IT(&hadc1);
	HAL_ADC_Start_IT(&hadc2);
  HAL_TIM_Base_Start_IT(&htim4);
	

	 //inicialización del vector de posiciones
	 JuegoInit(&Serpiente, Tablero);
   
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//Pantalla 84*48 pixeles
		//Cabeza 4 pixeles, cada fruta aumenta en dos el tamaño de la serpiente (4 pixeles +)
		




    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
		//LLamada a Dibujo(Para enviar a la pantalla los datos ) cuando se activa la interrupción del contador
		if (ISTIMER){
		
	  Dibujo(&estado, Tablero, &Serpiente);	 Dibujo(&estado, Tablero, &Serpiente);	
	  ISTIMER = false;
 }
		//LLamada a funciones Estado (para gestión del estado) 
    Estado(&estado, &Serpiente, Tablero);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_8B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 160000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 200;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE7 PE8 PE9 
                           PE10 PE11 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 
                           PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void Dibujo(Game *estado, uint8_t Tab[MAX_FILA][MAX_COLUMNA], Snake* snake){
	
	  char  num[3];
	//Elección de lo que hay que dibujar según el estado en el que nos encontramos
	switch (*estado) {
		
		case juego:
		SnakePos(Tab, snake, estado);
    drawTablero(Tab, snake->size - 1);
	 	HAL_Delay(50);
			break;
		case pausa: 
		LCD_print("PAUSA", 28, 3);
		
		   break;
		case muerto: 
			sprintf(num, "%d" ,snake->size - 1);
			LCD_clrScr();
		  LCD_invert(true);
	    LCD_print("GAME", 30, 2);
	    LCD_print("OVER", 30, 3);
			LCD_print("SCORE:", 20, 5);
	    LCD_print(num, 55, 5);
      HAL_Delay(750);
		  LCD_clrScr();
		  LCD_print("JUEGAS", 25, 2);
	    LCD_print("OTRA?", 28, 3);
			LCD_print("SCORE:", 20, 5);
	    LCD_print(num, 55, 5);
	    HAL_Delay(750);

			break;
		case inicio: 
			LCD_clrScr();
		  LCD_invert(true);
	    LCD_print("PULSA", 28, 2);
	    LCD_print("PARA", 30, 3);
		  LCD_print("INICIAR", 20, 4);
		  break;
		default: break;
		
	}

}

void drawFood(uint8_t pixelX, uint8_t pixelY){
	uint8_t x1,x2,y1,y2, x, y;
	x = 2*pixelY + 2;
	y = 2*pixelX + 10;
	x1 = x - 1;
	x2 = x + 1;
	y1 = y - 1;
	y2 = y + 1;
	if (x1 <= MAX_X)
		LCD_setPixel(x1, y, true);
	if (x2 <= MAX_X)
		LCD_setPixel(x2, y, true);
	if (y1 <= MAX_Y)
		LCD_setPixel(x, y1, true);
	if (y2 <= MAX_Y)
		LCD_setPixel(x, y2, true);
	LCD_refreshArea(x1, y1, x2, y2);
}

void drawTablero(uint8_t Tab[MAX_FILA][MAX_COLUMNA], uint8_t score){
  char  num[3];
	
	//Paso de int a char* (impresión de la puntuación)
	sprintf(num, "%d" ,score);
	char* puntos = "SCORE: ";
	
 //Limpieza de la pantalla
	LCD_clrScr();
	
 //Dibujo del tablero
	LCD_drawHLine(0, 8, 84);
  LCD_drawHLine(0, 9, 84);		
  LCD_drawHLine(0, 47, 84);
  LCD_drawHLine(0, 46, 84);	
	LCD_drawVLine(0, 8, 37);
	LCD_drawVLine(1, 8, 37);
	LCD_drawVLine(82, 8, 37);
	LCD_drawVLine(83, 8, 37);
		
	//Dibujo del tablero
	for (int i = 0; i < MAX_COLUMNA; i++){
		for (int j = 0; j < MAX_FILA; j++){
			if (Tab[j][i] == 1 || Tab[j][i] == 3){
				LCD_drawHLine(2*i + 2, 2*j + 10, 2);
				LCD_drawHLine(2*i + 2, 2*j + 11, 2);
			}
			else if (Tab[j][i] == 2){
				drawFood(j, i);
			}
		}
	}
	LCD_refreshScr();
	
	//Dibujo de la puntuación
	LCD_print(puntos, 35,0);
	LCD_print(num, 70,0 );
}

void SnakePos(uint8_t Tab[MAX_FILA][MAX_COLUMNA], Snake* snake, Game* estado) {
	int aux_fila[snake->size], aux_columna[snake->size]; 
	//Cambio de dirección de la serpiente según los valores del ADC
	if (ADC_X > 200 && snake->dir != 3) snake->dir = 1;
	else if (ADC_X < 50 && snake->dir != 1) snake->dir = 3;
	else if (ADC_Y > 200 && snake->dir != 2) snake->dir = 0;
	else if (ADC_Y < 50 && snake->dir != 0) snake->dir = 2;
	else snake->dir = snake->dir;
	
	//Asignación de valores al vector auxiliar
	for (int i = 0; i < snake->size; i++){
		aux_fila[i] = snake->Pos[i].fila;
		aux_columna[i] = snake->Pos[i].columna;
	}
   
	//Desplazamiento del vector de posiciones 
	 for (int i = 0; i < snake->size; i++){
		snake->Pos[i+1].fila = aux_fila[i];
		snake->Pos[i+1].columna = aux_columna[i];
	}
	 //Comprobación de limites y movimiento de la serpiente
	switch (snake->dir) {
		case 0:
			snake->Pos[0].fila = snake->Pos[0].fila + 1;
			if (snake->Pos[0].fila == MAX_FILA) *estado = muerto; 
			break;
		
		case 1:
			snake->Pos[0].columna = snake->Pos[0].columna + 1;
			if (snake->Pos[0].columna == MAX_COLUMNA) *estado = muerto;			
			break;
		
		case 2: 
			snake->Pos[0].fila = snake->Pos[0].fila - 1;
			if (snake->Pos[0].fila == 255) *estado = muerto;
			break;
		
		case 3:
			snake->Pos[0].columna = snake->Pos[0].columna - 1;
			if (snake->Pos[0].columna == 255) *estado = muerto; 
			break;
		
		default: 
			break;
	}
	 //Comprobación de comida
	if(Tab[snake->Pos[0].fila][snake->Pos[0].columna] == 2) {
		uint8_t fila, columna;
		snake->size = snake->size++; 
	  do {
		fila = rand() % MAX_FILA;
		columna = rand() % MAX_COLUMNA;
		}while(Tab[fila][columna] == 1);
		
		if (rand() % 50 < 5)
		Tab[fila][columna] = 3;
		
		else
			Tab[fila][columna] = 2;
	}
	else if(Tab[snake->Pos[0].fila][snake->Pos[0].columna] == 3) {
		uint8_t fila, columna;
		snake->size = snake->size + 2;
		do {
		fila = rand() % MAX_FILA;
		columna = rand() % MAX_COLUMNA;
		}while(Tab[fila][columna] == 1);
		
		 Tab[fila][columna] = 2;
	}
	//Comprobación si se come a si misma
	for(int i = 1; i < snake->size; i++)
		if(snake->Pos[0].fila == snake->Pos[i].fila && snake->Pos[0].columna == snake->Pos[i].columna)
			*estado = muerto;
			
	
	//Las posiciones que ocupan la serpiente se ponen a 1
	for (int i = 0; i < snake->size; i++){
		 Tab[snake->Pos[i].fila][snake->Pos[i].columna] = 1;
	}
   
	//La antigua posición de la cola se pone a 0 (la serpiente se ha movido)
	Tab[snake->Pos[snake->size].fila][snake->Pos[snake->size].columna] = 0;
	
	snake->Pos[snake->size].fila = 0;
	snake->Pos[snake->size].columna = 0;

}
//Interrupciones del ADC
 void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

     ADC_X = HAL_ADC_GetValue(&hadc1);
	   ADC_Y = HAL_ADC_GetValue(&hadc2);
 }
 //Interrupción del botón
 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	 
	 ISR = true;
 }
 
 //Inicio del juego
 void JuegoInit(Snake *Serpiente, uint8_t Tab[MAX_FILA][MAX_COLUMNA]){
	   LCD_clrScr();
	   LCD_invert(false);
	   LCD_print("CARGANDO...", 10, 2);
	 	 for (int i = 0; i < MAX_COLUMNA; i++){
		 for (int j = 0; j < MAX_FILA; j++){
			 Tab[j][i] = 0;
		 }
	 }
	 Serpiente->Pos[0].fila = 10;
	 Serpiente->Pos[0].columna = 30;
	 Serpiente->dir = 3;
	 Serpiente->size = 1;
	 Tab[8][20] = 2;
	 HAL_Delay(100);
 }
 
 //Cambio de estado con ISR
 void Estado(Game *estado, Snake *Serpiente, uint8_t Tab[MAX_FILA][MAX_COLUMNA]){
	   if (*estado == juego && ISR == true){
		 *estado = pausa;
		 ISR = false;
	 }	
   else if (*estado == pausa && ISR == true){
		 *estado = juego;
		 ISR = false;
	 }		 	 
    else if ((*estado == muerto || *estado == inicio) && ISR == true){
		 JuegoInit(Serpiente, Tab);
		 *estado = juego;
		 ISR = false;
	 }	
		
 }
 
 void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  ISTIMER = true;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
