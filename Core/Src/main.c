/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "damier.h"
#include "serveur_udp.h"
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

DAC_HandleTypeDef hdac;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

LTDC_HandleTypeDef hltdc;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart7;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

SDRAM_HandleTypeDef hsdram1;

osThreadId task_initHandle;
osThreadId affichageHandle;
osThreadId task_selectHandle;
osThreadId task_calculPossHandle;
osThreadId task_victoryHandle;
osThreadId task_menuHandle;
osThreadId tache_defaultHandle;
osMessageQId queueSelHandle;
/* USER CODE BEGIN PV */
SemaphoreHandle_t mutexEcran;

struct cell {
	uint16_t ligne;
	uint16_t colonne;
};


struct chess_cell {
	uint16_t ligne;
	uint16_t colonne;
	uint8_t isPossible;
	uint8_t isFilled;
	uint8_t isDame;
	uint8_t piece_color;
	uint8_t rayon;
};

struct chess_cell chessboard[8][8];
struct cell possible_eaten[32][12];
uint8_t change = 1, isTurn = 0, victory = 0;
uint8_t nb_blue = 12, nb_white = 12;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC3_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_LTDC_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_UART7_Init(void);
static void MX_RNG_Init(void);
void fonction_init(void const * argument);
void fonction_affichage(void const * argument);
void fonction_select(void const * argument);
void fonction_calculPossibilites(void const * argument);
void fonctionVictory(void const * argument);
void fonction_menu(void const * argument);
void StartTask07(void const * argument);

/* USER CODE BEGIN PFP */
uint8_t calculPossibilitesRec(uint16_t line, uint16_t col, uint8_t color, struct cell *possibilites, uint8_t index, uint8_t nb_eaten);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * Calcule récursivement les cases possibles a partir de la ligne, de la colonne et de la couleur.
 * line			: ligne de depart
 * col			: colonne de depart
 * color		: couleur de la piece a deplacer
 * possibilites	: tableau qui va contenir les possibilites (vide initialement)
 * index		: indice de la prochaine case vide du tableau (a initialiser a 0)
 * nb_eaten		: donne le nombre de pieces mangees jusqu'à cette possible case (init a 0)
 *
 * retour		: indice de la prochaine case vide du tableau
 */
uint8_t calculPossibilitesRec(uint16_t line, uint16_t col, uint8_t color, struct cell *possibilites, uint8_t index, uint8_t nb_eaten)
{
	int8_t pas   = (color == 0) ? 1 : -1; // en fonction couleur on regarde lignes croissantes ou decroissantes
	int8_t fin   = (color == 0) ? 7 : 0; // en fonction couleur pas meme arrivee
	int8_t debut = (color == 0) ? 0 : 7; // en fonction couleur pas meme arrivee

	// Controle de la colonne de droite en avant :
	if(col < 7 && line != fin)
	{
		// Controle colonne de droite en avant : piece presente
		if(chessboard[line + pas][col + 1].isFilled == 1)
		{
			if(chessboard[line + pas][col + 1].piece_color == color) ;//Une piece de sa couleur bloque
			else if((col <= 5) && (line + pas != fin)) // Assez de cases pour sauter
			{
				if(chessboard[line + 2 * pas][col + 2].isFilled == 0) // Pas de piece apres le saut
				{
					//Piece de l'autre couleur, place pour manger
					struct cell possible = {line + 2 * pas, col + 2};
					possibilites[index] = possible;
					possible_eaten[index][nb_eaten].ligne = line + pas;
					possible_eaten[index][nb_eaten].colonne = col + 1;
					index++;
					nb_eaten++;
					index = calculPossibilitesRec(line + 2 * pas, col + 2, color, possibilites, index, nb_eaten);
				}
			}
		}
		// Controle colonne de droite : pas de piece et pas en train de manger
		else if (nb_eaten == 0)
		{
			struct cell possible = {line + pas, col + 1};
			possibilites[index] = possible;
			index++;
		}
	}
	// Controle colonne de droite en arriere : piece presente
	if(col < 7 && line != debut)
	{
		if(chessboard[line - pas][col + 1].isFilled == 1)
		{
			if(chessboard[line - pas][col + 1].piece_color == color) ;//Une piece de sa couleur bloque
			else if((col <= 5) && (line - pas != debut)) // Assez de cases pour sauter
			{
				if(chessboard[line - 2 * pas][col + 2].isFilled == 0) // Pas de piece apres le saut
				{
					//=> Piece de l'autre couleur, place pour manger
					struct cell possible = {line - 2 * pas, col + 2};
					possibilites[index] = possible;
					possible_eaten[index][nb_eaten].ligne = line - pas;
					possible_eaten[index][nb_eaten].colonne = col + 1;
					index++;
					nb_eaten++;
					index = calculPossibilitesRec(line - 2 * pas, col + 2, color, possibilites, index, nb_eaten);
				}
			}
		}
		// Controle colonne de droite : pas de piece et pas en train de manger
		else if (nb_eaten == 0)
		{
			struct cell possible = {line + pas, col - 1};
			possibilites[index] = possible;
			index++;
		}
	}
	// Controle de la colonne de gauche en avant :
	if(col > 0 && line != fin)
	{
		// Controle colonne de gauche : piece presente
		if(chessboard[line + pas][col - 1].isFilled == 1)
		{
			if(chessboard[line + pas][col - 1].piece_color == color) ;//Une piece de sa couleur bloque
			else if((col >= 2) && (line + pas != fin)) // Assez de cases pour sauter
			{
				if(chessboard[line + 2 * pas][col - 2].isFilled == 0) // Pas de piece apres le saut
				{
					//Piece de l'autre couleur, place pour manger
					struct cell possible = {line + 2 * pas, col - 2};
					possibilites[index] = possible;
					possible_eaten[index][nb_eaten].ligne = line + pas;
					possible_eaten[index][nb_eaten].colonne = col - 1;
					index++;
					nb_eaten++;
					index = calculPossibilitesRec(line + 2 * pas, col - 2, color, possibilites, index, nb_eaten);
				}
			}
		}
		// Controle colonne de gauche : pas de piece et pas en train de manger
		else if (nb_eaten == 0)
		{
			struct cell possible = {line + pas, col - 1};
			possibilites[index] = possible;
			index++;
		}
	}
	// Controle colonne de gauche en arriere : piece presente
	if(col > 0 && line != debut)
	{
		if(chessboard[line - pas][col - 1].isFilled == 1)
		{
			if(chessboard[line - pas][col - 1].piece_color == color) ;//Une piece de sa couleur bloque
			else if((col >= 2) && (line - pas != debut)) // Assez de cases pour sauter
			{
				if(chessboard[line - 2 * pas][col - 2].isFilled == 0) // Pas de piece apres le saut
				{
					//Piece de l'autre couleur, place pour manger
					struct cell possible = {line - 2 * pas, col - 2};
					possibilites[index] = possible;
					possible_eaten[index][nb_eaten].ligne = line - pas;
					possible_eaten[index][nb_eaten].colonne = col - 1;
					index++;
					nb_eaten++;
					index = calculPossibilitesRec(line - 2 * pas, col - 2, color, possibilites, index, nb_eaten);
				}
			}
		}
	}
	return index;

}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//uint32_t potl,potr,joystick_h, joystick_v;
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC3_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_UART7_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */
  BSP_LCD_Init();
    BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
    BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS+ BSP_LCD_GetXSize()*BSP_LCD_GetYSize()*4);
    BSP_LCD_DisplayOn();
    BSP_LCD_SelectLayer(0);
    BSP_LCD_Clear(LCD_COLOR_BLACK);
    //BSP_LCD_DrawBitmap(0,0,(uint8_t*)HorombeRGB565_bmp);
    BSP_LCD_DrawBitmap(0,0,(uint8_t*)damier_bmp);
    BSP_LCD_SelectLayer(1);
    BSP_LCD_Clear(0);
    BSP_LCD_SetFont(&Font12);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_SetBackColor(LCD_COLOR_BLUE);

    BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
    //BSP_TS_ITConfig();

	// Init potentiometre
	  sConfig.Channel = ADC_CHANNEL_6;
	  HAL_ADC_ConfigChannel(&hadc3, &sConfig);
	  HAL_ADC_Start(&hadc3);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
    mutexEcran = xSemaphoreCreateMutex();

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of queueSel */
  osMessageQDef(queueSel, 16, uint16_t);
  queueSelHandle = osMessageCreate(osMessageQ(queueSel), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of task_init */
  osThreadDef(task_init, fonction_init, osPriorityHigh, 0, 1024);
  task_initHandle = osThreadCreate(osThread(task_init), NULL);

  /* definition and creation of affichage */
  osThreadDef(affichage, fonction_affichage, osPriorityNormal, 0, 1024);
  affichageHandle = osThreadCreate(osThread(affichage), NULL);

  /* definition and creation of task_select */
  osThreadDef(task_select, fonction_select, osPriorityAboveNormal, 0, 256);
  task_selectHandle = osThreadCreate(osThread(task_select), NULL);

  /* definition and creation of task_calculPoss */
  osThreadDef(task_calculPoss, fonction_calculPossibilites, osPriorityBelowNormal, 0, 4096);
  task_calculPossHandle = osThreadCreate(osThread(task_calculPoss), NULL);

  /* definition and creation of task_victory */
  osThreadDef(task_victory, fonctionVictory, osPriorityNormal, 0, 1024);
  task_victoryHandle = osThreadCreate(osThread(task_victory), NULL);

  /* definition and creation of task_menu */
  osThreadDef(task_menu, fonction_menu, osPriorityHigh, 0, 512);
  task_menuHandle = osThreadCreate(osThread(task_menu), NULL);

  /* definition and creation of tache_default */
  osThreadDef(tache_default, StartTask07, osPriorityIdle, 0, 128);
  tache_defaultHandle = osThreadCreate(osThread(tache_default), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  //Suppression des taches qui ne doivent pas fonctionner a l'initialisation
  osThreadTerminate(task_victoryHandle);
  //osThreadTerminate(task_initHandle);

  /* USER CODE END RTOS_THREADS */

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
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
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

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
  hi2c1.Init.Timing = 0x00C0EAFF;
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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00C0EAFF;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 40;
  hltdc.Init.VerticalSync = 9;
  hltdc.Init.AccumulatedHBP = 53;
  hltdc.Init.AccumulatedVBP = 11;
  hltdc.Init.AccumulatedActiveW = 533;
  hltdc.Init.AccumulatedActiveH = 283;
  hltdc.Init.TotalWidth = 565;
  hltdc.Init.TotalHeigh = 285;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 480;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 272;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg.FBStartAdress = 0xC0000000;
  pLayerCfg.ImageWidth = 480;
  pLayerCfg.ImageHeight = 272;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;
  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm B
  */
  sAlarm.Alarm = RTC_ALARM_B;
  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the TimeStamp
  */
  if (HAL_RTCEx_SetTimeStamp(&hrtc, RTC_TIMESTAMPEDGE_RISING, RTC_TIMESTAMPPIN_POS1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 3;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED14_Pin|LED15_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED16_GPIO_Port, LED16_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_DISP_GPIO_Port, LCD_DISP_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, LED13_Pin|LED17_Pin|LED11_Pin|LED12_Pin
                          |LED2_Pin|LED18_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : OTG_HS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_HS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_HS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_D2_Pin */
  GPIO_InitStruct.Pin = QSPI_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(QSPI_D2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_D7_Pin ULPI_D6_Pin ULPI_D5_Pin ULPI_D2_Pin
                           ULPI_D1_Pin ULPI_D4_Pin */
  GPIO_InitStruct.Pin = ULPI_D7_Pin|ULPI_D6_Pin|ULPI_D5_Pin|ULPI_D2_Pin
                          |ULPI_D1_Pin|ULPI_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPDIF_RX0_Pin */
  GPIO_InitStruct.Pin = SPDIF_RX0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_SPDIFRX;
  HAL_GPIO_Init(SPDIF_RX0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SDMMC_CK_Pin SDMMC_D3_Pin SDMMC_D2_Pin PC9
                           PC8 */
  GPIO_InitStruct.Pin = SDMMC_CK_Pin|SDMMC_D3_Pin|SDMMC_D2_Pin|GPIO_PIN_9
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BP2_Pin PA6 */
  GPIO_InitStruct.Pin = BP2_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED14_Pin LED15_Pin */
  GPIO_InitStruct.Pin = LED14_Pin|LED15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_NCS_Pin */
  GPIO_InitStruct.Pin = QSPI_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(QSPI_NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = OTG_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Audio_INT_Pin */
  GPIO_InitStruct.Pin = Audio_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Audio_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_P_Pin OTG_FS_N_Pin */
  GPIO_InitStruct.Pin = OTG_FS_P_Pin|OTG_FS_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SAI2_MCLKA_Pin SAI2_SCKA_Pin SAI2_FSA_Pin SAI2_SDA_Pin */
  GPIO_InitStruct.Pin = SAI2_MCLKA_Pin|SAI2_SCKA_Pin|SAI2_FSA_Pin|SAI2_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : SAI2_SDB_Pin */
  GPIO_InitStruct.Pin = SAI2_SDB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
  HAL_GPIO_Init(SAI2_SDB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin LED16_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|LED16_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : uSD_Detect_Pin */
  GPIO_InitStruct.Pin = uSD_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(uSD_Detect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_CTRL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PG9 RMII_RXER_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_9|RMII_RXER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SDMMC_CMD_Pin */
  GPIO_InitStruct.Pin = SDMMC_CMD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(SDMMC_CMD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TP3_Pin PH13 NC2_Pin */
  GPIO_InitStruct.Pin = TP3_Pin|GPIO_PIN_13|NC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_DISP_Pin */
  GPIO_InitStruct.Pin = LCD_DISP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_DISP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED13_Pin LED17_Pin LED11_Pin LED12_Pin
                           LED2_Pin LED18_Pin */
  GPIO_InitStruct.Pin = LED13_Pin|LED17_Pin|LED11_Pin|LED12_Pin
                          |LED2_Pin|LED18_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_INT_Pin */
  GPIO_InitStruct.Pin = LCD_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ULPI_NXT_Pin */
  GPIO_InitStruct.Pin = ULPI_NXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(ULPI_NXT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_D4_Pin ARDUINO_D2_Pin EXT_RST_Pin */
  GPIO_InitStruct.Pin = ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_STP_Pin ULPI_DIR_Pin */
  GPIO_InitStruct.Pin = ULPI_STP_Pin|ULPI_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : QSPI_D1_Pin QSPI_D3_Pin QSPI_D0_Pin */
  GPIO_InitStruct.Pin = QSPI_D1_Pin|QSPI_D3_Pin|QSPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_CLK_Pin ULPI_D0_Pin */
  GPIO_InitStruct.Pin = ULPI_CLK_Pin|ULPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
	// Executer a chaque reception du nombre de caractère indiqué dans HAL_UART_Receive_IT
	/*
		uint8_t Message[2];
		if(rxbuffer[0]=='a') HAL_GPIO_WritePin(LED11_GPIO_Port, LED11_Pin,1); // Allume la led si 'a'
		if(rxbuffer[0]=='e') HAL_GPIO_WritePin(LED11_GPIO_Port, LED11_Pin,0); // Eteint la led si 'e'
		// Si caractere utile pour la tâche de deplacement
		if((rxbuffer[0]=='d') || (rxbuffer[0]=='q') || (rxbuffer[0]=='z') || (rxbuffer[0]=='x'))
		{
			Message[0]=rxbuffer[0];
			// Envoie du message dans la file des interruptions
			xQueueSendFromISR(myQueueU2HHandle, &Message, 0);
		}

		HAL_UART_Receive_IT(&huart1,rxbuffer,1); // Rappel (callback) de l'interruption
		*/
	}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{	/**
	uint16_t MessageTS[1], MessageTempo = {1};
	MessageTS[0] = GPIO_Pin;
	//BP2 32758, BP1 256, TS 65248
	if(flag == 0 && GPIO_Pin == LCD_INT_Pin)
	{
		HAL_GPIO_TogglePin(LED11_GPIO_Port, LED11_Pin);
 		xQueueSendFromISR(myQueueTSHandle, &MessageTS, 0);
 		flag = 1;
	}
	//xQueueSendFromISR(myQueueTempoHandle, &MessageTempo, 0);
	 *
	 */
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_fonction_init */
/**
  * @brief  Function implementing the task_init thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_fonction_init */
void fonction_init(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */

  udpserver_init() ;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 20;
    uint8_t i, j, cpt_lignesw = 0, cpt_colonnesw = 1, cpt_lignesb, cpt_colonnesb;
    uint8_t init = 1;
    if(victory == 1)
    {
    	osThreadTerminate(task_victoryHandle);
    	osThreadDef(affichage, fonction_affichage, osPriorityNormal, 0, 1024);
    	affichageHandle = osThreadCreate(osThread(affichage), NULL);
    	victory = 0;
    }

  /* Infinite loop */
  for(;;)
  {
	  if (init == 1)
	  {
		  osThreadTerminate(task_menuHandle);
		  init = 0;
	  }
	  cpt_lignesw = 0;
	  cpt_colonnesw = 1;

	  for (i = 0; i < 3; i++)
	  {
		  for (j = 0; j < 4; j++)
		  {
			  taskENTER_CRITICAL();
			  	 // init white pieces
			  chessboard[cpt_lignesw][cpt_colonnesw].ligne = cpt_lignesw;
			  chessboard[cpt_lignesw][cpt_colonnesw].colonne = cpt_colonnesw;
			  chessboard[cpt_lignesw][cpt_colonnesw].isFilled = 1;
			  chessboard[cpt_lignesw][cpt_colonnesw].rayon = 9;
			  chessboard[cpt_lignesw][cpt_colonnesw].piece_color = 0;
			  // init blue pieces
			  cpt_lignesb = cpt_lignesw + 5;
			  cpt_colonnesb = (cpt_colonnesw % 2 == 0) ? cpt_colonnesw + 1 : cpt_colonnesw - 1;
			  chessboard[cpt_lignesb][cpt_colonnesb].ligne = cpt_lignesb;
			  chessboard[cpt_lignesb][cpt_colonnesb].colonne = cpt_colonnesb;
			  chessboard[cpt_lignesb][cpt_colonnesb].isFilled = 1;
			  chessboard[cpt_lignesb][cpt_colonnesb].rayon = 9;
			  chessboard[cpt_lignesb][cpt_colonnesb].piece_color = 1;
			  taskEXIT_CRITICAL();
			  cpt_colonnesw += 2;
		  }
		  cpt_colonnesw = (cpt_colonnesw % 2 == 0) ? 1 : 0;
		  cpt_lignesw++;
	  }

      vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_fonction_affichage */
/**
* @brief Function implementing the affichage thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_fonction_affichage */
void fonction_affichage(void const * argument)
{
  /* USER CODE BEGIN fonction_affichage */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 50;
	const uint8_t pasX 			= 30;
	const uint8_t pasY 			= 30;
	const uint8_t margeX		= 14;
	const uint8_t margeY		= 14;
	uint16_t pointeurX 			= margeX + pasX / 2;
	uint16_t pointeurY 			= margeY + pasY / 2;
	uint8_t color				= 2;
	uint8_t i, j;
	uint8_t filled = 0, possible = 0, dame = 0;
	osThreadTerminate(task_initHandle);

  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(LED12_GPIO_Port, LED12_Pin);
	  // Clear que pour certains changements
	  taskENTER_CRITICAL();
	  if(change == 1)
	  {
		  BSP_LCD_Clear(0);
	  }
	  taskEXIT_CRITICAL();
	  BSP_LCD_FillCircle(margeX, margeY, 3);
	  BSP_LCD_FillCircle(margeX + pasX, margeY, 3);
	  BSP_LCD_FillCircle(margeX, margeY  + pasY, 3);
	  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	  if(isTurn == 0)
	  {
		  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		  BSP_LCD_DisplayStringAt(3 * margeX + 8 * pasX, margeY, (uint8_t *) "Au tour du joueur blanc", LEFT_MODE);
	  }
	  else
	  {
		  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
		  BSP_LCD_DisplayStringAt(3 * margeX + 8 * pasX, margeY + 8 * pasY, (uint8_t *) "Au tour du joueur bleu", LEFT_MODE);
	  }
	  for (i = 0; i < 8; i++)
	  {
		  for (j = 0; j < 8; j++)
		  {
			  taskENTER_CRITICAL();
			  filled = chessboard[i][j].isFilled;
			  possible = chessboard[i][j].isPossible;
			  dame = chessboard[i][j].isDame;
			  taskEXIT_CRITICAL();
			  // Case avec un pion
			  if ( filled != 0)
			  {
				  color = chessboard[i][j].piece_color;
				  xSemaphoreTake(mutexEcran, portMAX_DELAY);
				  if (color == 1) BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
				  else if (color == 0) BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			      pointeurX = margeX + pasX / 2 + j * pasX;
			      pointeurY = margeY + pasY / 2 + i * pasY;
			      if (dame == 0) BSP_LCD_FillCircle(pointeurX, pointeurY, chessboard[i][j].rayon);
			      else BSP_LCD_FillRect(pointeurX - chessboard[i][j].rayon, pointeurY - chessboard[i][j].rayon, chessboard[i][j].rayon * 2, chessboard[i][j].rayon * 2);
				  xSemaphoreGive(mutexEcran);
			  }
			  //Case possible
			  else if (possible != 0)
			  {
				  if (change == 1) // Il y a eu une deselection, reinitialisation des possibles et pas d'affichage
				  {
					  taskENTER_CRITICAL();
					  chessboard[i][j].isPossible = 0;
					  taskEXIT_CRITICAL();
				  }
				  else
				  {
					  xSemaphoreTake(mutexEcran, portMAX_DELAY);
					  BSP_LCD_SetTextColor(LCD_COLOR_RED);
				      pointeurX = margeX + pasX / 2 + j * pasX;
				      pointeurY = margeY + pasY / 2 + i * pasY;
					  BSP_LCD_FillCircle(pointeurX, pointeurY, 9);
					  xSemaphoreGive(mutexEcran);
				  }

			  }
		  }
	  }
	taskENTER_CRITICAL();
	change = 0; // S'il y avait des changements, ils on ete pris en compte
	taskEXIT_CRITICAL();

    vTaskDelayUntil(&xLastWakeTime, (TickType_t) xFrequency);
  }
  /* USER CODE END fonction_affichage */
}

/* USER CODE BEGIN Header_fonction_select */
/**
* @brief Function implementing the task_select thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_fonction_select */
void fonction_select(void const * argument)
{
  /* USER CODE BEGIN fonction_select */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 100;
	static TS_StateTypeDef TS_State;
	//flag = 0;
	uint8_t posx = 0, posy = 0;
	uint8_t line = 0, col = 0;
	const uint8_t pas 			= 30;
	const uint8_t marge			= 15;
	uint8_t selected 			= 0;
	uint8_t line_selected		= 8;
	uint8_t col_selected		= 8;
	struct cell eaten_piece;
	uint8_t p;
	uint16_t message[1];
  /* Infinite loop */
  for(;;)
  {

	  BSP_TS_GetState(&TS_State);
	  if(TS_State.touchDetected)
	  {
		  posx = TS_State.touchX[0];
		  posy = TS_State.touchY[0];

		  col = (posx - marge) / pas;
		  line = (posy - marge) / pas;
		  taskENTER_CRITICAL();
		  // Selection d'un pion
		  if(chessboard[line][col].isFilled && (chessboard[line][col].piece_color == isTurn))
		  {
			  // Aucun pion n'etait selectionne
			  if(chessboard[line][col].rayon < 12 && selected == 0)
			  {
				  chessboard[line][col].rayon = 12;
				  selected = 1;
				  line_selected = line;
				  col_selected = col;
				  message[0] = (line << 8) + col;
				  xQueueSend(queueSelHandle, &message, 0);
			  }
			  /// Ce pion etait selectionne
			  else if (chessboard[line][col].rayon == 12)
			  {
				  chessboard[line][col].rayon = 9;
				  change = 1;
				  selected = 0;
			  }
		  }

		  // Case pour un deplacement
		  // TODO globalement tout se qui doit etre transmis se passe ici car seul moment où on deplace un pion et rend la main
		  if(chessboard[line][col].isPossible > 0)
		  {
			  // TODO envoi de la case de depart par un message "D + line_selected + col_selected"
			  chessboard[line_selected][col_selected].isFilled = 0;
			  // Suppression des pions manges
			  for (p = 0; p < 12; p++)
			  {
				  eaten_piece = possible_eaten[chessboard[line][col].isPossible - 1][p];
				  if(eaten_piece.ligne != 8)
				  {
					  // TODO envoi de la case mangee par un message "M + eaten_piece.ligne + eaten_piece.colonne"
					  chessboard[eaten_piece.ligne][eaten_piece.colonne].isFilled = 0;
					  // TODO decrementation a faire a la reception de chaque pion mange
			          if(isTurn == 0)
			              nb_blue--;
			          else
			              nb_white--;
				  }
			  }

			  // TODO envoi de la case d'arrivee avec le message "A + line + col"
			  // TODO faire a la reception les lignes suivantes
			  chessboard[line][col].isFilled = 1;
			  chessboard[line][col].isPossible = 0;
			  chessboard[line][col].piece_color = chessboard[line_selected][col_selected].piece_color;
			  chessboard[line][col].rayon = 9;
			  // La ligne de la dame est atteinte !
			  if((isTurn == 0 && line == 7) || (isTurn == 1 && line == 0)) chessboard[line][col].isDame = 1;
			  // Si le pion etait une dame, il le reste
			  if(chessboard[line_selected][col_selected].isDame == 1)
			  {
				  chessboard[line][col].isDame = 1;
				  chessboard[line_selected][col_selected].isDame = 0;
			  }
			  selected = 0;
			  change = 1;

	          // Check la fin de jeu
			  // TODO a faire a la reception aussi OU envoi d'un message "F + couleur gagnant"
	          if(nb_blue == 0 || nb_white == 0)
	          {

	        	  osThreadDef(task_victory, fonctionVictory, osPriorityNormal, 0, 1024);
	        	  task_victoryHandle = osThreadCreate(osThread(task_victory), NULL);
				  nb_blue = 12;
				  nb_white = 12;

	          }

			  // Changement de tour
	          // TODO a faire a la fin de la reception (donc critere a determiner) OU envoi d'un message "C"
			  isTurn = (isTurn == 0) ? 1 : 0;
		  }

		  taskEXIT_CRITICAL();

	  }

    vTaskDelayUntil(&xLastWakeTime, (TickType_t) xFrequency);
  }
  /* USER CODE END fonction_select */
}

/* USER CODE BEGIN Header_fonction_calculPossibilites */
/**
* @brief Function implementing the task_calculPoss thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_fonction_calculPossibilites */
void fonction_calculPossibilites(void const * argument)
{
  /* USER CODE BEGIN fonction_calculPossibilites */
	uint16_t message[1];
	uint8_t line, col;
	uint16_t color;
	struct cell possibilites[32];
	uint8_t length, i, m, n;

  /* Infinite loop */
  for(;;)
  {
	  // Recuperation information selection
	  xQueueReceive(queueSelHandle, &message, portMAX_DELAY);
	  line = (uint8_t) (message[0] >> 8);
	  col  = (uint8_t)  message[0];
	  taskENTER_CRITICAL();
	  color = chessboard[line][col].piece_color;
	  taskEXIT_CRITICAL();

	  // Calcul des possibilites
	  	  // Reinitialisation des cases possibles
	  for(m = 0; m < 32; m++)
	  {
		  for(n = 0; n < 12; n++)
		  {
			  possible_eaten[m][n].colonne = 8;
			  possible_eaten[m][n].ligne   = 8;
		  }
	  }
	  length = calculPossibilitesRec(line, col, color, possibilites, 0, 0);

	  // Modification de l'echiquier avec cases possibles
	  taskENTER_CRITICAL();
	  for(i = 0; i < length; i++)
	  {
		  chessboard[possibilites[i].ligne][possibilites[i].colonne].isPossible = i + 1;
	  }
	  taskEXIT_CRITICAL();
      osDelay(1);
  }
  /* USER CODE END fonction_calculPossibilites */
}

/* USER CODE BEGIN Header_fonctionVictory */
/**
* @brief Function implementing the task_victory thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_fonctionVictory */
void fonctionVictory(void const * argument)
{
  /* USER CODE BEGIN fonctionVictory */

	// TODO a modifier si l'on veut distinguer les deux ecrans de finish en fonction des joueurs

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 100;
	uint8_t init = 1;
	static TS_StateTypeDef TS_State;
  /* Infinite loop */
  for(;;)
  {
	  if (init == 1)
	  {
		  osThreadTerminate(affichageHandle);
		  init = 0;
	  }
	  BSP_LCD_SelectLayer(0);
	  if (isTurn == 1)
	  {
		  BSP_LCD_Clear(LCD_COLOR_WHITE);
		  BSP_LCD_SelectLayer(1);
		  BSP_LCD_Clear(LCD_COLOR_WHITE);
		  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
		  BSP_LCD_DisplayStringAtLine(10, (uint8_t *)"VICTOIRE des BLANCS !");
		  BSP_LCD_DisplayStringAtLine(12, (uint8_t *)"Felicitations au gagnant !");
		  BSP_LCD_SetTextColor(LCD_COLOR_RED);
		  BSP_LCD_DisplayStringAtLine(14, (uint8_t *)"Touchez l'ecran pour rejouer");
	  }
	  else if (isTurn == 0)
	  {
		  BSP_LCD_Clear(LCD_COLOR_BLUE);
		  BSP_LCD_SelectLayer(1);
		  BSP_LCD_Clear(LCD_COLOR_BLUE);
		  BSP_LCD_SelectLayer(1);
		  BSP_LCD_Clear(LCD_COLOR_BLUE);
		  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		  BSP_LCD_DisplayStringAtLine(10, (uint8_t *)"VICTOIRE des BLEUS !");
		  BSP_LCD_DisplayStringAtLine(12, (uint8_t *)"Felicitations au gagnant !");
		  BSP_LCD_SetTextColor(LCD_COLOR_RED);
		  BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
		  BSP_LCD_DisplayStringAtLine(14, (uint8_t *)"Touchez l'ecran pour rejouer");
		  nb_blue = 1;
		  nb_white = 1;
	  }
	  BSP_TS_GetState(&TS_State);
	  if(TS_State.touchDetected)
	  {
		  victory = 1;
		  osThreadDef(task_init, fonction_init, osPriorityHigh, 0, 1024);
		  task_initHandle = osThreadCreate(osThread(task_init), NULL);
	  }
	  vTaskDelayUntil(&xLastWakeTime, (TickType_t) xFrequency);
  }

  /* USER CODE END fonctionVictory */
}

/* USER CODE BEGIN Header_fonction_menu */
/**
* @brief Function implementing the task_menu thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_fonction_menu */
void fonction_menu(void const * argument)
{
  /* USER CODE BEGIN fonction_menu */
	uint8_t init = 1;
  /* Infinite loop */
  for(;;)
  {
	  /*
	  BSP_LCD_Clear(LCD_COLOR_WHITE);
	  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	  BSP_LCD_FillRect(0, 0, BSP_LCD_GetXSize()/2, BSP_LCD_GetYSize());

	  //Creation de la tache init pour qu'elle demarre
	  if (init == 1)
	  {
		  osThreadDef(task_init, fonction_init, osPriorityHigh, 0, 1024);
		  task_initHandle = osThreadCreate(osThread(task_init), NULL);
		  init = 0;
	  }
*/
    osDelay(1000);
  }
  /* USER CODE END fonction_menu */
}

/* USER CODE BEGIN Header_StartTask07 */
/**
* @brief Function implementing the tache_lwip_init thread.
* @param argument: Not used
* @retval None
* Je n'ai absolument pas confiance en free rtos du coup nouvelle tache default spéciale
*/
/* USER CODE END Header_StartTask07 */
void StartTask07(void const * argument)
{
  /* USER CODE BEGIN StartTask07 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask07 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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

