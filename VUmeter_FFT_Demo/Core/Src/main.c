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
#include "dma.h"
#include "windows.h"
#include "displayFFT.h"
#include "colors.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VU_GAIN 3.17 //in order to calibrate the VU meter (0dB = 0.775 Vrms)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;
DMA_HandleTypeDef hdma_sai1_a;
DMA_HandleTypeDef hdma_sai1_b;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int16_t saiTx_buf[SAI_BUF_LEN];
int16_t saiRx_buf[SAI_BUF_LEN];
q63_t PowerLeft, PowerRight;
float PwrAvgLeft, PwrAvgRight;
volatile int powerComputed = 0;
arm_rfft_fast_instance_f32 fft_instance;
volatile int fft_data_request = 1;
float32_t inputLeftFFT[BUFFER_SIZE], inputRightFFT[BUFFER_SIZE];
float32_t outputFFT[BUFFER_SIZE];
uint32_t lastTouchTick;
enum displayFFTstyle_t style = LINEAR;
fftDisplay_t fftDisplayLeft, fftDisplayRight;
float32_t octValuesLeft[NOCTAVES], octValuesRight[NOCTAVES];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SAI1_Init(void);
/* USER CODE BEGIN PFP */
void processData(int16_t *saiRxBuffer, int16_t *saiTxBuffer);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

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
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_SPI1_Init();
	MX_SAI1_Init();
	/* USER CODE BEGIN 2 */
	Displ_Init(Displ_Orientat_90);
	Displ_CLS(BACKGROUND_COLOR);
	Displ_BackLight('F');
	HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t*) saiTx_buf, SAI_BUF_LEN);
	HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint8_t*) saiRx_buf, SAI_BUF_LEN);
	arm_rfft_fast_init_f32(&fft_instance, BUFFER_SIZE);

	initFFTdisplay(&fftDisplayLeft, 12, 148, style, octValuesLeft );
	initFFTdisplay(&fftDisplayRight, 172, 148, style, octValuesRight );

	initVUmeter(12, 30);
	initVUmeter(172, 30);

	Displ_CString(12, 112, 148, Font16.Height + 114, "Left", Font16, 1, TEXT_COLOR, BACKGROUND_COLOR);
	Displ_CString(172, 112, 308, Font16.Height + 114, "Right", Font16, 1, TEXT_COLOR, BACKGROUND_COLOR);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (!fft_data_request) {
			arm_mult_f32(inputLeftFFT, KAISER_WINDOW, inputLeftFFT, BUFFER_SIZE);
			arm_rfft_fast_f32(&fft_instance, inputLeftFFT, outputFFT, 0);
			displayFFTdata(&fftDisplayLeft, outputFFT);
			arm_mult_f32(inputRightFFT, KAISER_WINDOW, inputRightFFT, BUFFER_SIZE);
			arm_rfft_fast_f32(&fft_instance, inputRightFFT, outputFFT, 0);
			displayFFTdata(&fftDisplayRight, outputFFT);
			fft_data_request = 1;
		}

		if (powerComputed) {
			q15_t PwrLeft = PowerLeft >> 15;
			q15_t PwrRight = PowerRight >> 15;
			q15_t VeffLeft, VeffRight;
			arm_sqrt_q15(PwrLeft, &VeffLeft);
			arm_sqrt_q15(PwrRight, &VeffRight);
			PwrAvgLeft = 0.93 * PwrAvgLeft + 0.07 * VU_GAIN / 32768 * (float32_t) (VeffLeft);
			PwrAvgRight = 0.93 * PwrAvgRight + 0.07 * VU_GAIN / 32768 * (float32_t) (VeffRight);
			displayVUmeter(12, 30, (int) 100.0 * PwrAvgLeft);
			displayVUmeter(172, 30, (int) 100.0 * PwrAvgRight);
			powerComputed = 0;

		}
		if (HAL_GetTick() - lastTouchTick > 20) {
			if (Touch_GotATouch(1)) {
				if (Touch_In_XY_area(12, 148, 135, 60) || Touch_In_XY_area(172, 148, 135, 60)) {
					if (style == LINEAR) {
						style = OCTAVE;
						fftDisplayLeft.style = OCTAVE;
						redrawFFTdisplay(&fftDisplayLeft);
						fftDisplayRight.style = OCTAVE;
						redrawFFTdisplay(&fftDisplayRight);
					} else {
						style = LINEAR;
						fftDisplayLeft.style = LINEAR;
						redrawFFTdisplay(&fftDisplayLeft);
						fftDisplayRight.style = LINEAR;
						redrawFFTdisplay(&fftDisplayRight);
					}
				}
				lastTouchTick = HAL_GetTick();
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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_9;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 5;
	RCC_OscInitStruct.PLL.PLLN = 33;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}

	/** Enable MSI Auto calibration
	 */
	HAL_RCCEx_EnableMSIPLLMode();
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void) {
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the peripherals clock
	 */
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
	PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
	PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
	PeriphClkInit.PLLSAI1.PLLSAI1M = 5;
	PeriphClkInit.PLLSAI1.PLLSAI1N = 64;
	PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV25;
	PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SAI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SAI1_Init(void) {

	/* USER CODE BEGIN SAI1_Init 0 */

	/* USER CODE END SAI1_Init 0 */

	/* USER CODE BEGIN SAI1_Init 1 */

	/* USER CODE END SAI1_Init 1 */
	hsai_BlockA1.Instance = SAI1_Block_A;
	hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
	hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
	hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
	hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
	hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
	hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
	hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
	hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
	hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
	hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
	if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK) {
		Error_Handler();
	}
	hsai_BlockB1.Instance = SAI1_Block_B;
	hsai_BlockB1.Init.AudioMode = SAI_MODESLAVE_RX;
	hsai_BlockB1.Init.Synchro = SAI_SYNCHRONOUS;
	hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
	hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
	hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
	hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
	hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
	hsai_BlockB1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
	if (HAL_SAI_InitProtocol(&hsai_BlockB1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SAI1_Init 2 */

	/* USER CODE END SAI1_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

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
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	/* DMA2_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
	/* DMA2_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, DISPL_DC_Pin | DISPL_CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(TOUCH_CS_GPIO_Port, TOUCH_CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(DISPL_RST_GPIO_Port, DISPL_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(DISPL_LED_GPIO_Port, DISPL_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : DISPL_DC_Pin DISPL_CS_Pin */
	GPIO_InitStruct.Pin = DISPL_DC_Pin | DISPL_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : TOUCH_CS_Pin */
	GPIO_InitStruct.Pin = TOUCH_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(TOUCH_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : TOUCH_INT_Pin */
	GPIO_InitStruct.Pin = TOUCH_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(TOUCH_INT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : DISPL_RST_Pin */
	GPIO_InitStruct.Pin = DISPL_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DISPL_RST_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : DISPL_LED_Pin */
	GPIO_InitStruct.Pin = DISPL_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DISPL_LED_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai) {
	processData(&saiRx_buf[SAI_BUF_LEN / 2], &saiTx_buf[SAI_BUF_LEN / 2]);
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
	processData(&saiRx_buf[0], &saiTx_buf[0]);
}

void processData(int16_t *saiRxBuffer, int16_t *saiTxBuffer) {
	uint16_t index1, index2;

	static int16_t inputSignalLeft[BUFFER_SIZE], inputSignalRight[BUFFER_SIZE];
	static int16_t outputSignalLeft[BUFFER_SIZE], outputSignalRight[BUFFER_SIZE];

	// copy RxBuffer to leftSignalBuffer and rightSignalBuffer
	for (index1 = 0, index2 = 0; index1 < BUFFER_SIZE; index1++) {
		inputSignalRight[index1] = saiRxBuffer[index2++];
		inputSignalLeft[index1] = saiRxBuffer[index2++];
	}

	// Here, the data of inputSignalLeft[] and inputSignalRight[] can be processed
	// The result should be stored in outputSignalLeft[] and outpuSignalRight[].
	// Note that the arrays contain BUFFER_SIZE values each

	// copy input to output
	arm_copy_q15(inputSignalLeft, outputSignalLeft, BUFFER_SIZE);
	arm_copy_q15(inputSignalRight, outputSignalRight, BUFFER_SIZE);

	// determine power
	arm_dot_prod_q15(inputSignalLeft, inputSignalLeft, BUFFER_SIZE, &PowerLeft);
	PowerLeft = PowerLeft >> LOG2_BUFFERSIZE;
	arm_dot_prod_q15(inputSignalRight, inputSignalRight, BUFFER_SIZE, &PowerRight);
	PowerRight = PowerRight >> LOG2_BUFFERSIZE;
	powerComputed = 1;

	// load new data for fft if requested
	if (fft_data_request) {
		arm_q15_to_float(inputSignalLeft, inputLeftFFT, BUFFER_SIZE);
		arm_q15_to_float(inputSignalRight, inputRightFFT, BUFFER_SIZE);
		fft_data_request = 0;
	}

	// copy left and right txBuffer into TxBuffer
	for (index1 = 0, index2 = 0; index1 < BUFFER_SIZE; index1++) {
		saiTxBuffer[index2++] = outputSignalRight[index1];
		saiTxBuffer[index2++] = outputSignalLeft[index1];
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
