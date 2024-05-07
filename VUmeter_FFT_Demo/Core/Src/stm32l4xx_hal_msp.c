/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file         stm32l4xx_hal_msp.c
 * @brief        This file provides code for the MSP Initialization
 *               and de-Initialization codes.
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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern DMA_HandleTypeDef hdma_spi1_tx;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
 * Initializes the Global MSP.
 */
void HAL_MspInit(void) {
	/* USER CODE BEGIN MspInit 0 */

	/* USER CODE END MspInit 0 */

	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_PWR_CLK_ENABLE();

	/* System interrupt init*/

	/* USER CODE BEGIN MspInit 1 */

	/* USER CODE END MspInit 1 */
}

/**
 * @brief SPI MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hspi: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (hspi->Instance == SPI1) {
		/* USER CODE BEGIN SPI1_MspInit 0 */

		/* USER CODE END SPI1_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_SPI1_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**SPI1 GPIO Configuration
		 PA1     ------> SPI1_SCK
		 PA6     ------> SPI1_MISO
		 PA7     ------> SPI1_MOSI
		 */
		GPIO_InitStruct.Pin = DISPL_SCK_Pin | TOUCH_MISO_Pin | DISPL_MOSI_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* SPI1 DMA Init */
		/* SPI1_TX Init */
		hdma_spi1_tx.Instance = DMA1_Channel3;
		hdma_spi1_tx.Init.Request = DMA_REQUEST_1;
		hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_spi1_tx.Init.Mode = DMA_NORMAL;
		hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
		if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK) {
			Error_Handler();
		}

		__HAL_LINKDMA(hspi, hdmatx, hdma_spi1_tx);

		/* SPI1 interrupt Init */
		HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(SPI1_IRQn);
		/* USER CODE BEGIN SPI1_MspInit 1 */

		/* USER CODE END SPI1_MspInit 1 */
	}

}

/**
 * @brief SPI MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hspi: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI1) {
		/* USER CODE BEGIN SPI1_MspDeInit 0 */

		/* USER CODE END SPI1_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_SPI1_CLK_DISABLE();

		/**SPI1 GPIO Configuration
		 PA1     ------> SPI1_SCK
		 PA6     ------> SPI1_MISO
		 PA7     ------> SPI1_MOSI
		 */
		HAL_GPIO_DeInit(GPIOA, DISPL_SCK_Pin | TOUCH_MISO_Pin | DISPL_MOSI_Pin);

		/* SPI1 DMA DeInit */
		HAL_DMA_DeInit(hspi->hdmatx);

		/* SPI1 interrupt DeInit */
		HAL_NVIC_DisableIRQ(SPI1_IRQn);
		/* USER CODE BEGIN SPI1_MspDeInit 1 */

		/* USER CODE END SPI1_MspDeInit 1 */
	}

}

/**
 * @brief UART MSP Initialization
 * This function configures the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };
	if (huart->Instance == USART2) {
		/* USER CODE BEGIN USART2_MspInit 0 */

		/* USER CODE END USART2_MspInit 0 */

		/** Initializes the peripherals clock
		 */
		PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
		PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
			Error_Handler();
		}

		/* Peripheral clock enable */
		__HAL_RCC_USART2_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**USART2 GPIO Configuration
		 PA2     ------> USART2_TX
		 PA15 (JTDI)     ------> USART2_RX
		 */
		GPIO_InitStruct.Pin = VCP_TX_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
		HAL_GPIO_Init(VCP_TX_GPIO_Port, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = VCP_RX_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF3_USART2;
		HAL_GPIO_Init(VCP_RX_GPIO_Port, &GPIO_InitStruct);

		/* USER CODE BEGIN USART2_MspInit 1 */

		/* USER CODE END USART2_MspInit 1 */
	}

}

/**
 * @brief UART MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		/* USER CODE BEGIN USART2_MspDeInit 0 */

		/* USER CODE END USART2_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_USART2_CLK_DISABLE();

		/**USART2 GPIO Configuration
		 PA2     ------> USART2_TX
		 PA15 (JTDI)     ------> USART2_RX
		 */
		HAL_GPIO_DeInit(GPIOA, VCP_TX_Pin | VCP_RX_Pin);

		/* USER CODE BEGIN USART2_MspDeInit 1 */

		/* USER CODE END USART2_MspDeInit 1 */
	}

}

extern DMA_HandleTypeDef hdma_sai1_a;

extern DMA_HandleTypeDef hdma_sai1_b;

static uint32_t SAI1_client = 0;

void HAL_SAI_MspInit(SAI_HandleTypeDef *hsai) {

	GPIO_InitTypeDef GPIO_InitStruct;
	/* SAI1 */
	if (hsai->Instance == SAI1_Block_A) {
		/* Peripheral clock enable */
		if (SAI1_client == 0) {
			__HAL_RCC_SAI1_CLK_ENABLE();

			/* Peripheral interrupt init*/
			HAL_NVIC_SetPriority(SAI1_IRQn, 0, 0);
			HAL_NVIC_EnableIRQ(SAI1_IRQn);
		}
		SAI1_client++;

		/**SAI1_A_Block_A GPIO Configuration
		 PA3     ------> SAI1_MCLK_A
		 PA8     ------> SAI1_SCK_A
		 PA9     ------> SAI1_FS_A
		 PA10     ------> SAI1_SD_A
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* Peripheral DMA init*/

		hdma_sai1_a.Instance = DMA2_Channel1;
		hdma_sai1_a.Init.Request = DMA_REQUEST_1;
		hdma_sai1_a.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_sai1_a.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_sai1_a.Init.MemInc = DMA_MINC_ENABLE;
		hdma_sai1_a.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		hdma_sai1_a.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		hdma_sai1_a.Init.Mode = DMA_CIRCULAR;
		hdma_sai1_a.Init.Priority = DMA_PRIORITY_LOW;
		if (HAL_DMA_Init(&hdma_sai1_a) != HAL_OK) {
			Error_Handler();
		}

		/* Several peripheral DMA handle pointers point to the same DMA handle.
		 Be aware that there is only one channel to perform all the requested DMAs. */
		__HAL_LINKDMA(hsai, hdmarx, hdma_sai1_a);

		__HAL_LINKDMA(hsai, hdmatx, hdma_sai1_a);

	}
	if (hsai->Instance == SAI1_Block_B) {
		/* Peripheral clock enable */
		if (SAI1_client == 0) {
			__HAL_RCC_SAI1_CLK_ENABLE();

			/* Peripheral interrupt init*/
			HAL_NVIC_SetPriority(SAI1_IRQn, 0, 0);
			HAL_NVIC_EnableIRQ(SAI1_IRQn);
		}
		SAI1_client++;

		/**SAI1_B_Block_B GPIO Configuration
		 PB5     ------> SAI1_SD_B
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_5;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* Peripheral DMA init*/

		hdma_sai1_b.Instance = DMA2_Channel2;
		hdma_sai1_b.Init.Request = DMA_REQUEST_1;
		hdma_sai1_b.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_sai1_b.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_sai1_b.Init.MemInc = DMA_MINC_ENABLE;
		hdma_sai1_b.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		hdma_sai1_b.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		hdma_sai1_b.Init.Mode = DMA_CIRCULAR;
		hdma_sai1_b.Init.Priority = DMA_PRIORITY_LOW;
		if (HAL_DMA_Init(&hdma_sai1_b) != HAL_OK) {
			Error_Handler();
		}

		/* Several peripheral DMA handle pointers point to the same DMA handle.
		 Be aware that there is only one channel to perform all the requested DMAs. */
		__HAL_LINKDMA(hsai, hdmarx, hdma_sai1_b);
		__HAL_LINKDMA(hsai, hdmatx, hdma_sai1_b);
	}
}

void HAL_SAI_MspDeInit(SAI_HandleTypeDef *hsai) {
	/* SAI1 */
	if (hsai->Instance == SAI1_Block_A) {
		SAI1_client--;
		if (SAI1_client == 0) {
			/* Peripheral clock disable */
			__HAL_RCC_SAI1_CLK_DISABLE();
			/* SAI1 interrupt DeInit */
			HAL_NVIC_DisableIRQ(SAI1_IRQn);
		}

		/**SAI1_A_Block_A GPIO Configuration
		 PA3     ------> SAI1_MCLK_A
		 PA8     ------> SAI1_SCK_A
		 PA9     ------> SAI1_FS_A
		 PA10     ------> SAI1_SD_A
		 */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10);

		/* SAI1 DMA Deinit */
		HAL_DMA_DeInit(hsai->hdmarx);
		HAL_DMA_DeInit(hsai->hdmatx);
	}
	if (hsai->Instance == SAI1_Block_B) {
		SAI1_client--;
		if (SAI1_client == 0) {
			/* Peripheral clock disable */
			__HAL_RCC_SAI1_CLK_DISABLE();
			/* SAI1 interrupt DeInit */
			HAL_NVIC_DisableIRQ(SAI1_IRQn);
		}

		/**SAI1_B_Block_B GPIO Configuration
		 PB5     ------> SAI1_SD_B
		 */
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5);

		/* SAI1 DMA Deinit */
		HAL_DMA_DeInit(hsai->hdmarx);
		HAL_DMA_DeInit(hsai->hdmatx);
	}
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
