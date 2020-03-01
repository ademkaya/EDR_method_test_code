/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : stm32f3xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"

extern DMA_HandleTypeDef  		  Dma2Handle_ADC2;
static void TIMCLKInit(TIM_HandleTypeDef *htim);

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc){

GPIO_InitTypeDef          GPIO_InitStruct;

	if (hadc->Instance == ExtBatADCx){

		ExtBatADCx_PIN_CLK_ENABLE();
		/* ADC1 Periph clock enable */
		ExtBatADCx_CLK_ENABLE();
		/* Enable DMA1 clock */
		ExtBatDMAx_DMA_CLK_ENABLE();

		/*##- 2- Configure peripheral GPIO #########################################*/
		/* ADC Channel GPIO pin configuration */
		GPIO_InitStruct.Pin  = ExtBatADCx_CHANNEL_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(ExtBatADCx_CHANNEL_GPIO_PORT, &GPIO_InitStruct);
		/*##- 3- Configure DMA #####################################################*/

		/*********************** Configure DMA parameters ***************************/
		Dma2Handle_ADC2.Instance                 = ExtBatADCx_DMA_CHANNEL;
		Dma2Handle_ADC2.Init.Direction           = DMA_PERIPH_TO_MEMORY;
		Dma2Handle_ADC2.Init.PeriphInc           = DMA_PINC_DISABLE;
		Dma2Handle_ADC2.Init.MemInc              = DMA_MINC_ENABLE;
		Dma2Handle_ADC2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		Dma2Handle_ADC2.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
		Dma2Handle_ADC2.Init.Mode                = DMA_NORMAL;
		Dma2Handle_ADC2.Init.Priority            = DMA_PRIORITY_MEDIUM;

		/* Deinitialize  & Initialize the DMA for new transfer */
		HAL_DMA_DeInit(&Dma2Handle_ADC2);
		HAL_DMA_Init(&Dma2Handle_ADC2);

		/* Associate the DMA handle */
		__HAL_LINKDMA(hadc, DMA_Handle, Dma2Handle_ADC2);

	// /* NVIC configuration for DMA Input data interrupt */
		HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);

	}
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim) {

	TIMCLKInit(htim);
}

static void TIMCLKInit(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM1){
		__HAL_RCC_TIM1_CLK_ENABLE();
	}else if  (htim->Instance == TIM2){
		__HAL_RCC_TIM2_CLK_ENABLE();
	}else if (htim->Instance == TIM3){
		__HAL_RCC_TIM3_CLK_ENABLE();
	}else if (htim->Instance == TIM4){
		__HAL_RCC_TIM4_CLK_ENABLE();
	}else if (htim->Instance == TIM6){
		__HAL_RCC_TIM6_CLK_ENABLE();
	}else if (htim->Instance == TIM7){
		__HAL_RCC_TIM7_CLK_ENABLE();
	}else if (htim->Instance == TIM8){
		__HAL_RCC_TIM8_CLK_ENABLE();
	}else if (htim->Instance == TIM15){
		__HAL_RCC_TIM15_CLK_ENABLE();
	}else if (htim->Instance == TIM16){
		__HAL_RCC_TIM16_CLK_ENABLE();
	}else if (htim->Instance == TIM17){
		__HAL_RCC_TIM17_CLK_ENABLE();
	}
}


void HAL_MspInit(void)  {
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
	
//  /* SysTick_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 1);

}


