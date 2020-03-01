#ifndef MAIN_H
#define MAIN_H



#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>




		#define		CoreFrequency									24000				//Khz		

		#define		SET_PIN_ON(port,pin)							((port)->BSRR = (pin))
		#define		SET_PIN_OFF(port,pin)							((port)->BRR  = (pin))

		#define 	setBit(regPtr,bit)								 ((*regPtr)|=(1<<bit))
		#define 	resetBit(regPtr,bit)							 ((*regPtr)&=~(1<<bit))

		#define LEDPort						  GPIOC
		#define LEDPin							GPIO_PIN_13

		#define LOADPort						GPIOA
		#define LOADPin							GPIO_PIN_8

		#define StateSet_PORT				GPIOA
		#define StateSet_PIN				GPIO_PIN_3

		/*---------------- EXTERNAL BATTERY READ DEFINES -------------------------*/
		#define ADC_BATTERY_BUFFER_SIZE			  		(200)
		#define ExtBatADCx                          ADC2
		#define ExtBatADCx_CLK_ENABLE()             __HAL_RCC_ADC12_CLK_ENABLE()
		#define ExtBatADCx_FORCE_RESET()            __HAL_RCC_ADC12_FORCE_RESET()
		#define ExtBatADCx_RELEASE_RESET()          __HAL_RCC_ADC12_RELEASE_RESET()
		#define ExtBatADCx_PIN_CLK_ENABLE()   		  __HAL_RCC_GPIOA_CLK_ENABLE()
		#define ExtBatADCx_CHANNEL_PIN              GPIO_PIN_6
		#define ExtBatADCx_CHANNEL_GPIO_PORT        GPIOA
		#define ExtBatADCx_CHANNEL                  ADC_CHANNEL_3
		#define ExtBatDMAx_DMA_CLK_ENABLE()      	  __HAL_RCC_DMA2_CLK_ENABLE()
		#define ExtBatADCx_DMA_CHANNEL				  		DMA2_Channel1
			
		/*		DEFINES for alkaline battery	*/
		#define batteryDepletedLevel					 			1300
		#define testLength						 							248			//mSec	
				
#endif
