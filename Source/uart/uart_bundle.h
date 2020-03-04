#ifndef _UART_BUNDLE_H_
#define _UART_BUNDLE_H_

	#include <stdbool.h>
	#include "stm32f3xx_hal.h"

//*** <<< Use Configuration Wizard in Context Menu >>> ***
// <!c1> Disable log
// <i> Disable log file generation
#define _USE_LOG
// </c>
//*** <<< end of configuration section >>>    ***

		typedef enum {
				UART_TX_Pool_RX_IT	=								0x34,
				UART_IT_INIT				=								0x23,
				UART_DMA_INIT				=								0x12
			} UART_INIT_EnumTypedef;
		
			typedef enum {
				TX_Return				=		false,
				RX_Return				=		true
			} RX_TX_Return_EnumTypedef;
		
		#define USARTx                           USART1
		#define USARTx_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE()
		#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()
		#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
		#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

		#define USARTx_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
		#define USARTx_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()


		#define USARTx_TX_PIN                    GPIO_PIN_9						// pin 30
		#define USARTx_TX_GPIO_PORT              GPIOA
		#define USARTx_TX_AF                     GPIO_AF7_USART1
		
		#define USARTx_RX_PIN                    GPIO_PIN_10					// pin 31
		#define USARTx_RX_GPIO_PORT              GPIOA
		#define USARTx_RX_AF                     GPIO_AF7_USART1


		#define USARTx_TX_DMA_CHANNEL            DMA1_Channel4
		#define USARTx_RX_DMA_CHANNEL            DMA1_Channel5 


		#define USARTx_DMA_TX_IRQn               DMA1_Channel4_IRQn
		#define USARTx_DMA_RX_IRQn               DMA1_Channel5_IRQn
		#define USARTx_DMA_TX_IRQHandler         DMA1_Channel4_IRQHandler
		#define USARTx_DMA_RX_IRQHandler         DMA1_Channel5_IRQHandler

		#define USARTx_IRQn                      USART1_IRQn
		#define USARTx_IRQHandler                USART1_IRQHandler


		/* it is not changed into dynamic mode because debug problems.*/
		/* Size of Reception buffer */
		#define RXBUFFERSIZE                     1
		
		bool UART_INIT(UART_HandleTypeDef* UartHandle,uint8_t** ptrTX,uint8_t** ptrRX,uint32_t uartBaudrate,UART_INIT_EnumTypedef UART_DMA_INIT);
		
		/*On demand*/
		void clearPointerIndexes(UART_HandleTypeDef *UartHandle);
		
		void sendStringDataSelf(UART_HandleTypeDef *UartHandle,char* ptr);
		void sendStringDataMan(UART_HandleTypeDef *UartHandle,char* ptr,uint16_t size);
		
		/*weak defs*/
		__weak void RXReturnCallback(UART_HandleTypeDef* UartHandle);
		__weak void TXReturnCallback(UART_HandleTypeDef* UartHandle);
		
#endif





