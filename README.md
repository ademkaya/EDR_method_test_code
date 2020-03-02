

# EDR Test Program 

Main purpose of the code is to demonstrate Electrochemical Dynamic Response method. Written in C using KEIL IDE.

## What does the code do? :bowtie:

	1- it initializes the core properly, assigns the pll frequency, defined in main.h.
	2- Initializes the debug and load switch pins. 
	3- Initializes the ADC & TIM8, ADC samples the battery voltage on every TIM8 pulse.
	4- ADC is connected to the DMA in order to have some flexibility.
	5- Initializes the UART,
	6- When the system receive 'S' char from uart, load switch is turned on and sampling starts.
	7- half of the measurement, load switch is turned off
	8- when the dma buffer is full, measurement is completed.
	9- data is analyzed and send to the user for further processing.

## Note:

 - You can see the flowchart under the main folder structure.

 - The project is uncompleted without STM32F3 family framework, please download it from [STM32F3 HAL DRIVERS](https://www.st.com/en/embedded-software/stm32cubef3.html) and place it under a folder named "F3_Drivers".

 - Folder layout
								[EDR_test_code] -------  [F3_Drivers]
															|
															|
												   [CMSIS]-----[STM32F3xx_HAL_Driver]
												   