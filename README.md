

# EDR Test Program

Main purpose of the code is to demonstrate Electrochemical Dynamic Response method. Written in C using KEIL IDE.

## What does the code do?

	1- it initializes the core properly, assigns the pll frequency, defined in main.h.
	2- Initializes the debug and load switch pins. 
	3- Initializes the ADC & TIM8, ADC samples the battery voltage on every TIM8 pulse.
	4- ADC is connected to the DMA in order to have some flexibility.
	5- Initializes the UART,
	6- When the system receive 'S' char from uart, load switch is turned on and sampling starts.
	7- half of the measurement, load switch is turned off
	8- when the dma buffer is full, measurement is completed.
	9- data is analyzed and send to the user for further processing.

