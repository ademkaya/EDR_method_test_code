

# EDR Test Program

Main purpose of the code is to demonstrate Electrochemical Dynamic Response method. 

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

## Flow Chart
```mermaid
graph TD
	A(START) -->B[Initialize the hardware]
	B -->  C{measurement start ?}
	C-- NO -->  C{measurement start?}
	C-- YES--> D[Start sampling & turn on load switch]
	D--> E[Is half of the measurement reached?]
	E-- NO --> E{Is half of the measurement reached?}
	E-- YES--> F[Turn off the load switch]
	F--> G{Is the measurement finished?}
	G--NO--> G{Is the measurement finished?}
	G--YES-->H[Analyze the data send the result to the user]
	H-->I{RESTART?}
	I-- NO -->I{RESTART?}
	I-- YES -->D[Start sampling & turn on load switch]