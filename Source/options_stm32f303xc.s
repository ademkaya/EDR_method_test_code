;/*****************************************************************************/
;/* STM32F303xOPT.S: STM32F303 Flash Option Bytes                              */
;/*****************************************************************************/
;/* <<< Use Configuration Wizard in Context Menu >>>                          */
;/*****************************************************************************/
;/* This file is part of the uVision/ARM development tools.                   */
;/* Copyright (c) 2005-2008 Keil Software. All rights reserved.               */
;/* This software may only be used under the terms of a valid, current,       */
;/* end user licence from KEIL for a compatible version of KEIL software      */
;/* development tools. Nothing else gives you the right to use this software. */
;/*****************************************************************************/
;// <e> Flash Option Bytes
FLASH_OPT       EQU     0

;//   <o0> Flash Read Protection
;//			<i>  WARNING! : LEVEL 2 IS COMPLETELY DISABLES SWD/JTAG INTERFACE. THERE IS NO GOING BACK TO LOWER LEVELS FROM THIS PROTECTION LEVEL.
;//			<0=> RDP LEVEL 0  <1=> RDP LEVEL 1 <2=> RDP LEVEL 2 
RDP             EQU     0x01

;// <h> Flash Write Protection
;//   <o0.0> Page 0..1
;//   <o0.1> Page 2..3
;//   <o0.2> Page 4..5
;//   <o0.3> Page 6..7
;//   <o0.4> Page 8..9
;//   <o0.5> Page 10..11
;//   <o0.6> Page 12..13
;//   <o0.7> Page 14..15
;//   <o1.0> Page 16..17
;//   <o1.1> Page 18..19
;//   <o1.2> Page 20..21
;//   <o1.3> Page 22..23
;//   <o1.4> Page 24..25
;//   <o1.5> Page 26..27
;//   <o1.6> Page 28..29
;//   <o1.7> Page 30..31
;//   <o2.0> Page 32..33
;//   <o2.1> Page 34..35
;//   <o2.2> Page 36..37
;//   <o2.3> Page 38..39
;//   <o2.4> Page 40..41
;//   <o2.5> Page 42..43
;//   <o2.6> Page 44..45
;//   <o2.7> Page 46..47
;//   <o3.0> Page 48..49
;//   <o3.1> Page 50..51
;//   <o3.2> Page 52..53
;//   <o3.3> Page 54..55
;//   <o3.4> Page 56..57
;//   <o3.5> Page 58..59
;//   <o3.6> Page 60..61
;//   <o3.7> Page 62..127
;// </h>
nWRP0           EQU     0xFF
nWRP1           EQU     0xFF
nWRP2           EQU     0xFF
nWRP3           EQU     0x7F
WRP0            EQU     nWRP0:EOR:0xFF
WRP1            EQU     nWRP1:EOR:0xFF
WRP2            EQU     nWRP2:EOR:0xFF
WRP3            EQU     nWRP3:EOR:0xFF

 
;// <h> User Configuration
;//   <o0.0> WDG_SW     
;//          <0=> HW Watchdog <1=> SW Watchdog
;//   <o0.1> nRST_STOP  <i> Generate Reset when entering STOP Mode
;//          <0=>  Reset generated when entering Stop mode <1=>  No reset generated
;//   <o0.2> nRST_STDBY <i> Generate Reset when entering Standby Mode
;//          <0=> Reset generated when entering Standby mode <1=> No reset generated
;//	  <o0.4> nBOOT1		<i>	Together with the BOOT0 pin, this bit selects Boot mode from the main Flash memory, SRAM or System memory.
;// 		 <1=> Boot from System FLASH when Boot0 = 1 <0=> Boot from Embedded SRAM when Boot0 = 1
;//	  <o0.5> VDDA_MONITOR <i> This bit selects the analog monitoring on the VDDA power source
;//			 <0=> Disabled <1=> Enabled
;//	  <o0.6> SRAM_Parity_Check	 <i> The SRAM hardware parity check is disabled by default. This bit allows the user to enable the SRAM hardware parity check
;//			 <0=> Enabled  <1=> Disabled
;// </h>
USER          EQU     0x37
nUSER         EQU     USER:EOR:0xFF

;// <h> User Data
;//   <o0> Byte 0 <0x00-0xFF>
;//   <o1> Byte 1 <0x00-0xFF>
;// </h>
DATA0           EQU     0xF0
DATA1           EQU     0xF1
nDATA0          EQU     DATA0:EOR:0xFF
nDATA1          EQU     DATA1:EOR:0xFF
;// </e>


                IF      FLASH_OPT <> 0
                AREA    |.ARM.__AT_0x1FFFF800|, CODE, READONLY
                IF      RDP == 0x00
                DCB     0xAA,  0x55
                ELIF 	RDP == 0x01
                DCB     0xBB,  0x44
				ELIF 	RDP == 0x02
                DCB     0xCC,  0x33
                ENDIF
                DCB     USER,  nUSER
                DCB     DATA0, nDATA0, DATA1, nDATA1
                DCB     WRP0,  nWRP0,  WRP1,  nWRP1
                DCB     WRP2,  nWRP2,  WRP3,  nWRP3
                ENDIF


				END