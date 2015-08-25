/*********************************************************************
*		DACT Analytics LLC											 *
*       File modified for QED project								 *
**********************************************************************
----------------------------------------------------------------------
Author      : Chris Sweet
date        : 11/07/2014
--------------------END-OF-DACT-Info.---------------------------------
*/
/*********************************************************************
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2013  SEGGER Microcontroller GmbH & Co. KG       *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.22 - Graphical user interface for embedded applications **
All  Intellectual Property rights  in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product.  This file may
only be used in accordance with the following terms:

The software has been licensed to  NXP Semiconductors USA, Inc.  whose
registered  office  is  situated  at 411 E. Plumeria Drive, San  Jose,
CA 95134, USA  solely for  the  purposes  of  creating  libraries  for
NXPs M0, M3/M4 and  ARM7/9 processor-based  devices,  sublicensed  and
distributed under the terms and conditions of the NXP End User License
Agreement.
Full source code is available at: www.segger.com

We appreciate your understanding and fairness.
----------------------------------------------------------------------
File        : Main.c
Purpose     : Call of MainTask
--------------------END-OF-HEADER-------------------------------------
*/

#include "stdio.h"
#ifndef _WINDOWS
#include "board.h"
#include "LCDConf.h"
#endif

#ifdef __CROSSWORKS_ARM
extern void __low_level_init(); // hwconf.c
#endif

#define LCD_WIDTH       BOARD_LCD.PPL
#define LCD_HEIGHT      BOARD_LCD.LPP

void MainTask(void);  // Defined in SEGGERDEMO.c

static int mode_poll;   /* Poll/Interrupt mode flag */

/* Set I2C mode to polling/interrupt */
static void i2c_set_mode(I2C_ID_T id, int polling)
{
	if(!polling) {
		mode_poll &= ~(1 << id);
		Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandler);
		NVIC_EnableIRQ(id == I2C0 ? I2C0_IRQn : I2C1_IRQn);
	} else {
		mode_poll |= 1 << id;
		NVIC_DisableIRQ(id == I2C0 ? I2C0_IRQn : I2C1_IRQn);
		Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandlerPolling);
	}
}

/* Initialize the I2C bus */
static void i2c_app_init(I2C_ID_T id, int speed)
{
	Board_I2C_Init(id);

	/* Initialize I2C */
	Chip_I2C_Init(id);
	Chip_I2C_SetClockRate(id, speed);

	/* Set default mode to interrupt */
	i2c_set_mode(id, 0);
}

/*********************************************************************
*
*       main
*/
int main(void) {

	DEBUGOUT("Hi Chris\r\n");

	//initialize hardware
	Board_Init();

	//I2C init for EA 480x272 display
	i2c_app_init(I2C0, SPEED_100KHZ);

	//LCD initialize
	Board_LCD_Init();

	Chip_GPIO_Init(LPC_GPIO);


#ifdef __CROSSWORKS_ARM
	__low_level_init();
#endif
#ifndef _WINDOWS
	HW_X_Config();      // Initialization of Hardware

#endif
	MainTask();         // emWin application

	return 0;
}

