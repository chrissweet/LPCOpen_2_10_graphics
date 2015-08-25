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
*        (c) 1996 - 2012  SEGGER Microcontroller GmbH & Co. KG       *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.22 - Graphical user interface for embedded applications **
All  Intellectual Property rights  in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product.  This file may
only be used in accordance with a license and should not be re-
distributed in any way. We appreciate your understanding and fairness.
----------------------------------------------------------------------
File        : LCDConf.c
Purpose     : Display controller configuration (single layer)
---------------------------END-OF-HEADER------------------------------
*/

#include "LCDConf.h"
#include "GUI.h"
#include <stdio.h>
#include <stddef.h>

#ifndef _WINDOWS
#include "GUIDRV_Lin.h"
#include <string.h>
#endif

/*********************************************************************
*
*       Layer configuration (to be modified)
*
**********************************************************************
*/

#define TOUCH_TIMER_INTERVAL  10

//
// Display framebuffer size
// These values can be reduced if using a smaller display and might have
// to be adjusted if using virtual screens. The values will be checked
// during emWin init.
//
#define FB_XSIZE  800
#define FB_YSIZE  480

/* pointer to frame buffer */
static uint16_t *framebuffer = (uint16_t *) FRAMEBUFFER_ADDR;

//
// Physical display size
//
#ifndef _WINDOWS
  #define XSIZE_PHYS  BOARD_LCD.PPL
  #define YSIZE_PHYS  BOARD_LCD.LPP
#else
  #define XSIZE_PHYS  FB_XSIZE
  #define YSIZE_PHYS  FB_YSIZE
#endif

//
// Virtual display size
//
#define VXSIZE_PHYS (XSIZE_PHYS)
#define VYSIZE_PHYS (YSIZE_PHYS)

//
// Color conversion
//
#define COLOR_CONVERSION  GUICC_M565

//
// Pixel width in bytes
//
#define PIXEL_WIDTH  2

//
// Display driver
//
#define DISPLAY_DRIVER_TRULY  &GUIDRV_Lin_OSX_16_API
#define DISPLAY_DRIVER_OTHER  &GUIDRV_Lin_16_API

//
// Touch controller settings for 4.3" display board
//
#define TOUCH_BOARD_480_272_AD_LEFT    500
#define TOUCH_BOARD_480_272_AD_RIGHT   3650
#define TOUCH_BOARD_480_272_AD_TOP     3300
#define TOUCH_BOARD_480_272_AD_BOTTOM  610

/*********************************************************************
*
*       Configuration checking
*
**********************************************************************
*/
#ifndef   VXSIZE_PHYS
  #define VXSIZE_PHYS XSIZE_PHYS
#endif
#ifndef   VYSIZE_PHYS
  #define VYSIZE_PHYS YSIZE_PHYS
#endif
//#ifndef   VRAM_ADDR_PHYS
//  #define VRAM_ADDR_PHYS  0
//#endif
//#ifndef   VRAM_ADDR_VIRT
//  #define VRAM_ADDR_VIRT  0
//#endif

#ifndef   XSIZE_PHYS
  #error Physical X size of display is not defined!
#endif
#ifndef   YSIZE_PHYS
  #error Physical Y size of display is not defined!
#endif
#ifndef   COLOR_CONVERSION
  #error Color conversion not defined!
#endif

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/

//
// LCD types
//
enum {
  DISPLAY_TRULY_240_320,
  DISPLAY_BOARD_480_272,
  DISPLAY_BOARD_800_480
};


#ifndef   ABS
  #define ABS(x)                       (((int32_t)(x)) < 0 ? (-x) : (x))
#endif

/*********************************************************************
*
*       Defines, sfrs
*
**********************************************************************
*/

#define MATRIX_ARB  (*(volatile U32*)(0x400FC188))

/*********************************************************************
*
*       Types
*
**********************************************************************
*/

typedef struct {
  U16 LcdParams;
  U16 LcdInit;
  U16 PowerDown;
  U16 Touch;
  U16 End;
} EEPROM_CONFIG_OFFS;

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/

//
// LCD
//
static U8                 _Display = DISPLAY_BOARD_480_272;

#if GUI_SUPPORT_TOUCH  // Used when touch screen support is enabled

//
// Touch screen results
//
static int _TouchX;
static int _TouchY;
static U8  _PenIsDown;
static U8  _IsInited   = 0;

#endif

/*********************************************************************
*
*       Local functions
*
**********************************************************************
*/

/*********************************************************************
*
*       Local functions for display selection and LCDC init
*
**********************************************************************
*/

/*********************************************************************
*
*       Local functions for LCDs other than Truly
*
**********************************************************************
*/

/*********************************************************************
*
*       Local functions for display selection and general init
*
**********************************************************************
*/

/*********************************************************************
*
*       _InitController
*
* Function description
*   Initializes the LCD controller and touch screen
*/
#ifndef _WINDOWS
static void _InitController(unsigned LayerIndex) {
	DEBUGOUT("Init LCD Controller\r\n");

	Chip_LCD_Init(LPC_LCD, (LCD_CONFIG_T *) &BOARD_LCD);
	Chip_LCD_SetUPFrameBuffer(LPC_LCD, (void *) framebuffer);

	//Board_InitTouchController();

	Chip_LCD_PowerOn(LPC_LCD);

	/* Turn on backlight */
	Board_SetLCDBacklight(1);

	//enable LCD and backlight
	enableLCD();

	//
	// Set display size and video-RAM address
	//
	LCD_SetSizeEx (XSIZE_PHYS, YSIZE_PHYS, LayerIndex);
	LCD_SetVSizeEx(VXSIZE_PHYS, VYSIZE_PHYS, LayerIndex);
	LCD_SetVRAMAddrEx(LayerIndex, (void*)framebuffer);
	//
	// Init touch
	//
#if GUI_SUPPORT_TOUCH  // Used when touch screen support is enabled
	Board_InitTouchController();
	U32 TouchOrientation;
	int TouchADLeft;
	int TouchADRight;
	int TouchADTop;
	int TouchADBottom;

	TouchADLeft   = TOUCH_BOARD_480_272_AD_LEFT;//TSC_Config.ad_left;//
	TouchADRight  = TOUCH_BOARD_480_272_AD_RIGHT;//TSC_Config.ad_right;//
	TouchADTop    = TOUCH_BOARD_480_272_AD_TOP;//TSC_Config.ad_top;//
	TouchADBottom = TOUCH_BOARD_480_272_AD_BOTTOM;//TSC_Config.ad_bottom;//

    //
    // Calibrate touch
    //
    TouchOrientation = (GUI_MIRROR_X * LCD_GetMirrorXEx(0)) |
                       (GUI_MIRROR_Y * LCD_GetMirrorYEx(0)) |
                       (GUI_SWAP_XY  * LCD_GetSwapXYEx (0)) ;
    GUI_TOUCH_SetOrientation(TouchOrientation);
    GUI_TOUCH_Calibrate(GUI_COORD_X, 0, XSIZE_PHYS, TouchADLeft, TouchADRight);
    GUI_TOUCH_Calibrate(GUI_COORD_Y, 0, YSIZE_PHYS, TouchADTop , TouchADBottom);

	_IsInited = 1;

#endif
}
#endif

/*********************************************************************
*
*       Global functions for display init
*
**********************************************************************
*/
void HW_X_Config(void) {
	DEBUGOUT("In HX_X\r\n");

	SysTick_Config(Chip_Clock_GetSystemClockRate() / 1000);

}

/*********************************************************************
*
*       LCD_X_Config
*
* Purpose:
*   Called during the initialization process in order to set up the
*   display driver configuration.
*/
void LCD_X_Config(void) {
  const GUI_DEVICE_API * pDriver;

  #ifndef _WINDOWS
    // set display
    _Display = DISPLAY_BOARD_480_272;
  #endif
  //
  // Check framebuffer size
  //
  #ifndef _WINDOWS
    if ((FB_XSIZE * FB_YSIZE) < (VXSIZE_PHYS * VYSIZE_PHYS)) {
      while (1);  // Error, framebuffer too small
    }
  #endif
  //
  // Set display driver and color conversion for 1st layer
  //
  #ifndef _WINDOWS
  if (_Display == DISPLAY_TRULY_240_320) {
    pDriver = DISPLAY_DRIVER_TRULY;
  } else {
    pDriver = DISPLAY_DRIVER_OTHER;
  }
  #else
  pDriver = GUIDRV_WIN32;
  #endif
  GUI_DEVICE_CreateAndLink(pDriver, COLOR_CONVERSION, 0, 0);
  //
  // Display driver configuration, required for Lin-driver
  //
  LCD_SetPosEx(0, 0, 0);
  if (LCD_GetSwapXYEx(0)) {
    LCD_SetSizeEx  (0, YSIZE_PHYS , XSIZE_PHYS);
    LCD_SetVSizeEx (0, VYSIZE_PHYS, VXSIZE_PHYS);
  } else {
    LCD_SetSizeEx  (0, XSIZE_PHYS , YSIZE_PHYS);
    LCD_SetVSizeEx (0, VXSIZE_PHYS, VYSIZE_PHYS);
  }
  LCD_SetVRAMAddrEx(0, (void*)framebuffer);
  //
  // Set user palette data (only required if no fixed palette is used)
  //
  #if defined(PALETTE)
    LCD_SetLUTEx(0, PALETTE);
  #endif
}

/*********************************************************************
*
*       LCD_X_DisplayDriver
*
* Purpose:
*   This function is called by the display driver for several purposes.
*   To support the according task the routine needs to be adapted to
*   the display controller. Please note that the commands marked with
*   'optional' are not cogently required and should only be adapted if
*   the display controller supports these features.
*
* Parameter:
*   LayerIndex - Index of layer to be configured
*   Cmd        - Please refer to the details in the switch statement below
*   pData      - Pointer to a LCD_X_DATA structure
*
* Return Value:
*   < -1 - Error
*     -1 - Command not handled
*      0 - Ok
*/
int LCD_X_DisplayDriver(unsigned LayerIndex, unsigned Cmd, void * pData) {
  #ifndef _WINDOWS
  LCD_X_SETORG_INFO * pSetOrg;
  #endif
  int r;

  (void) LayerIndex;

  switch (Cmd) {
  //
  // Required
  //
  case LCD_X_INITCONTROLLER:
    //
    // Called during the initialization process in order to set up the
    // display controller and put it into operation. If the display
    // controller is not initialized by any external routine this needs
    // to be adapted by the customer...
    //
    // ...
    #ifndef _WINDOWS
      _InitController(0);
    #endif
    return 0;
  case LCD_X_SETORG:
    //
    // Required for setting the display origin which is passed in the 'xPos' and 'yPos' element of p
    //
    #ifndef _WINDOWS
	  DEBUGOUT("LCD_X_SETORG\r\n");

      pSetOrg = (LCD_X_SETORG_INFO *)pData;
      //
      // Set start address for display data and enable LCD controller
      //
      Chip_LCD_SetUPFrameBuffer(LPC_LCD, (void *) (framebuffer + pSetOrg->yPos * YSIZE_PHYS * PIXEL_WIDTH));

    #endif
    return 0;
  default:
    r = -1;
  }
  return r;
}

/*********************************************************************
*
*       Global functions for GUI touch
*
**********************************************************************
*/

#if GUI_SUPPORT_TOUCH  // Used when touch screen support is enabled

/*********************************************************************
*
*       GUI_TOUCH_X_ActivateX()
*
* Function decription:
*   Called from GUI, if touch support is enabled.
*   Switches on voltage on X-axis,
*   prepares measurement for Y-axis.
*   Voltage on Y-axis is switched off.
*/
void GUI_TOUCH_X_ActivateX(void) {
}

/*********************************************************************
*
*       GUI_TOUCH_X_ActivateY()
*
* Function decription:
*   Called from GUI, if touch support is enabled.
*   Switches on voltage on Y-axis,
*   prepares measurement for X-axis.
*   Voltage on X-axis is switched off.
*/
void GUI_TOUCH_X_ActivateY(void) {
}

/*********************************************************************
*
*       GUI_TOUCH_X_MeasureX()
*
* Function decription:
*   Called from GUI, if touch support is enabled.
*   Measures voltage of X-axis.
*/
int  GUI_TOUCH_X_MeasureX(void) {
  return _TouchX;
}

/*********************************************************************
*
*       GUI_TOUCH_X_MeasureY()
*
* Function decription:
*   Called from GUI, if touch support is enabled.
*   Measures voltage of Y-axis.
*/
int  GUI_TOUCH_X_MeasureY(void) {
  return _TouchY;
}


/*********************************************************************
*
*       ExecTouch
*
* Function description
*   Check for new touch event. Static x, y coordinates will be updated
*   by the _CheckUpdateTouch() routine. If no touch event has occurred
*   we do nothing. Is called by SysTickHandler().
*/
#ifndef _WINDOWS
void ExecTouch(void) {

  GUI_PID_STATE State;
  bool HasUpdate;
  int16_t tmp_x = -1, tmp_y = -1;


  if (_IsInited == 0) {
    return;
  }

  HasUpdate = Board_GetTouchPosRaw((int16_t *) &tmp_x, (int16_t *) &tmp_y);

  //HasUpdate = _CheckUpdateTouch();
  //_SSP_SendCmd(PWRDOWN);
  if (HasUpdate) {
	_TouchX = tmp_x;
	_TouchY = tmp_y;
	DEBUGOUT("Update %d, %d\r\n",_TouchX,_TouchY);

    _PenIsDown = 1;
    GUI_TOUCH_Exec();
  } else if (_PenIsDown) {
	DEBUGOUT("Pen up %d, %d\r\n",_TouchX,_TouchY);

    //
    // No further touch event after at least one touch event means we have
    // lift the pen from the touch screen which means a click.
    //
    _PenIsDown = 0;
    GUI_PID_GetState(&State);
    State.Pressed = 0;
    GUI_PID_StoreState(&State);
  }
}
#endif

#else

#ifndef _WINDOWS
void ExecTouch(void) {
}
#endif

#endif  // GUI_SUPPORT_TOUCH

/*********************************************************************
*
*       Extern data
*
**********************************************************************
*/
extern volatile int TimeMS;  // Defined in GUI_X.c

/*********************************************************************
*
*       Global functions
*
**********************************************************************
*/
/*********************************************************************
*
*       SysTick_Handler()
*
* Function description
*   This is the code that gets called when the processor receives a
*   _SysTick exception.
*/
#ifdef __cplusplus
extern "C" {
#endif
  void SysTick_Handler(void);      // Avoid warning, Systick_Handler is not prototyped in any CMSIS header
#ifdef __cplusplus
}
#endif

void SysTick_Handler(void) {
  static int TouchCnt = 0;

  if (TouchCnt == TOUCH_TIMER_INTERVAL) {
    ExecTouch();
    TouchCnt = 0;
  } else {
    TouchCnt++;
  }
  TimeMS++;
}

/*EA 480x272 LCD I2C back-light/enable control*************************
 * Uses PCA9532 I2C device
 * LED0->LCD_3v3_EN
 * LED1->LCD_5V_EN
 * LED4->LCD_DISP_EN
 * LED6->LCD_RESET
 * LED7->BL_CONTRAST2
 * LED8->BL_CONTRAST
 * LED15->EEPROM write protect
 */

//~~~~General I2C control~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
#define DEFAULT_I2C          I2C0

#define I2C_EEPROM_BUS       DEFAULT_I2C
#define I2C_IOX_BUS          DEFAULT_I2C

static I2C_ID_T i2cDev = DEFAULT_I2C; /* Currently active I2C device */

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* State machine handler for I2C0 and I2C1 */
static void i2c_state_handling(I2C_ID_T id)
{
	if (Chip_I2C_IsMasterActive(id)) {
		Chip_I2C_MasterStateHandler(id);
	} else {
		Chip_I2C_SlaveStateHandler(id);
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/
/**
 * @brief	I2C Interrupt Handler
 * @return	None
 */
void I2C1_IRQHandler(void)
{
	i2c_state_handling(I2C1);
}

/**
 * @brief	I2C0 Interrupt handler
 * @return	None
 */
void I2C0_IRQHandler(void)
{
	i2c_state_handling(I2C0);
}

//~~~~PCA9532 Interface functions~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
//
// PCA9532 (I2C)
//
#define PCA9532_I2C_ADDR         0x64  // Address of the PCA9532 on the display PCB
#define PCA9532_DISP_3V3_EN_BIT  0
#define PCA9532_DISP_5V_EN_BIT   1
#define PCA9532_DISP_EN_BIT      4
#define PCA9532_DISP_BL_BIT      8
#define I2C_PCA9532_PWM0		 2

//
// Register addresses
//
#define INPUT0  0
#define INPUT1  1
#define PSC0    2
#define PWM0    3
#define PSC1    4
#define PWM1    5
#define LS0     6
#define LS1     7
#define LS2     8
#define LS3     9

/*********************************************************************
*
*       Static functions
*
**********************************************************************
*/
/* Very simple (inaccurate) delay function */
static void delay(uint32_t i)
{
	while (i--) {}
}

/* Delay in miliseconds  (cclk = 120MHz) */
static void delayMs(uint32_t ms)
{
	delay(ms * 40000);
}

/*********************************************************************
*
*       _SetPinSelect
*
* Function description
*   Sets pin function select for one pin.
*
* Return value
*      0: O.K.
*   != 0: Error
*/
static uint8_t _SetPinSelect(uint8_t Addr, uint8_t Pin, uint8_t State) {
  uint8_t Data[2];
  uint8_t r, tmp;
  I2C_XFER_T xfer;

  xfer.slaveAddr = Addr;


  if        (Pin < 4) {
    Data[0]  = LS0;
  } else if (Pin < 8) {
    Data[0]  = LS1;
    Pin     -= 4;
  } else if (Pin < 12) {
    Data[0]  = LS2;
    Pin     -= 8;
  } else if (Pin < 16) {
    Data[0]  = LS3;
    Pin     -= 12;
  } else {
    return 1;  // Error, invalid pin
  }
  Pin *= 2;
  xfer.txBuff = &Data[0];
  xfer.txSz = 1;
  xfer.rxBuff = &Data[1];
  xfer.rxSz = 1;
  //r    = I2C_WriteRead(I2CBaseAddr, Addr, &Data[0], 1, &Data[1], 1);
  Chip_I2C_MasterTransfer(i2cDev, &xfer);

  r = 1;

  if (xfer.status == I2C_STATUS_DONE) {
    Data[1] &= ~(0x3   << Pin);
    Data[1] |=  (State << Pin);
    xfer.txBuff = &Data[0];
    xfer.txSz = 2;
    tmp = Chip_I2C_MasterSend(i2cDev, xfer.slaveAddr, xfer.txBuff, xfer.txSz);
    if( tmp == 2){ //if OK set R
    	r = 0;
    }else{
    	r = 1;
    }
    //I2C_Write(I2CBaseAddr, Addr, Data, 2);
  }

  return r;
}

/*********************************************************************
*
*       Global functions
*
**********************************************************************
*/

/*********************************************************************
*
*       I2C_PCA9532_GetPinState
*
* Function description
*   Retrieves the state for all 16 pins.
*
* Return value
*      0: O.K.
*   != 0: Error
*/
uint8_t I2C_PCA9532_GetPinState(uint8_t Addr, uint16_t * pState) {
  uint8_t  Register;
  uint8_t  r;
  I2C_XFER_T xfer;

  xfer.slaveAddr = Addr;
  xfer.txBuff = &Register;
  xfer.txSz = 1;
  xfer.rxBuff = (uint8_t*)pState;
  xfer.rxSz = 1;

  Register = INPUT0;
  Chip_I2C_MasterTransfer(i2cDev, &xfer);
  r = 1;

  //r        = I2C_WriteRead(I2CBaseAddr, Addr, &Register, 1, (uint8_t*)pState, 1);
  if (xfer.status == I2C_STATUS_DONE/*r == 0*/) {
    Register++;  // Set to INPUT1
    xfer.rxBuff = (uint8_t*)pState + 1;
    xfer.rxSz = 1;
    Chip_I2C_MasterTransfer(i2cDev, &xfer);
    if (xfer.status == I2C_STATUS_DONE) {
    	r = 0;
    }
    //r = I2C_WriteRead(I2CBaseAddr, Addr, &Register, 1, (uint8_t*)pState + 1, 1);
  }
  return r;
}

/*********************************************************************
*
*       I2C_PCA9532_SetPinSelect
*
* Function description
*   Sets pin function select for one pin.
*
* Return value
*      0: O.K.
*   != 0: Error
*/
uint8_t I2C_PCA9532_SetPinSelect(uint8_t Addr, uint8_t Pin, uint8_t State) {
  uint8_t r;

  r = _SetPinSelect(Addr, Pin, State);
  return r;
}

/*********************************************************************
*
*       I2C_PCA9532_SetPwm
*
* Function description
*   Sets the timing for PWM0 or PWM1 and assigns the pin to the
*   according PWM.
*
* Return value
*      0: O.K.
*   != 0: Error
*/
uint8_t I2C_PCA9532_SetPwm(uint8_t Addr, uint8_t Pin, uint8_t PwmNumber, uint8_t Pwm, uint8_t Psc) {
  uint8_t Data[2];
  uint8_t r, tmp;

  I2C_XFER_T xfer;

  if (PwmNumber > 1) {
    return 1;  // Error, invalid PWM number
  }

  if (PwmNumber) {
    Data[0] = PSC1;
  } else {
    Data[0] = PSC0;
  }
  Data[1] = Psc;

  xfer.slaveAddr = Addr;
  xfer.txBuff = Data;
  xfer.txSz = 2;
  tmp = Chip_I2C_MasterSend(i2cDev, xfer.slaveAddr, xfer.txBuff, xfer.txSz);
  if( tmp == 2 ){
	  r = 0;
  }else{
	  r = 1;
  }
  //r       = I2C_Write(I2CBaseAddr, Addr, Data, 2);
  if (r == 0) {
    if (PwmNumber) {
      Data[0] = PWM1;
    } else {
      Data[0] = PWM0;
    }
    Data[1] = Pwm;

    xfer.txBuff = Data;
    xfer.txSz = 2;
    tmp = Chip_I2C_MasterSend(i2cDev, xfer.slaveAddr, xfer.txBuff, xfer.txSz);
    if( tmp == 2 ){
  	  r = 0;
    }else{
  	  r = 1;
    }
    //r       = I2C_Write(I2CBaseAddr, Addr, Data, 2);

    if (r == 0) {
      r = _SetPinSelect(Addr, Pin, I2C_PCA9532_PWM0 + PwmNumber);
    }
  }
  return r;
}

//~~~~Enable display and backlight~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
uint8_t enableLCD(){
	int tmp;
	uint8_t PwmDuty;

	tmp = I2C_PCA9532_SetPinSelect(PCA9532_I2C_ADDR, PCA9532_DISP_EN_BIT, 1);
	DEBUGOUT("Set display EN to 1, %d.\r\n",tmp);

	PwmDuty = (255 * 100) / 100;
	I2C_PCA9532_SetPwm(PCA9532_I2C_ADDR, PCA9532_DISP_BL_BIT, 0, PwmDuty, 0);
	I2C_PCA9532_SetPinSelect(PCA9532_I2C_ADDR, PCA9532_DISP_3V3_EN_BIT, 0);
	//delay 100ms
	delayMs(100);
	//
	I2C_PCA9532_SetPinSelect(PCA9532_I2C_ADDR, PCA9532_DISP_3V3_EN_BIT, 1);
	//delay 100ms
	delayMs(100);
	//
	I2C_PCA9532_SetPinSelect(PCA9532_I2C_ADDR, PCA9532_DISP_EN_BIT, 0);
	//delay 10ms
	delayMs(10);
	//
	PwmDuty = (255 * 0) / 100;
	I2C_PCA9532_SetPwm(PCA9532_I2C_ADDR, PCA9532_DISP_BL_BIT, 0, PwmDuty, 0);
	DEBUGOUT("Initialized display.\r\n");

	return tmp;
}

/*End EA 480x272 LCD I2C backlight/enable control********************/

/*************************** End of file ****************************/
