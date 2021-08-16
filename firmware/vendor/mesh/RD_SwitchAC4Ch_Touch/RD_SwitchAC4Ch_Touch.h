/*
 * RD_SwitchAC4Ch_Touch.h
 *
 *  Created on: Apr 16, 2021
 *      Author: dangv
 */

#ifndef RD_SWITCHAC4CH_TOUCH_H_
#define RD_SWITCHAC4CH_TOUCH_H_
#include "proj/tl_common.h"
#include "proj/mcu/watchdog_i.h"
#include "vendor/common/user_config.h"
#include "proj_lib/rf_drv.h"
#include "proj_lib/pm.h"
#include "proj_lib/ble/blt_config.h"
#include "proj_lib/ble/ll/ll.h"
#include "proj_lib/sig_mesh/app_mesh.h"

#include "../RD_MessData/RD_MessData.h"

#define 	X1_TOUCH_SWITCH				1
#define 	X2_TOUCH_SWITCH				2
#define 	X3_TOUCH_SWITCH				3
#define 	X4_TOUCH_SWITCH				4
#define 	HIPOW_TOUCH_SWITCH			5
#define 	CURTAINS_TOUCH_SWITCH		6

#define 	TYPE_HARDWARE_SWITCH		CURTAINS_TOUCH_SWITCH

#define     I2C_CLK_SPEED               200000
#if(TYPE_HARDWARE_SWITCH == CURTAINS_TOUCH_SWITCH)
//	#define 	HI_PIN						GPIO_PC4
//	#define 	DETECT_ZERO_PIN				GPIO_PB7
//
//	#define 	LED1_PIN					GPIO_PC1				// 0-->RED    1--> Green
//	#define		LED2_PIN					GPIO_PC0
//	#define 	LED3_PIN					GPIO_PB1
//	#define 	LED4_PIN					0						// RD_EDIT not use
//
//	#define 	OPEN_LED					LED3_PIN				 // 0-->RED    1--> Green
//	#define		STOP_LED					LED2_PIN
//	#define 	CLOSE_LED					LED1_PIN
//	#define 	LED_RED_ON					0
//	#define		LED_GREEN_ON				1
//
//	#define		RELAY1_PIN					GPIO_PB4				// 1  = on
//	#define		RELAY2_PIN					GPIO_PB5
//	#define 	RELAY3_PIN					GPIO_PB6
//	#define 	RELAY4_PIN					0						// RD_EDIT not use

	#define 	DETECT_ZERO_PIN				GPIO_PB7

	#define		BT1_PIN						GPIO_PC4
	#define		BT2_PIN						GPIO_PC3
	#define		BT3_PIN						GPIO_PA1
	#define 	BT4_PIN						GPIO_PC2

	#define 	LED1_PIN					GPIO_PD3				// 0-->RED    1--> Green
	#define		LED2_PIN					GPIO_PC1
	#define 	LED3_PIN					GPIO_PD4
	#define 	LED4_PIN					GPIO_PC0

	#define 	LED_RED_ON					0
	#define		LED_GREEN_ON				1

	#define		RELAY1_PIN					GPIO_PB1				// 1  = on
	#define		RELAY2_PIN					GPIO_PB4
	#define 	RELAY3_PIN					GPIO_PB5
	#define 	RELAY4_PIN					GPIO_PB6

	#define 	OPEN_BUTTON					BT3_PIN
	#define		STOP_BUTTON					BT2_PIN
	#define 	CLOSE_BUTTON				BT1_PIN

	#define 	OPEN_LED					LED3_PIN				 // 0-->RED    1--> Green
	#define		STOP_LED					LED2_PIN
	#define 	CLOSE_LED					LED1_PIN

	#define		OPEN_RELAY					RELAY3_PIN				// RD_EDIT: Note need check
	#define		COMMON_RELAY				RELAY2_PIN
	#define		CLOSE_RELAY					RELAY1_PIN


	#define		BT1_READ		gpio_read(BT1_PIN)
	#define		BT2_READ		gpio_read(BT2_PIN)
	#define		BT3_READ		gpio_read(BT3_PIN)
	#define		BT4_READ		gpio_read(BT4_PIN)

	#define 	TIME_DELAY_TOGGLE_CONTROL   1000

	#define		DELAY_DETECH_TIME			3500					// us

	#define		NUM_CHECK_DETECH_MAX			1000				// time
#endif

#if(TYPE_HARDWARE_SWITCH == X1_TOUCH_SWITCH)
	#define 	HI_PIN						GPIO_PC4
	#define 	DETECT_ZERO_PIN				GPIO_PB7

	#define 	LED1_PIN					GPIO_PB6				// 0-->RED    1--> Green
	#define		LED2_PIN					GPIO_PC1

	#define 	LED_RED_ON					1
	#define		LED_GREEN_ON				0

	#define		RELAY1_PIN					GPIO_PB5				// 1  = on
#endif

#define 	UART_ON 					1

#define		TIME_OUT_RESET				3000					// 4000 loop break pressResetCount
#define		COUNT_DETECT_POWER_OFF		7						// 2ms (timer1 interrupt) * 6 = 12ms zero
#define 	SCENE_MAX					10 // if ++ SCENE_MAX ---> ++ FLASH_DATA_SIZE

#define 	DETECT_ZERO_OFF				0
#define		DETECT_ZERO_BY_ADC			1
#define 	DETECT_ZERO_BY_FAILING		2
#define 	DETECT_ZERO_BY_FAILING_V2	3

#define 	METHOD_DETECT_ZERO			DETECT_ZERO_BY_FAILING_V2

#define 	TIME_HOLD_BT				(700)

#define 	I2C_ADD_CY8   				    (0x37<<1)

#define 	SYNC_COUNTER0_CY8_REG			0xB9				// check pin number press
#define 	SYNC_COUNTER1_CY8_REG			0xDB				// check pin number press
#define 	DIFFERENCE_COUNT_SENSOR0_CY8_REG 0xBA   				// count signal sensor0; 16 bit
#define		BUTTON_STAT_CY8_REG				0xAA				// Button STT    touch/ no touch
#define		LATCHED_BUTTON_STAT_CY8_REG		0xAC				// the last time BUTTON_STAT_CY8_REG
#define 	CTRL_CMD_CY8_REG				0x86				//
#define 	CTRL_CMD_STATUS_CY8_REG			0x88
#define 	CTRL_CMD_ERR_CY8_REG			0x89
#define 	SIZE_OF_DATA_CONFIG			    128
//I2C_HandleTypeDef hi2c2;

//#define 	I2CWrite_CY8_DEF(Slave_Add,Reg_Add, Size, data)    HAL_I2C_Mem_Write(&hi2c2,Slave_Add,Reg_Add, 1 , data,Size,1000);
//#define 	I2CRead_CY8_DEF(Slave_Add, Reg_Add, Size, data)	HAL_I2C_Mem_Read(&hi2c2, Slave_Add, Reg_Add, 1, data, Size, 1000);

/*-----------------------------------Value Config------------------------------------*/
#if(TYPE_HARDWARE_SWITCH == X4_TOUCH_SWITCH)
#define		SENSOR_EN_L						0x32 //0xE3				// active sensor 1-5-4
#define 	SENSOR_EN_H						0x00
#endif

#if(TYPE_HARDWARE_SWITCH == X1_TOUCH_SWITCH)
#define		SENSOR_EN_L						0x20 //0xE3				// active sensor  5
#define 	SENSOR_EN_H						0x00
#endif



#if(TYPE_HARDWARE_SWITCH == CURTAINS_TOUCH_SWITCH)
#define		SENSOR_EN_L						0x32 //0xE3				// active sensor 1-5-4
#define 	SENSOR_EN_H						0x00
#endif
#define 	FSS_EN_L						0x00
#define		FSS_EN_H						0x00
#define 	TOGGLE_EN_L						0x00
#define 	TOGGLE_EN_H						0x00
#define 	LED_ON_EN_L						0x00
#define 	LED_ON_EN_H						0X00
#define 	SENSITIVITY0					0x00
#define 	SENSITIVITY1					0x00
#define 	SENSITIVITY2					0x00
#define 	BASE_THRESHOLD0					0x80
#define 	BASE_THRESHOLD1					0x80
#define 	FINGER_THRESHOLD2				0x80
#define 	FINGER_THRESHOLD3				0x80
#define 	FINGER_THRESHOLD4				0x80
#define 	FINGER_THRESHOLD5				0x80
#define 	FINGER_THRESHOLD6				0x80
#define 	FINGER_THRESHOLD7				0x80
#define 	FINGER_THRESHOLD8				0x80
#define 	FINGER_THRESHOLD9				0x80
#define 	SENSOR_DEBOUNCE					0x03
#define 	BUTTON_HYS						0x0C
#define 	BUTTON_LBR						0x32
#define 	BUTTON_NNT						0x33
#define 	BUTTON_NT						0x33
#define 	PROX_EN							0x00
#define 	PROX_CFG						0x80
#define 	PROX_CFG2						0x05
#define 	PROX_TOUCH_TH0_L				0x00
#define 	PROX_TOUCH_TH0_H				0x02
#define 	PROX_TOUCH_TH1_L				0x00
#define 	PROX_TOUCH_TH1_H				0x02
#define 	PROX_RESOLUTION0				0x00
#define 	PROX_RESOLUTION1				0x00
#define 	PROX_HYS						0x05
#define 	PROX_LBR						0x32
#define 	PROX_NNT						0x14
#define 	PROX_NT							0x14
#define 	PROX_POSITIVE_TH0				0x1E
#define 	PROX_POSITIVE_TH1				0x1E
#define 	PROX_NEGATIVE_TH0				0x1E
#define 	PROX_NEGATIVE_TH1				0x1E
#define 	LED_ON_TIME						0x00
#define 	BUZZER_CFG						0x01
#define		BUZZER_ON_TIME					0x01
#define 	GPO_CFG							0x00
#define 	PWM_DUTYCYCLE_CFG0				0x0F
#define 	PWM_DUTYCYCLE_CFG1				0x0F
#define 	PWM_DUTYCYCLE_CFG2				0x0F
#define 	PWM_DUTYCYCLE_CFG3				0x0F
#define 	PWM_DUTYCYCLE_CFG4				0x0F
#define 	SPO_CFG							0x41								// set SPO 1 as host interrupt
#define 	DEVICE_CFG0						0x03
#define 	DEVICE_CFG1						0x01
#define 	DEVICE_CFG2						0x18								// enable automatic thresholds and auto reset 5s
#define 	DEVICE_CFG3						0x00
#define 	I2C_ADDR						0x37
#define 	REFRESH_CTRL					0x06
#define 	STATE_TIMEOUT					0x0A
#define 	SCRATCHPAD0						0x00
#define 	SCRATCHPAD1						0x00
#define		CONFIG_CRC_L					0x01
#define 	CONFIG_CRC_H					0X00
/*-------------------------------------------------------------------------------------*/

/*-----------------------ADD_Register == local of Register_Config[x] -------------------------------------------*/
#define		SENSOR_EN_L_REG_CY8				0x00
#define 	SENSOR_EN_H_REG_CY8				0x01
#define 	FSS_EN_L_REG_CY8				0x02
#define		FSS_EN_H_REG_CY8				0x03
#define 	TOGGLE_EN_L_REG_CY8				0x04
#define 	TOGGLE_EN_H_REG_CY8				0x05
#define 	LED_ON_EN_L_REG_CY8				0x06
#define 	LED_ON_EN_H_REG_CY8				0X07
#define 	SENSITIVITY0_REG_CY8			0x08
#define 	SENSITIVITY1_REG_CY8			0x09
#define 	SENSITIVITY2_REG_CY8			0x0A
#define 	BASE_THRESHOLD0_REG_CY8			0x0C
#define 	BASE_THRESHOLD1_REG_CY8			0x0D
#define 	FINGER_THRESHOLD2_REG_CY8		0x0E
#define 	FINGER_THRESHOLD3_REG_CY8		0x0F
#define 	FINGER_THRESHOLD4_REG_CY8		0x10
#define 	FINGER_THRESHOLD5_REG_CY8		0x11
#define 	FINGER_THRESHOLD6_REG_CY8		0x12
#define 	FINGER_THRESHOLD7_REG_CY8		0x13
#define 	FINGER_THRESHOLD8_REG_CY8		0x14
#define 	FINGER_THRESHOLD9_REG_CY8		0x15
#define 	SENSOR_DEBOUNCE_REG_CY8			0x1C
#define 	BUTTON_HYS_REG_CY8				0x1D
#define 	BUTTON_LBR_REG_CY8				0x1F
#define 	BUTTON_NNT_REG_CY8				0x20
#define 	BUTTON_NT_REG_CY8				0x21
#define 	PROX_EN_REG_CY8					0x26
#define 	PROX_CFG_REG_CY8				0x27
#define 	PROX_CFG2_REG_CY8				0x28
#define 	PROX_TOUCH_TH0_L_REG_CY8		0x2A
#define 	PROX_TOUCH_TH0_H_REG_CY8		0x2B
#define 	PROX_TOUCH_TH1_L_REG_CY8		0x2C
#define 	PROX_TOUCH_TH1_H_REG_CY8		0x2D
#define 	PROX_RESOLUTION0_REG_CY8		0x2E
#define 	PROX_RESOLUTION1_REG_CY8		0x2F
#define 	PROX_HYS_REG_CY8				0x30
#define 	PROX_LBR_REG_CY8				0x32
#define 	PROX_NNT_REG_CY8				0x33
#define 	PROX_NT_REG_CY8					0x34
#define 	PROX_POSITIVE_TH0_REG_CY8		0x35
#define 	PROX_POSITIVE_TH1_REG_CY8		0x36
#define 	PROX_NEGATIVE_TH0_REG_CY8		0x39
#define 	PROX_NEGATIVE_TH1_REG_CY8		0x3A
#define 	LED_ON_TIME_REG_CY8				0x3D
#define 	BUZZER_CFG_REG_CY8				0x3E
#define		BUZZER_ON_TIME_REG_CY8			0x3F
#define 	GPO_CFG_REG_CY8					0x40
#define 	PWM_DUTYCYCLE_CFG0_REG_CY8		0x41
#define 	PWM_DUTYCYCLE_CFG1_REG_CY8		0x42
#define 	PWM_DUTYCYCLE_CFG2_REG_CY8		0x43
#define 	PWM_DUTYCYCLE_CFG3_REG_CY8		0x44
#define 	PWM_DUTYCYCLE_CFG4_REG_CY8		0x45
#define 	SPO_CFG_REG_CY8					0x4C
#define 	DEVICE_CFG0_REG_CY8				0x4D
#define 	DEVICE_CFG1_REG_CY8				0x4E
#define 	DEVICE_CFG2_REG_CY8				0x4F
#define 	DEVICE_CFG3_REG_CY8				0x50
#define 	I2C_ADDR_REG_CY8				0x51
#define 	REFRESH_CTRL_REG_CY8			0x52
#define 	STATE_TIMEOUT_REG_CY8			0x55
#define 	SCRATCHPAD0_REG_CY8				0x7A
#define 	SCRATCHPAD1_REG_CY8				0x7B
#define		CONFIG_CRC_L_REG_CY8			0x7E
#define 	CONFIG_CRC_H_REG_CY8			0x7F
#define		CALC_CRC_REG_CY8				0x94


typedef enum
{
	Button_None		= 0x00U,
	Button_Press	= 0x01U,
	Button_Keeping	= 0x02U
} Button_Stt_Type;
extern Button_Stt_Type button1_Stt;
extern Button_Stt_Type button2_Stt;
extern Button_Stt_Type button3_Stt;
extern Button_Stt_Type button4_Stt;

extern uint8_t		Button1_Hold_Flag;
extern uint8_t		Button2_Hold_Flag;
extern uint8_t		Button3_Hold_Flag;
extern uint8_t		Button4_Hold_Flag;

typedef enum{
	RD_MAINTYPE_HC		= 0,
	RD_MAINTYPE_LAMP   	= 1,
	RD_MAINTYPE_SWITCH  = 2,
	RD_MAINTYPE_SENSOR 	= 3,
	RD_MAINTYPE_REMOTE	= 4,
	RD_MAINTYPE_SPEAKER	= 5
} RD_TYPE_MAINDEVICE;

typedef enum{
	RD_SWITCHTYPE_ONOFF	= 1,
	RD_SWITCHTYPE_TOUCH = 2,
	RD_SWITCHTYPE_SCENE	= 3
} RD_TYPE_SWITCH;

typedef enum{
	RD_TYPE_LAMP_ONOFF   	= 0,
	RD_TYPE_LAMP_CCT  		= 1,
	RD_TYPE_LAMP_RGB 		= 2,
	RD_TYPE_LAMP_CCT_RGB 	= 3
} RD_TYPE_LAMP;

typedef enum{
	RD_TYPE_SCENESWITCH_AC   		= 2,
	RD_TYPE_SCENESWITCH_DC  		= 1,
	RD_TYPE_SCENESWITCH_TOUCH		= 3
} RD_TYPE_SCENE_SWITCH;

typedef enum{
	RD_TYPE_TOUCHSWITCH1CH   		= 1,
	RD_TYPE_TOUCHSWITCH2CH   		= 2,
	RD_TYPE_TOUCHSWITCH3CH   		= 3,
	RD_TYPE_TOUCHSWITCH4CH   		= 4,
	RD_TYPE_TOUCHSWITCHHIGHPOWER   	= 5,
	RD_TYPE_CURTAIN					= 6
} RD_TYPE_TOUCH_SWITCH;
typedef enum{
	RD_TYPE_SENSOR_PIR   		= 0,
	RD_TYPE_SENSOR_LIGHT 		= 1
} RD_TYPE_SENSOR;


typedef enum{
	RD_TYPE_POWER_NONE   		= 0,
	RD_TYPE_POWER_3W   			= 1,
	RD_TYPE_POWER_5W   			= 2,
	RD_TYPE_POWER_7W   			= 3,
	RD_TYPE_POWER_9W   			= 4,
	RD_TYPE_POWER_15W   		= 5
} RD_TYPE_POWER;

typedef enum
{
	Relay_On		= 0x01U,
	Relay_Off		= 0x00U
} Relay_Stt_Type;

typedef enum
{
	Control_Off		= 0x00u,
	Control_On		= 0x01,
	Control_Stop	= 0x02
} Control_Stt_type;
extern Relay_Stt_Type 	relay1_Stt;
extern Relay_Stt_Type	relay2_Stt;
extern Relay_Stt_Type 	relay3_Stt;
extern Relay_Stt_Type	relay4_Stt;

extern Control_Stt_type Control_Stt_Val; // bien dieu khien rem cua
extern uint8_t			Motor_Stt;	  // trang thai cua rem. 0--> 100%

extern uint32_t		Time_Control_Max ;  // 10000 ms
extern long		Time_Control_State_Now ; //  ms/ unit

extern uint8_t			Pos_Flag ;   // Control curtain to any position. 0-->100% Open.  Flag=1 ---> go to
extern uint8_t			Position_Taget;	// Position want control. use in RD_Curtains_ScanControlToPos()   and RD_Curtains_Goto()


extern uint8_t 			countPower;
extern uint8_t			powerSaved;

extern uint8_t 			calib_Stt;
/*
 * Init GPIO, ADC, Interrupt for Switch
 */
void RD_SwitchAC4Ch_Init(void);
void RD_SwitchAC4Ch_Touch(void);
/*
 * Loop for Swtich
 * +Scan touch Button
 * + update status of Button, led, relay,...
 */
void RD_SwitchAC4Ch_Loop(void);
/*
 * check touchSensorDetect
 * Scan Touch button signal and update  Button status
 */
void RD_SwitchAC4Ch_ScanBt(void);

void RD_SwitchAC4Ch_ScanB_V2(void);

void RD_SwitchAC4Ch_UpdateCtr(void);

void RD_SwitchAC4Ch_OnOffZero(GPIO_PinTypeDef RelayPin, GPIO_PinTypeDef LedPin, Relay_Stt_Type OnOff_State);

void RD_SwitchAC4Ch_ScanReset(void);
/*
 *  call in loop
 *  Scan val blink led
 *  set val of blink led by RD_SwitchAc4Ch_BlinkScan
 */
void RD_SwitchAc4Ch_BlinkScan(void);
/*
 * set blink all 4 led
 * cycle: int count cycle blink
 * countToggle: time of 1 cycle, Recommend 100,
 */
void RD_SwitchAc4Ch_BlinkSet(uint8_t cycle, uint16_t countToggle);
/*
 *  Read voltage and check Power
 *  call in Timer1 interrupt
 *  use with SaveDataPowerOff()
 */
void RD_SwitchAc4Ch_CheckPower(void);
/*
 * Scan countPower and save data to flash when detect power Off;
 */
void RD_SwitchAc4Ch_SaveDataPowerOffLoop(void);

/*
 *  Setup data config From Define list to Register_Config[] and setup for CY8 IC
 */
void RD_SwitchAc4Ch_UpdateDataConfigCy8(void);


uint8_t RD_SwitchAc4Ch_I2cRead(unsigned int Add_Reg, unsigned int Add_Reg_Len, unsigned char *data, int dataLen );
uint8_t RD_SwitchAc4Ch_I2cWrite(unsigned int Add_Reg, unsigned int Add_Reg_Len, unsigned char *data, int dataLen);
void RD_SwitchAc4Ch_CY8Init(void);

void RD_SwitchAC4Ch_CheckHold(void);

void RD_Socket_WaitDetechZero(void);
void RD_Curtains_Control(Control_Stt_type Control_Stt);
uint16_t RD_Curtains_Delta_Time_Control(void);
void RD_Curtanins_Update_Control(void);
//  Scan control to Position.  Call in loop. recommen call after RD_Curtanins_Update_Control
void RD_Curtains_ScanControlToPos(void);
// set taget position control.  0-->100% Open
void RD_Curtains_Goto(uint8_t Position);

void RD_Curtain_Calib(void);
#endif /* RD_SWITCHAC4CH_TOUCH_H_ */
