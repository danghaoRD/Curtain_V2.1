/*
 * RD_SwitchAC4Ch_Touch.c
 *
 *  Created on: Apr 16, 2021
 *      Author: dangv
 */

#include "RD_SwitchAC4Ch_Touch.h"
extern uint8_t touchSensorDetect;
uint8_t 	CycleBlink=0;
uint16_t 	CountToggleBlink=0;
uint8_t		Button1_Keep_Flag=0;
uint8_t		Button2_Keep_Flag=0;
uint8_t		Button3_Keep_Flag=0;
uint8_t		Button4_Keep_Flag=0;

uint8_t		Button1_Hold_Flag=0;
uint8_t		Button2_Hold_Flag=0;
uint8_t		Button3_Hold_Flag=0;
uint8_t		Button4_Hold_Flag=0;

uint8_t	Register_Config[128] = {0};

Button_Stt_Type button1_Stt = Button_None;
Button_Stt_Type button2_Stt = Button_None;
Button_Stt_Type	button3_Stt	= Button_None;
Button_Stt_Type button4_Stt	= Button_None;

Relay_Stt_Type 	relay1_Stt 	= Relay_Off;
Relay_Stt_Type	relay2_Stt 	= Relay_Off;
Relay_Stt_Type 	relay3_Stt 	= Relay_Off;
Relay_Stt_Type	relay4_Stt 	= Relay_Off;

Control_Stt_type Control_Stt_Val = Control_Stop; // bien dieu khien rem cua

uint8_t			Motor_Stt=0;	  // trang thai cua rem. 0--> 100%
uint8_t 		pressResetCount	=0;
uint8_t 		countPower		=0;
uint8_t			powerSaved		=0;
uint8_t 		calib_Stt		=0;
uint32_t		Time_Control_Max  = 10000;  // 10000*1ms = 10s
long			Time_Control_State_Now =0;

uint8_t			Pos_Flag =0;   // Control curtain to any position. 0-->100% Open.  Flag=1 ---> go to
uint8_t			Position_Taget=0;	// Position want control. use in RD_Curtains_ScanControlToPos()   and RD_Curtains_Goto()
void RD_SwitchAC4Ch_Init(void)
{

	/*--------------------------------I2C config --------------------------------*/
//	i2c_gpio_set(I2C_GPIO_GROUP_C2C3); 				// SDA-B      SCL-G
//	i2c_master_init(I2C_ADD_CY8, (unsigned char)(CLOCK_SYS_CLOCK_HZ/(4*I2C_CLK_SPEED)));

	/*-------------------------------GPIO Config---------------------------------*/
	//gpio_setup_up_down_resistor(HI_PIN, PM_PIN_UP_DOWN_FLOAT); 		gpio_set_func(HI_PIN, AS_GPIO);
	//gpio_set_output_en(HI_PIN, 0); gpio_set_input_en(HI_PIN, 1); 	gpio_set_interrupt(HI_PIN, POL_FALLING);
	#if( METHOD_DETECT_ZERO == DETECT_ZERO_BY_ADC)
		adc_init();
		adc_base_init(DETECT_ZERO_PIN);
		adc_power_on_sar_adc(1);
	#endif

	#if(METHOD_DETECT_ZERO == DETECT_ZERO_BY_FAILING || METHOD_DETECT_ZERO == DETECT_ZERO_BY_FAILING_V2)
		gpio_setup_up_down_resistor(DETECT_ZERO_PIN, PM_PIN_UP_DOWN_FLOAT); 		gpio_set_func(DETECT_ZERO_PIN, AS_GPIO);
		gpio_set_output_en(DETECT_ZERO_PIN, 0); 									gpio_set_input_en(DETECT_ZERO_PIN, 1);
	#endif

#if(TYPE_HARDWARE_SWITCH == CURTAINS_TOUCH_SWITCH)
	gpio_setup_up_down_resistor(LED1_PIN, PM_PIN_UP_DOWN_FLOAT);	gpio_set_func(LED1_PIN, AS_GPIO);
	gpio_set_input_en(LED1_PIN, 0);									gpio_set_output_en(LED1_PIN, 1);
	gpio_setup_up_down_resistor(LED2_PIN, PM_PIN_UP_DOWN_FLOAT);	gpio_set_func(LED2_PIN, AS_GPIO);
	gpio_set_input_en(LED2_PIN, 0);									gpio_set_output_en(LED2_PIN, 1);
	gpio_setup_up_down_resistor(LED3_PIN, PM_PIN_UP_DOWN_FLOAT);	gpio_set_func(LED3_PIN, AS_GPIO);
	gpio_set_input_en(LED3_PIN, 0);									gpio_set_output_en(LED3_PIN, 1);
//	gpio_setup_up_down_resistor(LED4_PIN, PM_PIN_UP_DOWN_FLOAT);	gpio_set_func(LED4_PIN, AS_GPIO);
//	gpio_set_input_en(LED4_PIN, 0);									gpio_set_output_en(LED4_PIN, 1);

	gpio_setup_up_down_resistor(RELAY1_PIN, PM_PIN_UP_DOWN_FLOAT);	gpio_set_func(RELAY1_PIN, AS_GPIO);
	gpio_set_input_en(RELAY1_PIN, 0);								gpio_set_output_en(RELAY1_PIN, 1);
	gpio_write(relay1_Stt, Relay_Off);
	gpio_setup_up_down_resistor(RELAY2_PIN, PM_PIN_UP_DOWN_FLOAT);	gpio_set_func(RELAY2_PIN, AS_GPIO);
	gpio_set_input_en(RELAY2_PIN, 0);								gpio_set_output_en(RELAY2_PIN, 1);
	gpio_write(relay2_Stt, Relay_Off);
	gpio_setup_up_down_resistor(RELAY3_PIN, PM_PIN_UP_DOWN_FLOAT);	gpio_set_func(RELAY3_PIN, AS_GPIO);
	gpio_set_input_en(RELAY3_PIN, 0);								gpio_set_output_en(RELAY3_PIN,1);
	gpio_write(relay3_Stt, Relay_Off);
	gpio_setup_up_down_resistor(RELAY4_PIN, PM_PIN_UP_DOWN_FLOAT);	gpio_set_func(RELAY4_PIN, AS_GPIO);
//	gpio_set_input_en(RELAY4_PIN, 0);								gpio_set_output_en(RELAY4_PIN, 1);
//	gpio_write(relay4_Stt, Relay_Off);

	gpio_setup_up_down_resistor(OPEN_BUTTON, PM_PIN_PULLUP_10K);	gpio_set_func(OPEN_BUTTON, AS_GPIO);
	gpio_set_input_en(OPEN_BUTTON, 1);								gpio_set_output_en(OPEN_BUTTON, 0);
	gpio_setup_up_down_resistor(CLOSE_BUTTON, PM_PIN_PULLUP_10K);	gpio_set_func(OPEN_BUTTON, AS_GPIO);
	gpio_set_input_en(CLOSE_BUTTON, 1);								gpio_set_output_en(CLOSE_BUTTON, 0);
	gpio_setup_up_down_resistor(STOP_BUTTON, PM_PIN_PULLUP_10K);	gpio_set_func(STOP_BUTTON, AS_GPIO);
	gpio_set_input_en(STOP_BUTTON, 1);								gpio_set_output_en(STOP_BUTTON, 0);
#endif

#if(TYPE_HARDWARE_SWITCH == X1_TOUCH_SWITCH)
	gpio_setup_up_down_resistor(LED1_PIN, PM_PIN_UP_DOWN_FLOAT);	gpio_set_func(LED1_PIN, AS_GPIO);
	gpio_set_input_en(LED1_PIN, 0);									gpio_set_output_en(LED1_PIN, 1);
	gpio_setup_up_down_resistor(LED2_PIN, PM_PIN_UP_DOWN_FLOAT);	gpio_set_func(LED2_PIN, AS_GPIO);
	gpio_set_input_en(LED2_PIN, 0);	 								gpio_set_output_en(LED2_PIN, 1);

	gpio_setup_up_down_resistor(RELAY1_PIN, PM_PIN_UP_DOWN_FLOAT);	gpio_set_func(RELAY1_PIN, AS_GPIO);
	gpio_set_input_en(RELAY1_PIN, 0);								gpio_set_output_en(RELAY1_PIN, 1);
	gpio_write(relay1_Stt, Relay_Off);


#endif
	/*--------------------------------Flash-------------------------------------*/
	RD_Flash_Init();
	RD_Curtains_Control(Control_Stop);

	//RD_SwitchAc4Ch_CY8Init();

	/*--------------------------------------TIMER IQR Init----------------------------------------------*/
}


void RD_SwitchAC4Ch_Touch(void)
{
//	adc_init();
//	adc_base_init(SMOKE_ADC_PIN);
//	adc_power_on_sar_adc(1);
//	gpio_setup_up_down_resistor(LED_B,PM_PIN_UP_DOWN_FLOAT);
//	gpio_set_func(LED_B,AS_GPIO);
//	gpio_set_input_en(LED_B, 0);
//	gpio_set_output_en(LED_B,1);
//	gpio_write(LED_B,1);
}
/*-------------------------------------------Loop Switch-----------------------------------------------------*/
void RD_SwitchAC4Ch_Loop(void)
{

	//RD_SwitchAC4Ch_ScanBt();		// Read Button if detech Interrupt by Touch IC
	RD_SwitchAC4Ch_ScanB_V2();		// READ bUTTON tOUCH
	RD_Curtain_Calib();				// RD_EDIT Test
	RD_Curtanins_Update_Control();	// Update Control Val and control relay
	RD_Curtains_ScanControlToPos(); // Scan control to any Position
	RD_SwitchAC4Ch_ScanReset();		// check button and Scan Factory Reset
	RD_SwitchAc4Ch_BlinkScan();		// Scan Blink Led Mode.

}
/*---------------------------------------------------------------------------------------------------------*/
void RD_SwitchAC4Ch_ScanBt(void)
{
	uint8_t dataBuff[2] = {0};
	uint8_t buttonSttBuff[4] = {2};
	uint8_t i2c_SttBuff=5;
	if(touchSensorDetect == 1)
	{
		touchSensorDetect=0;

		while(RD_SwitchAc4Ch_I2cRead(BUTTON_STAT_CY8_REG, 1, dataBuff, 2) != 0)
			i2c_SttBuff = reg_i2c_status;

		/*------------------------------------check Touch Signal-----------------*/

		buttonSttBuff[0] = (dataBuff[0]>>5) & 0x01; //cs5 0x100000
		buttonSttBuff[1] = (dataBuff[0]>>4) & 0x01; //cs4 0x10000
		buttonSttBuff[2] = (dataBuff[0]>>1) & 0x01; //cs1 0x100000
		buttonSttBuff[3] = (dataBuff[0]>>0) & 0x01; //cs50 0x100000

		char UART_TempSend2[128];
		sprintf(UART_TempSend2,"Data: %x-%x-%x-%x  i2cStt: %x \n",  buttonSttBuff[0], buttonSttBuff[1], buttonSttBuff[2], buttonSttBuff[3], i2c_SttBuff );
		uart_CSend(UART_TempSend2);


		//------------------update BT1 status------------------------//
		if( (buttonSttBuff[0]==1) )//&& (button1_Stt == Button_None) )
		{
			button1_Stt = Button_Press;
			Button1_Keep_Flag =1;
		}
		else if(buttonSttBuff[0] == 0)
		{
			button1_Stt = Button_None;
			Button1_Keep_Flag =0;
		}
		//------------------update BT2 status------------------------//
		if( (buttonSttBuff[1]==1) )//&& (button2_Stt == Button_None) )
		{
			button2_Stt = Button_Press;
			Button2_Keep_Flag = 1;
		}
		else if(buttonSttBuff[1] == 0)
		{
			button2_Stt = Button_None;
			Button2_Keep_Flag = 0;
		}
		//------------------update BT3 status------------------------//
		if( (buttonSttBuff[2]==1) )//&& (button3_Stt == Button_None) )
		{
			button3_Stt = Button_Press;
			Button3_Keep_Flag = 1;
		}
		else if(buttonSttBuff[2] == 0)
		{
			button3_Stt = Button_None;
			Button3_Keep_Flag = 0;
		}
		//------------------update BT4 status------------------------//
		if( (buttonSttBuff[3]==1))// && (button4_Stt == Button_None) )
		{
			button4_Stt = Button_Press;
			Button4_Keep_Flag = 1;
		}
		else if(buttonSttBuff[3] == 0)
		{
			button4_Stt = Button_None;
			Button4_Keep_Flag = 0;
		}

	}

	  if((button1_Stt == Button_Press) && (button2_Stt == Button_None) && (button3_Stt == Button_None))
	  {
		  Control_Stt_Val = Control_Off;
	  }
	  else if((button1_Stt == Button_None) && (button2_Stt == Button_None) && (button3_Stt == Button_Press))
	  {
		  Control_Stt_Val = Control_On;
	  }
	  else if((button1_Stt == Button_None) && (button2_Stt == Button_Press) && (button3_Stt == Button_None))
	  {
		  Control_Stt_Val = Control_Stop;
	  }
//	  else if((buttonSttBuff[0]+ buttonSttBuff[1]+ buttonSttBuff[2]) >= 2)
//	  {
//		  Control_Stt_Val = Control_Stop;
//	  }
	  RD_SwitchAC4Ch_CheckHold( );
	/*------------------------------- Check Hold Button-------------------------------*/

}

void RD_SwitchAC4Ch_ScanB_V2(void)
{
	static uint8_t Bt1_Count_Buff =0;
	static uint8_t Bt2_Count_Buff =0;
	static uint8_t Bt3_Count_Buff =0;
//	static uint8_t Bt4_Count_Buff =0;


	static uint8_t buttonSttBuff[4] = {0};
/*------------------------------------------BT1----------------------------------------------------*/
			if(BT1_READ != 0)
			{
				Bt1_Count_Buff++;
				if(Bt1_Count_Buff == 3)
				{
					//uart_CSend("BT1 On \n");
					buttonSttBuff[0] =1;
				}
				if(Bt1_Count_Buff>=4)
				{
					Bt1_Count_Buff=4;
				}
			}
			else
			{
					Bt1_Count_Buff=0;
					buttonSttBuff[0] =0;
			}

			//------------------update BT1 status------------------------//
			if( (buttonSttBuff[0]==1) && (button1_Stt == Button_None) )
			{
				uart_CSend("Bt 1 Press \n");
				button1_Stt = Button_Press;
				Button1_Keep_Flag =1;
			}
			else if(buttonSttBuff[0] == 0)
			{
				if(button1_Stt == Button_Keeping  )
				{
					uart_CSend("Reset bt Stt \n");
				}
				button1_Stt = Button_None;
				Button1_Keep_Flag =0;
			}

/*------------------------------------------BT2----------------------------------------------------*/
			if(BT2_READ != 0)
			{
				Bt2_Count_Buff++;
				if(Bt2_Count_Buff == 3)
				{
					//uart_CSend("BT2 On \n");
					buttonSttBuff[1] =1;
				}
				if(Bt2_Count_Buff>=4)
				{
					Bt2_Count_Buff=4;
				}
			}
			else
			{
					Bt2_Count_Buff=0;
					buttonSttBuff[1] =0;
			}

			//------------------update BT2 status------------------------//
			if( (buttonSttBuff[1]==1) && (button2_Stt == Button_None) )
			{
				uart_CSend("Bt 2 Press \n");
				button2_Stt = Button_Press;
				Button2_Keep_Flag =1;
			}
			else if(buttonSttBuff[1] == 0)
			{
				if(button2_Stt == Button_Keeping  )
				{
					uart_CSend("Reset bt Stt \n");
				}
				button2_Stt = Button_None;
				Button2_Keep_Flag =0;
			}
/*------------------------------------------BT3----------------------------------------------------*/
			if(BT3_READ != 0)
			{
				Bt3_Count_Buff++;
				if(Bt3_Count_Buff == 3)
				{
					//uart_CSend("BT3 On \n");
					buttonSttBuff[2] =1;
				}
				if(Bt3_Count_Buff>=4)
				{
					Bt3_Count_Buff=4;
				}
			}
			else
			{
					Bt3_Count_Buff=0;
					buttonSttBuff[2] =0;
			}

			//------------------update BT3 status------------------------//
			if( (buttonSttBuff[2]==1) && (button3_Stt == Button_None) )
			{
				uart_CSend("Bt 3 Press \n");
				button3_Stt = Button_Press;
				Button3_Keep_Flag =1;
			}
			else if(buttonSttBuff[2] == 0)
			{
				if(button3_Stt == Button_Keeping  )
				{
					uart_CSend("Reset bt Stt \n");
				}
				button3_Stt = Button_None;
				Button3_Keep_Flag =0;
			}

  if((button1_Stt == Button_Press) && (button2_Stt == Button_None) && (button3_Stt == Button_None))
  {
	  Control_Stt_Val = Control_Off;
  }
  else if((button1_Stt == Button_None) && (button2_Stt == Button_None) && (button3_Stt == Button_Press))
  {
	  Control_Stt_Val = Control_On;
  }
  else if((button1_Stt == Button_None) && (button2_Stt == Button_Press) && (button3_Stt == Button_None))
  {
	  Control_Stt_Val = Control_Stop;
  }

 RD_SwitchAC4Ch_CheckHold( );
	/*------------------------------- Check Hold Button-------------------------------*/
}

void RD_SwitchAC4Ch_CheckHold(void)
{
	static uint16_t Count_BuffBT1, Count_BuffBT2, Count_BuffBT3; //, Count_BuffBT4;
	static bool4 Flag_Print1, Flag_Print2, Flag_Print3 =0; //, Flag_Print4=0;
//--------------------BUTTON 1 --------------------//
	if(button1_Stt == Button_Press)
	{
		Count_BuffBT1++;
		if(Count_BuffBT1>= 50000) Count_BuffBT1=50000;
		if(Count_BuffBT1 >= TIME_HOLD_BT)
		{

			if(Flag_Print1 == 0)
			{
				Button1_Hold_Flag =1;
				Flag_Print1=1;
			    uart_CSend("BT1 Hold\n");
			}
		}
	}
	else {
		Count_BuffBT1=0;
		Flag_Print1=0;
	}
//--------------------BUTTON 2 --------------------//
	if(button2_Stt == Button_Press)
	{
		Count_BuffBT2++;
		if(Count_BuffBT2>= 50000) Count_BuffBT2=50000;
		if(Count_BuffBT2 >= TIME_HOLD_BT)
		{

			if(Flag_Print2 == 0)
			{
				Button2_Hold_Flag =1;
				Flag_Print2=1;
			    uart_CSend("BT2 Hold\n");
			};
		}
	}
	else
		{
			Count_BuffBT2=0;
			Flag_Print2=0;
		}
	///////////////////BUTTON3
	if(button3_Stt == Button_Press)
	{
		Count_BuffBT3++;
		if(Count_BuffBT3>= 50000) Count_BuffBT3=50000;
		if(Count_BuffBT3 >= TIME_HOLD_BT)
		{

			if(Flag_Print3 == 0)
			{
				Button3_Hold_Flag =1;
				Flag_Print3=1;
			    uart_CSend("BT3 Hold\n");
			}
		}
	}
	else
		{
			Count_BuffBT3=0;
			Flag_Print3=0;
		}
	/////////////////BUTTON 4
/*	if(button4_Stt == Button_Keeping)
	{
		Count_BuffBT4++;
		if(Count_BuffBT4>= 50000) Count_BuffBT4=50000;
		if(Count_BuffBT4 >= TIME_HOLD_BT)
		{

			if(Flag_Print4 == 0)
			{
				Button4_Hold_Flag =1;
				Flag_Print4=1;
			    uart_CSend("BT4 Hold\n");
			}
		}
	}
	else
		{
			Count_BuffBT4=0;
			Flag_Print4=0;
		}*/
}

void RD_SwitchAC4Ch_UpdateCtr(void)
{
	/*--------------------update Relay Stt------------------	 */
	if(button1_Stt == Button_Press)
	{
		button1_Stt = Button_Keeping;
		relay1_Stt =  !relay1_Stt;
		//Send_Relay_Stt_Message();
	}

	if(button2_Stt == Button_Press)
	{
		button2_Stt = Button_Keeping;
		relay2_Stt =  !relay2_Stt;
		//Send_Relay_Stt_Message();
	}

	if(button3_Stt == Button_Press)
	{
		button3_Stt = Button_Keeping;
		relay3_Stt =  !relay3_Stt;
		//Send_Relay_Stt_Message();
	}

	if(button4_Stt == Button_Press)
	{
		button4_Stt = Button_Keeping;
		relay4_Stt =  !relay4_Stt;
		//Send_Relay_Stt_Message();
	}

	/*-------------------------Recontrol relay and led----------------*/
	static uint8_t time_CountBuff=0;
	static  Relay_Stt_Type Relay1LastStt_Buff = Relay_Off;
	static  Relay_Stt_Type Relay2LastStt_Buff = Relay_Off;
	static  Relay_Stt_Type Relay3LastStt_Buff = Relay_Off;
	static  Relay_Stt_Type Relay4LastStt_Buff = Relay_Off;
	time_CountBuff++;
	if(time_CountBuff>=50)
	{
		time_CountBuff=0;

#if(TYPE_HARDWARE_SWITCH == X4_TOUCH_SWITCH)
			if(Relay1LastStt_Buff != relay1_Stt)								// relay 1
			{
				if(relay1_Stt  ==  Relay_Off)
				{
					RD_SwitchAC4Ch_OnOffZero(RELAY1_PIN, LED1_PIN, Relay_Off);
				}
				else
				{
					RD_SwitchAC4Ch_OnOffZero(RELAY1_PIN, LED1_PIN, Relay_On);
				}

				Relay1LastStt_Buff = relay1_Stt;
				RD_Flash_SavePowerOff();
			}

			if(Relay2LastStt_Buff != relay2_Stt)								// relay 2
			{
				if(relay2_Stt  ==  Relay_Off)
				{
					RD_SwitchAC4Ch_OnOffZero(RELAY2_PIN, LED2_PIN, Relay_Off);
				}
				else
				{
					RD_SwitchAC4Ch_OnOffZero(RELAY2_PIN, LED2_PIN, Relay_On);
				}

				Relay2LastStt_Buff = relay2_Stt;
				RD_Flash_SavePowerOff();
			}

			if(Relay3LastStt_Buff != relay3_Stt)								// relay 3
			{
				if(relay3_Stt  ==  Relay_Off)
				{
					RD_SwitchAC4Ch_OnOffZero(RELAY3_PIN, LED3_PIN, Relay_Off);
				}
				else
				{
					RD_SwitchAC4Ch_OnOffZero(RELAY3_PIN, LED3_PIN, Relay_On);
				}

				Relay3LastStt_Buff = relay3_Stt;
				RD_Flash_SavePowerOff();
			}

			if(Relay4LastStt_Buff != relay4_Stt)								// relay 4
			{
				if(relay4_Stt  ==  Relay_Off)
				{
					RD_SwitchAC4Ch_OnOffZero(RELAY4_PIN, LED4_PIN, Relay_Off);
				}
				else
				{
					RD_SwitchAC4Ch_OnOffZero(RELAY4_PIN, LED4_PIN, Relay_On);
				}
				Relay4LastStt_Buff = relay4_Stt;
				RD_Flash_SavePowerOff();
			}
#endif

#if(TYPE_HARDWARE_SWITCH == X1_TOUCH_SWITCH)
			if(Relay1LastStt_Buff != relay1_Stt)								// relay 1
			{
				if(relay1_Stt  ==  Relay_Off)
				{
					RD_SwitchAC4Ch_OnOffZero(RELAY1_PIN, LED1_PIN, Relay_Off);
					gpio_write(LED2_PIN, Relay_On);
				}
				else
				{
					RD_SwitchAC4Ch_OnOffZero(RELAY1_PIN, LED1_PIN, Relay_On);
					gpio_write(LED2_PIN, Relay_Off);
				}

				Relay1LastStt_Buff = relay1_Stt;
				RD_Flash_SavePowerOff();
			}
#endif
//		Relay1LastStt_Buff = relay1_Stt;
//		Relay2LastStt_Buff = relay2_Stt;
//		Relay3LastStt_Buff = relay3_Stt;
//		Relay4LastStt_Buff = relay4_Stt;

	}

}

void RD_SwitchAC4Ch_OnOffZero(GPIO_PinTypeDef RelayPin, GPIO_PinTypeDef LedPin, Relay_Stt_Type OnOff_State)
{
	uint16_t overTimeBuff =0;
	#if (METHOD_DETECT_ZERO == DETECT_ZERO_BY_ADC)
		while((adc_sample_and_get_result() >= 2600) || (adc_sample_and_get_result() <= 2200) )
		{
			overTimeBuff ++;
			if(overTimeBuff>=20)
			{
				break;
			}

		}
		sleep_ms(4);
	#endif
	#if (METHOD_DETECT_ZERO == DETECT_ZERO_BY_FAILING)//DETECT_ZERO_BY_FAILING)
		uint8_t detectRezoState_Buff, detectRezoState_Past_Buff, detectRezoState_Unnoise_Buff;
		detectRezoState_Unnoise_Buff = detectRezoState_Past_Buff = detectRezoState_Buff = gpio_read(DETECT_ZERO_PIN);
		do
		{
			overTimeBuff ++;
			detectRezoState_Past_Buff = gpio_read(DETECT_ZERO_PIN);
			sleep_us(300);
			detectRezoState_Buff = gpio_read(DETECT_ZERO_PIN);
			sleep_us(10);
			detectRezoState_Unnoise_Buff = gpio_read(DETECT_ZERO_PIN);
			if(overTimeBuff>=1000)  // edit 6000-> 1000 20/05   not test yet
			{
				uart_CSend("break \n");
				break;
			}
		}
		while( !( (detectRezoState_Past_Buff != 0) && (detectRezoState_Buff == 0) && (detectRezoState_Unnoise_Buff == 0) ) );
		sleep_us(4500);
	#endif

	gpio_write(RelayPin, OnOff_State);
	gpio_write(LedPin, OnOff_State);
}

void RD_Socket_WaitDetechZero(void)
	{
	uint16_t overTimeBuff =0;
	uint8_t detectRezoState_Buff, detectRezoState_Past_Buff, detectRezoState_Unnoise_Buff,  detectRezoState_Unnoise_Buff2;
	detectRezoState_Unnoise_Buff = detectRezoState_Past_Buff = detectRezoState_Buff = gpio_read(DETECT_ZERO_PIN);
	do
	{
		overTimeBuff ++;
		detectRezoState_Past_Buff = gpio_read(DETECT_ZERO_PIN);
		sleep_us(50);
		detectRezoState_Buff = gpio_read(DETECT_ZERO_PIN);

		sleep_us(500);

		detectRezoState_Unnoise_Buff = gpio_read(DETECT_ZERO_PIN);
		sleep_us(50);
		detectRezoState_Unnoise_Buff2 = gpio_read(DETECT_ZERO_PIN);
		if(overTimeBuff>=NUM_CHECK_DETECH_MAX)
		{

			uart_CSend("break \n");
			break;
		}
	}
	while( !( (detectRezoState_Past_Buff != 0) && (detectRezoState_Buff != 0) && (detectRezoState_Unnoise_Buff == 0) && (detectRezoState_Unnoise_Buff2 ==0)) );

	}

void RD_SwitchAC4Ch_ScanReset(void)
{
	/*--------------------------Factory Reset------------------------*/
//	static uint16_t checkTimeBuff=0;
//	checkTimeBuff++;
//	if(checkTimeBuff >= TIME_OUT_RESET)
//	{
//		checkTimeBuff=0;
//		if(pressResetCount != 5)
//		{
//			pressResetCount=0;
//		}
//
//	}
//	if(pressResetCount == 5)
//	{
//		for(int i=0; i<5; i ++)
//		{
//			#if(TYPE_HARDWARE_SWITCH == X4_TOUCH_SWITCH)
//				gpio_write(LED1_PIN, LED_RED_ON);
//				gpio_write(LED2_PIN, LED_RED_ON);
//				gpio_write(LED3_PIN, LED_RED_ON);
//				gpio_write(LED4_PIN, LED_RED_ON);
//				sleep_ms(200);
//				gpio_write(LED1_PIN, LED_GREEN_ON);
//				gpio_write(LED2_PIN, LED_GREEN_ON);
//				gpio_write(LED3_PIN, LED_GREEN_ON);
//				gpio_write(LED4_PIN, LED_GREEN_ON);
//				sleep_ms(200);
//			#endif
//
//			#if(TYPE_HARDWARE_SWITCH == X1_TOUCH_SWITCH)
//				gpio_write(LED1_PIN, LED_RED_ON);
//				gpio_write(LED2_PIN, LED_GREEN_ON);
//				sleep_ms(200);
//				gpio_write(LED1_PIN, LED_GREEN_ON);
//				gpio_write(LED2_PIN, LED_RED_ON);
//				sleep_ms(200);
//			#endif
//		}
//		RD_Flash_CleanSenceFlash();
//		sleep_ms(200);
//		factory_reset();
//		start_reboot();
//	}
	static uint16_t checkTimeBuff2=0;
	static uint8_t  Reset_Count_Buff=0;
	#if(TYPE_HARDWARE_SWITCH == CURTAINS_TOUCH_SWITCH)
		if((Button1_Hold_Flag + Button2_Hold_Flag + Button3_Hold_Flag +Button4_Hold_Flag) == 2)
		{
			checkTimeBuff2=0;
			Reset_Count_Buff++;
			Button1_Hold_Flag=0;
			Button2_Hold_Flag=0;
			Button3_Hold_Flag=0;
			Button4_Hold_Flag=0;
			RD_SwitchAc4Ch_BlinkSet(3, 150);
		}
	#endif

	#if(TYPE_HARDWARE_SWITCH == X1_TOUCH_SWITCH)
		if( Button1_Hold_Flag == 1)
		{
			checkTimeBuff2=0;
			Reset_Count_Buff++;
			Button1_Hold_Flag=0;
			Button2_Hold_Flag=0;
			Button3_Hold_Flag=0;
			Button4_Hold_Flag=0;
			RD_SwitchAc4Ch_BlinkSet(3, 150);
		}
	#endif

		else
		{
			checkTimeBuff2++;
			if(checkTimeBuff2 >= TIME_OUT_RESET)
			{
				checkTimeBuff2=TIME_OUT_RESET;
				Reset_Count_Buff=0;
			}
		}

	if(Reset_Count_Buff == 3)
	{
		for(int i=0; i<5; i ++)
		{
			#if(TYPE_HARDWARE_SWITCH == X4_TOUCH_SWITCH)
				gpio_write(LED1_PIN, LED_RED_ON);
				gpio_write(LED2_PIN, LED_RED_ON);
				gpio_write(LED3_PIN, LED_RED_ON);
				gpio_write(LED4_PIN, LED_RED_ON);
				sleep_ms(200);
				gpio_write(LED1_PIN, LED_GREEN_ON);
				gpio_write(LED2_PIN, LED_GREEN_ON);
				gpio_write(LED3_PIN, LED_GREEN_ON);
				gpio_write(LED4_PIN, LED_GREEN_ON);
				sleep_ms(200);
				wd_clear(); //clear watch dogsdfasdf
				gpio_write(LED1_PIN, LED_RED_ON);
				gpio_write(LED2_PIN, LED_RED_ON);
				gpio_write(LED3_PIN, LED_RED_ON);
				gpio_write(LED4_PIN, LED_RED_ON);
				sleep_ms(200);
				gpio_write(LED1_PIN, LED_GREEN_ON);
				gpio_write(LED2_PIN, LED_GREEN_ON);
				gpio_write(LED3_PIN, LED_GREEN_ON);
				gpio_write(LED4_PIN, LED_GREEN_ON);
				sleep_ms(200);

			#endif

			#if(TYPE_HARDWARE_SWITCH == X1_TOUCH_SWITCH)
				gpio_write(LED1_PIN, LED_RED_ON);
				gpio_write(LED2_PIN, LED_GREEN_ON);
				sleep_ms(200);
				gpio_write(LED1_PIN, LED_GREEN_ON);
				gpio_write(LED2_PIN, LED_RED_ON);
				sleep_ms(200);
			#endif
		}
		RD_Flash_CleanSenceFlash();
		uart_CSend("\nFactory Reset\n");
		sleep_ms(200);

		factory_reset();
		start_reboot();
	}




	/*---------------------------Keep touch button Reset-------------------*/
/*	static uint16_t timeKeepButton1 =0;
	static uint16_t timeKeepButton2 =0;
	static uint16_t timeKeepButton3 =0;
	static uint16_t timeKeepButton4 =0;
	if(Button1_Keep_Flag == 1)
	{
		timeKeepButton1++;
		if(timeKeepButton1 >= 50000)
			timeKeepButton1=50000;
	}
	else
	{
		timeKeepButton1=0;
	}

	if(Button2_Keep_Flag == 1)
	{
		timeKeepButton2++;
		if(timeKeepButton2 >= 50000)
			timeKeepButton2=50000;
	}
	else
	{
		timeKeepButton2=0;
	}

	if(Button3_Keep_Flag == 1)
	{
		timeKeepButton3++;
		if(timeKeepButton3 >= 50000)
			timeKeepButton3=50000;
	}
	else
	{
		timeKeepButton3=0;
	}

	if(Button4_Keep_Flag == 1)
	{
		timeKeepButton4++;
		if(timeKeepButton4 >= 50000)
			timeKeepButton4=50000;
	}
	else
	{
		timeKeepButton4=0;
	}
	*/
}

void RD_SwitchAc4Ch_BlinkScan(void)
{
	static uint16_t TimeBlink_Buff=0;
	static uint8_t  provision_blink=0;
	if((get_provision_state() == STATE_DEV_PROVING) && (provision_blink == 0))
	{
		provision_blink=1;
		RD_SwitchAc4Ch_BlinkSet(50, 100);
	}
	if((get_provision_state() == STATE_DEV_PROVED )&& (provision_blink == 1) )
	{
		provision_blink=0;
		RD_SwitchAc4Ch_BlinkSet(10, 100);
	}
	if(CycleBlink >0)
	{
		TimeBlink_Buff++;

		if(TimeBlink_Buff>= CountToggleBlink)
		{
			TimeBlink_Buff=0;
			if(CycleBlink >=1)
			{
				if(CycleBlink%2 == 0)
				{
					#if(TYPE_HARDWARE_SWITCH == X4_TOUCH_SWITCH)
						gpio_write(LED1_PIN, LED_RED_ON);
						gpio_write(LED2_PIN, LED_RED_ON);
						gpio_write(LED3_PIN, LED_RED_ON);
					//	gpio_write(LED4_PIN, LED_RED_ON);
					#endif

					#if(TYPE_HARDWARE_SWITCH == X1_TOUCH_SWITCH)
						gpio_write(LED1_PIN, LED_RED_ON);
						gpio_write(LED2_PIN, LED_GREEN_ON);
					#endif
				}
				else
				{
					#if(TYPE_HARDWARE_SWITCH == X4_TOUCH_SWITCH)
						gpio_write(LED1_PIN, LED_GREEN_ON);
						gpio_write(LED2_PIN, LED_GREEN_ON);
						gpio_write(LED3_PIN, LED_GREEN_ON);
					//	gpio_write(LED4_PIN, LED_GREEN_ON);
					#endif

					#if(TYPE_HARDWARE_SWITCH == X1_TOUCH_SWITCH)
						gpio_write(LED1_PIN, LED_GREEN_ON);
						gpio_write(LED2_PIN, LED_RED_ON);
					#endif
				}
			}

			if(CycleBlink == 1)								// reset led. set as relay stt
			{
				#if(TYPE_HARDWARE_SWITCH == X4_TOUCH_SWITCH)
					if(Control_Stt_Val == Control_Off )
					{
							gpio_write(CLOSE_LED, LED_RED_ON);
							gpio_write(STOP_LED, LED_GREEN_ON);
							gpio_write(OPEN_LED, LED_GREEN_ON);
					}
					if(Control_Stt_Val == Control_On)
					{
							gpio_write(CLOSE_LED, LED_GREEN_ON);
							gpio_write(STOP_LED, LED_GREEN_ON);
							gpio_write(OPEN_LED, LED_RED_ON);

					}
					if(Control_Stt_Val == Control_Stop)
					{
							gpio_write(CLOSE_LED, LED_GREEN_ON);
							gpio_write(STOP_LED, LED_RED_ON);
							gpio_write(OPEN_LED, LED_GREEN_ON);
					}
				#endif

				#if(TYPE_HARDWARE_SWITCH == X1_TOUCH_SWITCH)
					gpio_write(LED1_PIN, relay1_Stt);
					gpio_write(LED2_PIN, !relay1_Stt);
				#endif
			}

			CycleBlink--;
		}
	}


}

void RD_SwitchAc4Ch_BlinkSet(uint8_t cycle, uint16_t countToggle)
{
	CycleBlink		 = cycle;
	CountToggleBlink = countToggle;
}

void RD_SwitchAc4Ch_CheckPower(void)
{
	if(adc_sample_and_get_result() <= 1000)		// Low voltage
	{
		countPower++;
	}
	else										// high voltage
	{
		countPower = 0;
	}
}

void RD_SwitchAc4Ch_SaveDataPowerOffLoop(void)
{
	if((countPower >=COUNT_DETECT_POWER_OFF) && (powerSaved == 0))
	{
		powerSaved=1;
//		gpio_write(RELAY1_PIN, Relay_Off);					// turn off relay to save power on capacitor
//		gpio_write(RELAY2_PIN, Relay_Off);
//		gpio_write(RELAY3_PIN, Relay_Off);
//		gpio_write(RELAY4_PIN, Relay_Off);

		RD_Flash_SavePowerOff( );
//		gpio_write(RELAY1_PIN, relay1_Stt);					// turn off relay to save power on capacitor
//		gpio_write(RELAY2_PIN, relay2_Stt);
//		gpio_write(RELAY3_PIN, relay3_Stt);
//		gpio_write(RELAY4_PIN, relay4_Stt);

	}
}

void RD_SwitchAc4Ch_UpdateDataConfigCy8(void)
{

	Register_Config[ SENSOR_EN_L_REG_CY8] 		= SENSOR_EN_L;
	Register_Config[ SENSOR_EN_H_REG_CY8] 		= SENSOR_EN_H;
	Register_Config[ FSS_EN_L_REG_CY8] 			= FSS_EN_L;
	Register_Config[ FSS_EN_H_REG_CY8] 			= FSS_EN_H;
	Register_Config[ TOGGLE_EN_L_REG_CY8] 		= TOGGLE_EN_L;
	Register_Config[ TOGGLE_EN_H_REG_CY8] 		= TOGGLE_EN_H;
	Register_Config[ LED_ON_EN_L_REG_CY8] 		= LED_ON_EN_L;
	Register_Config[ LED_ON_EN_H_REG_CY8] 		= LED_ON_EN_H;
	Register_Config[ SENSITIVITY0_REG_CY8] 		= SENSITIVITY0;
	Register_Config[ SENSITIVITY1_REG_CY8]	 	= SENSITIVITY1;
	Register_Config[ SENSITIVITY2_REG_CY8] 		= SENSITIVITY2;
	Register_Config[ BASE_THRESHOLD0_REG_CY8] 	= BASE_THRESHOLD0;
	Register_Config[ BASE_THRESHOLD1_REG_CY8] 	= BASE_THRESHOLD1;
	Register_Config[ FINGER_THRESHOLD2_REG_CY8] = FINGER_THRESHOLD2;
	Register_Config[ FINGER_THRESHOLD3_REG_CY8] = FINGER_THRESHOLD3;
	Register_Config[ FINGER_THRESHOLD4_REG_CY8] = FINGER_THRESHOLD4;
	Register_Config[ FINGER_THRESHOLD5_REG_CY8] = FINGER_THRESHOLD5;
	Register_Config[ FINGER_THRESHOLD6_REG_CY8] = FINGER_THRESHOLD6;
	Register_Config[ FINGER_THRESHOLD7_REG_CY8] = FINGER_THRESHOLD7;
	Register_Config[ FINGER_THRESHOLD8_REG_CY8] = FINGER_THRESHOLD8;
	Register_Config[ FINGER_THRESHOLD9_REG_CY8] = FINGER_THRESHOLD9;
	Register_Config[ SENSOR_DEBOUNCE_REG_CY8] 	= SENSOR_DEBOUNCE;
	Register_Config[ BUTTON_HYS_REG_CY8] 		= BUTTON_HYS;
	Register_Config[ BUTTON_LBR_REG_CY8] 		= BUTTON_LBR;
	Register_Config[ BUTTON_NNT_REG_CY8	] 		= BUTTON_NNT;
	Register_Config[ BUTTON_NT_REG_CY8] 		= BUTTON_NT;
	Register_Config[ PROX_EN_REG_CY8] 			= PROX_EN;
	Register_Config[ PROX_CFG_REG_CY8 ] 		= PROX_CFG;
	Register_Config[ PROX_CFG2_REG_CY8] 		= PROX_CFG2;
	Register_Config[ PROX_TOUCH_TH0_L_REG_CY8] 	= PROX_TOUCH_TH0_L;
	Register_Config[ PROX_TOUCH_TH0_H_REG_CY8]	= PROX_TOUCH_TH0_H;
	Register_Config[ PROX_TOUCH_TH1_L_REG_CY8] 	= PROX_TOUCH_TH1_L;
	Register_Config[ PROX_TOUCH_TH1_H_REG_CY8] 	= PROX_TOUCH_TH1_H;
	Register_Config[ PROX_RESOLUTION0_REG_CY8] 	= PROX_RESOLUTION0;
	Register_Config[ PROX_RESOLUTION1_REG_CY8] 	= PROX_RESOLUTION1;
	Register_Config[ PROX_HYS_REG_CY8] 			= PROX_HYS;
	Register_Config[ PROX_LBR_REG_CY8] 			= PROX_LBR;
	Register_Config[ PROX_NNT_REG_CY8]		 	= PROX_NNT;
	Register_Config[ PROX_NT_REG_CY8] 			= PROX_NT;
	Register_Config[ PROX_POSITIVE_TH0_REG_CY8] = PROX_POSITIVE_TH0;
	Register_Config[ PROX_POSITIVE_TH1_REG_CY8] = PROX_POSITIVE_TH1;
	Register_Config[ PROX_NEGATIVE_TH0_REG_CY8] = PROX_NEGATIVE_TH0;
	Register_Config[ PROX_NEGATIVE_TH1_REG_CY8] = PROX_NEGATIVE_TH1;
	Register_Config[ LED_ON_TIME_REG_CY8] 		= LED_ON_TIME;
	Register_Config[ BUZZER_CFG_REG_CY8	] 		= BUZZER_CFG;
	Register_Config[ BUZZER_ON_TIME_REG_CY8	] 	= BUZZER_ON_TIME;
	Register_Config[ GPO_CFG_REG_CY8] 			= GPO_CFG;
	Register_Config[PWM_DUTYCYCLE_CFG0_REG_CY8] = PWM_DUTYCYCLE_CFG0;
	Register_Config[PWM_DUTYCYCLE_CFG1_REG_CY8] = PWM_DUTYCYCLE_CFG1;
	Register_Config[PWM_DUTYCYCLE_CFG2_REG_CY8] = PWM_DUTYCYCLE_CFG2;
	Register_Config[PWM_DUTYCYCLE_CFG3_REG_CY8] = PWM_DUTYCYCLE_CFG3;
	Register_Config[PWM_DUTYCYCLE_CFG4_REG_CY8] = PWM_DUTYCYCLE_CFG4;
	Register_Config[SPO_CFG_REG_CY8] 			= SPO_CFG;
	Register_Config[ DEVICE_CFG0_REG_CY8] 		= DEVICE_CFG0;
	Register_Config[ DEVICE_CFG1_REG_CY8] 		= DEVICE_CFG1;
	Register_Config[ DEVICE_CFG2_REG_CY8] 		= DEVICE_CFG2;
	Register_Config[ DEVICE_CFG3_REG_CY8] 		= DEVICE_CFG3;
	Register_Config[ I2C_ADDR_REG_CY8] 			= I2C_ADDR;
	Register_Config[ REFRESH_CTRL_REG_CY8] 		= REFRESH_CTRL;
	Register_Config[ STATE_TIMEOUT_REG_CY8] 	= STATE_TIMEOUT;
	Register_Config[ SCRATCHPAD0_REG_CY8] 		= SCRATCHPAD0;
	Register_Config[ SCRATCHPAD1_REG_CY8] 		= SCRATCHPAD1;
	Register_Config[CONFIG_CRC_L_REG_CY8]       = 0x00;
	Register_Config[CONFIG_CRC_H_REG_CY8]       = 0x00;
}

uint8_t RD_SwitchAc4Ch_I2cWrite(unsigned int Add_Reg, unsigned int Add_Reg_Len, unsigned char *data, int dataLen)
{
	i2c_write_series(Add_Reg, Add_Reg_Len, data, dataLen);
	return(reg_i2c_status);
}

uint8_t RD_SwitchAc4Ch_I2cRead(unsigned int Add_Reg, unsigned int Add_Reg_Len, unsigned char *data, int dataLen )
{
	i2c_read_series(Add_Reg, Add_Reg_Len, data, dataLen );
	return (reg_i2c_status);
}

void RD_SwitchAc4Ch_CY8Init(void)
{

		uint8_t  ctrlCmdStatus_Buff 	= 0xff;		// buff check
		uint8_t	 ctrlCmdErr_Buff		= 0x00;
		uint8_t  cmdOpCode_Save_Buff	= 0x02;		// save data
		uint8_t  cmdOpCode_GetCRC_Buff	= 0x03;		// save data
		uint8_t	 cmdOpCode_Reset_Buff	= 255;
		uint8_t check_CALC_CRC[2]      ={0};

		RD_SwitchAc4Ch_UpdateDataConfigCy8();
		do
		{


			Register_Config[CONFIG_CRC_L_REG_CY8] =  0xb2;					// get value for CONFIG_CRC_L
			Register_Config[CONFIG_CRC_H_REG_CY8] =  0xe0;					// get value for CONFIG_CRC_H

/*--------------------------------------------------get CRC on CY8--------------------------------------------------------------*/ 	// config data
			while(RD_SwitchAc4Ch_I2cWrite(SENSOR_EN_L_REG_CY8, 1, Register_Config, SIZE_OF_DATA_CONFIG) != 0)				// config data
			{

			}
			sleep_ms(5);
			while(RD_SwitchAc4Ch_I2cWrite(CTRL_CMD_CY8_REG, 1, &cmdOpCode_GetCRC_Buff, 1) != 0);								// send cmd op code =2 to save
			sleep_ms(220);																										// delay foe Cy8 ic uadate config

			while(RD_SwitchAc4Ch_I2cRead(CALC_CRC_REG_CY8, 1, check_CALC_CRC, 2) != 0);
			Register_Config[CONFIG_CRC_L_REG_CY8] =  check_CALC_CRC[0];															// get value for CONFIG_CRC_L
			Register_Config[CONFIG_CRC_H_REG_CY8] =  check_CALC_CRC[1];															// get value for CONFIG_CRC_H
/*--------------------------------------------------set CRC and re config--------------------------------------------------------------*/

			while(RD_SwitchAc4Ch_I2cWrite(SENSOR_EN_L_REG_CY8, 1, Register_Config, SIZE_OF_DATA_CONFIG) != 0);				// config data
			sleep_ms(5);

			do
			{
				while(RD_SwitchAc4Ch_I2cWrite(CTRL_CMD_CY8_REG, 1, &cmdOpCode_Save_Buff, 1) != 0);								// send cmd op code =2 to save
				sleep_ms(220);																										// delay foe Cy8 ic uadate config

				while(RD_SwitchAc4Ch_I2cRead(CTRL_CMD_STATUS_CY8_REG, 1, &ctrlCmdStatus_Buff, 1) != 0);
				if(ctrlCmdStatus_Buff == 0)																							// config and save successfully
				{
					while(RD_SwitchAc4Ch_I2cWrite(CTRL_CMD_CY8_REG, 1, &cmdOpCode_Reset_Buff, 1) != 0);								// save and reset ic
					sleep_ms(50);
				}
				else
				{
					while(RD_SwitchAc4Ch_I2cRead(CTRL_CMD_ERR_CY8_REG, 1, &ctrlCmdErr_Buff, 1) != 0);
					sleep_ms(220);
				}

			}
			while((ctrlCmdErr_Buff == 253) || (ctrlCmdErr_Buff == 255));
		}
		while(ctrlCmdErr_Buff == 254);

}

uint16_t RD_Curtains_Delta_Time_Control(void)
{
	static Control_Stt_type Pre_Control_Stt = Control_Stop;
	static uint16_t Pre_Time=0, Time=0;
	uint16_t delta_time=0;
	if(Pre_Control_Stt != Control_Stt_Val)
	{
		Pre_Time = Time = clock_time_ms();
	}
	Time = clock_time_ms();
	delta_time = Time - Pre_Time;
	Pre_Time = Time;
	Pre_Control_Stt = Control_Stt_Val;
	return delta_time;

}

void RD_Curtains_Control(Control_Stt_type Control_Stt)
{
	uint8_t Open_Percent_Buff=0;
			if(Control_Stt == Control_Off )
			{

					Open_Percent_Buff = (Time_Control_State_Now*100)/Time_Control_Max ;
					Send_Relay_Stt_Message(Control_Off, Open_Percent_Buff);
					gpio_write(OPEN_RELAY, 0);
					gpio_write(COMMON_RELAY, 1);
					gpio_write(CLOSE_RELAY, 1);

					gpio_write(CLOSE_LED, LED_RED_ON);
					gpio_write(STOP_LED, LED_GREEN_ON);
					gpio_write(OPEN_LED, LED_GREEN_ON);

			}

			if(Control_Stt == Control_On)
			{

					Open_Percent_Buff = (Time_Control_State_Now*100)/Time_Control_Max ;
					Send_Relay_Stt_Message(Control_On, Open_Percent_Buff);
					gpio_write(CLOSE_RELAY, 0);
					gpio_write(COMMON_RELAY, 1);
					gpio_write(OPEN_RELAY, 1);

					gpio_write(CLOSE_LED, LED_GREEN_ON);
					gpio_write(STOP_LED, LED_GREEN_ON);

					gpio_write(OPEN_LED, LED_RED_ON);

			}
			if(Control_Stt == Control_Stop)
			{
					Open_Percent_Buff = (Time_Control_State_Now*100)/Time_Control_Max ;
					Send_Relay_Stt_Message(Control_Stop, Open_Percent_Buff);
					RD_Flash_SaveOpenStt(Open_Percent_Buff);
					gpio_write(CLOSE_RELAY, 1);
					gpio_write(COMMON_RELAY, 1);
					gpio_write(OPEN_RELAY, 1);

					gpio_write(CLOSE_LED, LED_GREEN_ON);
					gpio_write(STOP_LED, LED_RED_ON);
					gpio_write(OPEN_LED, LED_GREEN_ON);

			}

}

void RD_Curtanins_Update_Control(void)
{
	static Control_Stt_type Pre_Control_Stt_Taget_Val = Control_Stop;
	static Control_Stt_type Control_Stt_Taget_Val = Control_Stop;
	uint8_t Buff=0;

	static Control_Stt_type Pre_Control_Stt_Val = Control_Stop;
	static uint16_t Count_Delay =0;
	if(Control_Stt_Val != Control_Stop)
	{
		if(Control_Stt_Val == Control_Off && Pre_Control_Stt_Val == Control_On)
		{
			Control_Stt_Taget_Val = Control_Stop;
			Count_Delay++;
			if(Count_Delay >= TIME_DELAY_TOGGLE_CONTROL)
			{
				Count_Delay=0;
				Control_Stt_Taget_Val = Control_Off;
				Pre_Control_Stt_Val   = Control_Stop;
			}
		}
		else if(Control_Stt_Val == Control_On && Pre_Control_Stt_Val == Control_Off)
		{
			Control_Stt_Taget_Val = Control_Stop;
			Count_Delay++;
			if(Count_Delay >= TIME_DELAY_TOGGLE_CONTROL)
			{
				Count_Delay=0;
				Control_Stt_Taget_Val = Control_On;
				Pre_Control_Stt_Val   = Control_Stop;
			}
		}
		else
		{
			Count_Delay=0;
			Control_Stt_Taget_Val = Control_Stt_Val;
			Pre_Control_Stt_Val = Control_Stt_Taget_Val;
		}
	}
	else
	{
		Count_Delay=0;
		Control_Stt_Taget_Val = Control_Stt_Val;
		Pre_Control_Stt_Val =  Control_Stt_Val;
	}


		//---------------------------- main control ------------------------------//
	if( Control_Stt_Taget_Val == Control_Off)
	{
		if(Time_Control_State_Now >0)
		{
			if( Control_Stt_Taget_Val != Pre_Control_Stt_Taget_Val)
			{
				#if(  METHOD_DETECT_ZERO == DETECT_ZERO_BY_FAILING_V2)
					RD_Socket_WaitDetechZero();									// RD_EDIT Detech
					sleep_us(DELAY_DETECH_TIME);
				#endif
				RD_Curtains_Control(Control_Off);
				Buff = RD_Curtains_Delta_Time_Control();
				Pre_Control_Stt_Val = Control_Off;
			}
			Time_Control_State_Now = Time_Control_State_Now - RD_Curtains_Delta_Time_Control();
			if(Time_Control_State_Now <0)
			{
				Time_Control_State_Now=0;
			}

		}
		else
		{
			#if(  METHOD_DETECT_ZERO == DETECT_ZERO_BY_FAILING_V2)
				RD_Socket_WaitDetechZero();									// RD_EDIT Detech
				sleep_us(DELAY_DETECH_TIME);
			#endif
			Control_Stt_Val = Control_Stop;
			Control_Stt_Taget_Val = Control_Stop;
			RD_Curtains_Control(Control_Stop);
		}
	}

	if( Control_Stt_Taget_Val == Control_On)
	{
		if(Time_Control_State_Now <Time_Control_Max)
		{
			if( Control_Stt_Taget_Val != Pre_Control_Stt_Taget_Val)
			{
				#if(  METHOD_DETECT_ZERO == DETECT_ZERO_BY_FAILING_V2)
					RD_Socket_WaitDetechZero();									// RD_EDIT Detech
					sleep_us(DELAY_DETECH_TIME);
				#endif
				RD_Curtains_Control(Control_On);
				Buff = RD_Curtains_Delta_Time_Control();
				Pre_Control_Stt_Val = Control_On;
			}
			Time_Control_State_Now = Time_Control_State_Now + RD_Curtains_Delta_Time_Control();
			if(Time_Control_State_Now >Time_Control_Max)
			{
				Time_Control_State_Now=Time_Control_Max;
			}

		}
		else
		{
			#if(  METHOD_DETECT_ZERO == DETECT_ZERO_BY_FAILING_V2)
				RD_Socket_WaitDetechZero();									// RD_EDIT Detech
				sleep_us(DELAY_DETECH_TIME);
			#endif
			Control_Stt_Val = Control_Stop;
			Control_Stt_Taget_Val = Control_Stop;
			RD_Curtains_Control(Control_Stop);
		}
	}

	if( Control_Stt_Taget_Val == Control_Stop)
	{
		if( Control_Stt_Taget_Val != Pre_Control_Stt_Taget_Val)
		{
			#if(  METHOD_DETECT_ZERO == DETECT_ZERO_BY_FAILING_V2)
				RD_Socket_WaitDetechZero();									// RD_EDIT Detech
				sleep_us(DELAY_DETECH_TIME);
			#endif
			RD_Curtains_Control(Control_Stop);
			char UART_TempSend2[128];
			sprintf(UART_TempSend2,"Stt_Now: L= %d   ms \n",Time_Control_State_Now);
			uart_CSend(UART_TempSend2);
		}
	}

	Pre_Control_Stt_Taget_Val = Control_Stt_Taget_Val;

}

void RD_Curtains_ScanControlToPos(void)   // % Open.  100%-->
{
	uint8_t Pos_Now_Buff =0; // Position Open of Curtain.  unit %.   0-->100% Open
	Pos_Now_Buff = (Time_Control_State_Now*100) / Time_Control_Max;
	if(Pos_Flag == 1)
	{
		if(Pos_Now_Buff < (Position_Taget-1))
		{
			Control_Stt_Val = Control_On;
		}
		else
			if(Pos_Now_Buff > (Position_Taget+1))
			{
				Control_Stt_Val = Control_Off;
			}
			else{
				Control_Stt_Val = Control_Stop;
				Pos_Flag=0;
			}
	}
}

void RD_Curtains_Goto(uint8_t Position)
{
	if(Position>100) Position=100;
	if(Position<0)   Position=0;
	Pos_Flag =1; // turn On Flag control to Position
	Position_Taget = Position; // unit %
}

void RD_Curtain_Calib(void)
{
	if( calib_Stt  ==1)
	{
		Time_Control_State_Now = Time_Control_Max/2;

	}
}
