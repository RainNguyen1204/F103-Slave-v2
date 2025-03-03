/**
  ******************************************************************************
  * @file    	EncoderPossition.c
  * @author  	Nguyen Vu
	*	@version 	1.0.0
  * @brief   	This file provides function to calculate angle using IMU
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "IMU.h"

/**
  * @brief  Command which is provided by WIT Standard Communication Protocol
  */
static uint8_t unlock_cmd[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
static uint8_t reset_zero_cmd[5] = {0xFF, 0xAA, 0x01, 0x04, 0x00};
static uint8_t save_cmd[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00};

/**
  * @brief  Some variables for IMU
  */
static uint8_t buff[11];
static uint8_t recieve_flag = 0;
static uint8_t data_len = 0;
static uint8_t uart_flag = 0;
static uint8_t flag;
static uint8_t checksum;




/** @brief    IMU basic function for cauculating angle
  ==============================================================================
												##### IMU Basic Functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Data in handling.
    (+) Angle calculating.
  */

/**
  * @brief  IMU data in handling
	*	@param	data	Saving data in value
	*	@note		Place this function in UART interupt
  */
void IMU_Data_In(uint8_t data)
{
	//wait for IMU address
	if (data == 0x55 && data_len == 0)
		recieve_flag = 1;
	//start saving HEX value from IMU to data buff
	if (recieve_flag)
	{
		buff[data_len] = data;
		data_len++;
	}
	//Set flag after data buff is full
	if (data_len > 10)
		uart_flag = 1;
}


/**
  * @brief  IMU angle calculating
	*	@param	angle	Saving angle value
	*	@param	aData	Saving HEX value for data transmition or somethings
	*	@note		Place this function in while-loop
  */
void IMU_Data_Process(Angle_ReadTypeDef *angle, uint8_t aData[])
{
	  if (uart_flag)
	  {
		  //Checksum value
		  checksum = 0x55 + 0x53 + buff[2] + buff[3] + buff[4]
					+ buff[5] + buff[6] + buff[7] + buff[8] + buff[9];
			//Checking data content byte
		  if (buff[1] == 0x53 && checksum == buff[10])
		  {
				//Saving angle value
			  angle->x = ((float)((short)buff[3] << 8| buff[2])/32768.0)*180.0;
			  angle->y = ((float)((short)buff[5] << 8| buff[4])/32768.0)*180.0;
			  angle->z = ((float)((short)buff[7] << 8| buff[6])/32768.0)*180.0;
				//Saving HEX value
				aData[0] = buff[2];
				aData[1] = buff[3];
				aData[2] = buff[4];
				aData[3] = buff[5];
				aData[4] = buff[6];
				aData[5] = buff[7];
		  }
			//Reset flag
		  uart_flag = 0;
		  recieve_flag = 0;
		  data_len = 0;
	  }
}

/** @brief    IMU control funtion 
  ==============================================================================
											##### IMU Control Funtion #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
		(+) Reset IMU z angle value.
    (+) Reset IMU's reset flag for reseting IMU z angle value.
  */

/**
  * @brief  IMU z angle reset
	*	@param	huart	UART which is used for sending command to IMU
	*	@note		Place this function in while-loop
  */
void IMU_Reset_Zero(UART_HandleTypeDef *huart)
{
	static uint32_t time;
	//Delay 100ms before sending another command	
	if (HAL_GetTick() - time > 100 && flag != 3)
	{
		switch (flag)
		{
			case 0:
			  HAL_UART_Transmit(huart, unlock_cmd, sizeof(unlock_cmd),100);
			  flag++;
			  time = HAL_GetTick();
			  break;
		  case 1:
			  HAL_UART_Transmit(huart, reset_zero_cmd, sizeof(reset_zero_cmd),100);
			  flag++;
			  time = HAL_GetTick();
			  break;
		  case 2:
			  HAL_UART_Transmit(huart, save_cmd, sizeof(save_cmd), 100);
			  flag++;
			  time = HAL_GetTick();
		}
	}
}

/**
  * @brief  Resetting reset flag
	*	@note		Call this funtion to run WIT_Reset_Zero again
  */
void IMU_Reset_Flag(void)
{
	flag = 0;
}








