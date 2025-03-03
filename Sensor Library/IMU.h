/**
  ******************************************************************************
  * @file    	IMU.h
  * @author  	Nguyen Vu
  * @brief   	This file contains all the functions prototypes 
	*						for the IMU driver
  *****************************************************************************/
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef IMU_H_
#define IMU_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

//IMU angle Struct
typedef struct
{
	float x;
	float y;
	float z;
}Angle_ReadTypeDef;

/* Basic handling functions  **************************************************/
void IMU_Data_In(uint8_t data);
void IMU_Data_Process(Angle_ReadTypeDef *angle, uint8_t aData[]);

/* IMU controlling functions  *************************************************/
void IMU_Reset_Zero(UART_HandleTypeDef *huart);
void IMU_Reset_Flag(void);

#endif







