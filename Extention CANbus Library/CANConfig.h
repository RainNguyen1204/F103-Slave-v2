/**
  ******************************************************************************
  * @file    	bxCANlib.h
  * @author  	Nguyen Vu
  * @brief   	This file contains all configuration value 
							for others CANbus library
  *****************************************************************************/
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CAN_CONFIG_H_
#define CAN_CONFIG_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/**
  * @brief  Configuration Queue Size
  */
#define CAN_QUEUE_CAPACITY	4	

/**
  * @brief  Configuration Sensor ID
  */
#define IMU_ID 					0x00
#define ENC_ID					0x01

/**
  * @brief  Configuration Sensor Data Address/ID
  */
#define IMU_DATA				0x0A
#define ENC_DATA				0x0A

/**
  * @brief  Configuration Sensor Data DLC
  */
#define IMU_DATA_DLC 		0x06
#define ENC_DATA_DLC		0x08

/**
  * @brief  Configuration Command ID for Master
  */
#define START_ID				0x00
#define RESET_ID				0x01
#define	STOP_ID					0x02
#define ENC_ASSIGN_ID 	0x03

/**
  * @brief  Configuration Command DLC for Master
  */
#define START_DLC				0x02
#define RESET_DLC 			0x00
#define	STOP_DLC				0x00
#define ENC_ASSIGN_DLC	0x08

/**
  * @brief  Configuration Feedback ID for Slave
  */
#define START_FB_ID 		0x00
#define RESET_FB_ID			0x01
#define STOP_FB_ID			0x02
#define ASSIGN_FB_ID		0x03

/**
  * @brief  Configuration Feedback DLC for Slave
  */
#define START_FB_DLC		0x02
#define RESET_FB_DLC		0x00
#define	STOP_FB_DLC			0x00
#define ASSIGN_FB_DLC		0x08

/**
  * @brief  Configuration Error ID for Slave
  */
#define ERROR_ID				0x0B
#define ERROR_DLC				0x00


#endif









