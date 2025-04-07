/**
  ******************************************************************************
  * @file    	CANSlavelib.h
  * @author  	Nguyen Vu
  * @brief   	This file contains all the functions prototypes 
	*						for the Slave device using CANbus protocol
  *****************************************************************************/
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CANSLAVELIB_H_
#define CANSLAVELIB_H_

/* Includes ------------------------------------------------------------------*/
#include "bxCANlib.h"
#include "CANConfig.h"
#include "EncoderPosition.h"
#include "IMU.h"

/**
  * @brief  TxMessage struct
	* @param	sensor_it	Sensor ID
	* @param	freq			Frequency
  */
typedef struct
{
	uint32_t	sensor_id;
	uint16_t	freq;
	uint8_t		start_flag;
	uint8_t		stop_flag;
}Sensor_HandleTypedef;

/* Initialization functions  **************************************************/
void CAN_Sensor_Init(Sensor_HandleTypedef *Sensor, uint32_t sensor_id);

/* Receiving functions through CAN protocol  **********************************/
void CAN_Slave_FIFO0_RxMessage(CAN_HandleTypeDef *hcan);
void CAN_Slave_FIFO0_Recieve_Cmd_Handle(CAN_HandleTypeDef *hcan);

/* Sensor control functions through CAN protocol  *****************************/
void CAN_Start_Encoder(Sensor_HandleTypedef *Sensor, TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2);
void CAN_Start_IMU(Sensor_HandleTypedef *Sensor ,UART_HandleTypeDef *huart, uint8_t *rxdata);

void CAN_Reset_Encoder(CAN_HandleTypeDef *hcan, Sensor_HandleTypedef *Sensor, Encoder_HandleTypeDef *Encoderx, Encoder_HandleTypeDef *Encodery);
void CAN_Reset_IMU(CAN_HandleTypeDef *hcan, Sensor_HandleTypedef *Sensor);

void CAN_Stop_Sensor(CAN_HandleTypeDef *hcan, Sensor_HandleTypedef *Sensor);

void CAN_Assign_Encoder(CAN_HandleTypeDef *hcan, Sensor_HandleTypedef Sensor, Encoder_HandleTypeDef *Encoderx, Encoder_HandleTypeDef *Encodery);

/* Refeedback function  *******************************************************/
void CAN_Slave_FIFO0_ReFb_Handle(CAN_HandleTypeDef *hcan);

/* Sensor data transmit function  *********************************************/
void CAN_IMU_Data_Transmit(CAN_HandleTypeDef *hcan, Sensor_HandleTypedef *IMU, uint8_t aData[6]);
void CAN_Encoder_Data_Transmit(CAN_HandleTypeDef *hcan, Sensor_HandleTypedef *Encoder, float x_pos, float y_pos);

/* Error feedback function  ***************************************************/
void CAN_Sensor_ErrorFb(CAN_HandleTypeDef *hcan, Sensor_HandleTypedef Sensor);

#endif
