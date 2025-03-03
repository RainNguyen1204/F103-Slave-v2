/**
  ******************************************************************************
  * @file    	EncoderPosition.h
  * @author  	Nguyen Vu
  * @brief   	This file contains all the functions prototypes 
	*						for the encoder driver
  *****************************************************************************/
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ENCODERPOSITION_H_
#define ENCODERPOSITION_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/**
  * @brief  Configuration Value
  */
#define ZX_PIN					GPIO_PIN_3
#define ZY_PIN					GPIO_PIN_4
#define WHEEL_DIAMETER	50.0

/**
  * @brief  Constant Value
  */
#define PI			3.14159265358979323846 
#define TIMER_MAX_CNT	65535
#define TIMER_MIN_CNT	0

//Encoder Struct
typedef struct
{
	uint16_t 					resolution				;
	TIM_HandleTypeDef *htim							;
	uint16_t 					Z_Pin							;
	
	int32_t						CNT_value					;
	uint16_t					last_CNT_value		;
	int32_t						pulse							;
	
	uint8_t						z_pulse_flag			;
	uint8_t						offset_flag				;
	int16_t						offset_value			;
	
	uint8_t						last_direction		;
	int16_t						round_counter			;
	
	float							asign_position		;
	float							position					;
}Encoder_HandleTypeDef;

/* Initialization and basic handling functions  *******************************/
void Encoder_Init(Encoder_HandleTypeDef *encoder, TIM_HandleTypeDef *htim, uint16_t resolution, uint16_t Z_Pin);
void Encoder_Position_Handle(Encoder_HandleTypeDef *encoder, float wheel_diameter);	

/* Calibration using z pulse functions  ***************************************/
void Encoder_Zpulse_Dectect(Encoder_HandleTypeDef *encoder, uint16_t GPIO_Pin);
void Encoder_CNT_Calibration(Encoder_HandleTypeDef *encoder); 

/* Encoder controlling functions  *********************************************/
void Encoder_Reset(Encoder_HandleTypeDef *encoder);
void Encoder_Assign_Position(Encoder_HandleTypeDef *encoder, float new_position);

#endif



