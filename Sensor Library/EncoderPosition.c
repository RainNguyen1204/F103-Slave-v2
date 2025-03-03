/**
  ******************************************************************************
  * @file    	EncoderPossition.c
  * @author  	Nguyen Vu
	*	@version 	1.0.1
  * @brief   	This file provides function to calculate position using encoder
  *****************************************************************************/
	
/* Includes ------------------------------------------------------------------*/
#include "EncoderPosition.h"

/** @brief    Encoder basic function for cauculating position
  ==============================================================================
										##### Encoder Basic Functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Initialize Encoder.
    (+) Counting CNT value and handling overflow / breakdown.
    (+) Converting CNT value to encoder's pulse.
    (+) Calculating position.
  */


/**
  * @brief  Initializes the encoder
	* @note		Initializes the encoder in the main function and before while loop
  * @param	encoder   Pointer to the Encoder_HandleTypeDef structure.
	* @param 	htim      Pointer to the TIM_HandleTypeDef structure used for the encoder.
	* @param	Z_Pin			Pin that recieve z pulse from encoder.
	* @param 	resolution Resolution of the encoder (number of pulses per revolution).
  */
void Encoder_Init(Encoder_HandleTypeDef *encoder, TIM_HandleTypeDef *htim, uint16_t resolution, uint16_t Z_Pin)
{
	encoder->htim = htim;
	encoder->Z_Pin = Z_Pin;
	encoder->resolution = resolution;
	encoder->CNT_value = 0;
	encoder->last_CNT_value = encoder->pulse = 0;
}

/**
  * @brief 	Updating encoder's CNT value using timer
	*					and handling CNT overflow / breakdown
  * @param 	encoder		Pointer to the Encoder_HandleTypeDef structure.
  */
void Encoder_CNT_Counter(Encoder_HandleTypeDef *encoder)	
{
	//update current CNT value
	uint16_t current_CNT_value = encoder->htim->Instance->CNT;
	
	//difference between CNT current value and last CNT value
	int16_t diff;
	
	//if encoder is not moving
	if (current_CNT_value ==  encoder->last_CNT_value)
		diff = 0;
	
	//if encoder is moving
	else if (current_CNT_value > encoder->last_CNT_value)
	{
		//CNT overflow handling
		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(encoder->htim))
			diff = -encoder->last_CNT_value - (TIMER_MAX_CNT - current_CNT_value);
		else
			diff = current_CNT_value - encoder->last_CNT_value;
	}
	else
	{
		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(encoder->htim))
			diff = current_CNT_value - encoder->last_CNT_value;
		//CNT breakdown handling
		else
			diff = current_CNT_value + (TIMER_MAX_CNT - encoder->last_CNT_value);
	}
	
	//update total CNT
	encoder->CNT_value += diff;
	
	//update last CNT value
	encoder->last_CNT_value = current_CNT_value;
}

/**
  * @brief 	Counting encoder's pulse
  * @param 	encoder    Pointer to the Encoder_HandleTypeDef structure.
	* @note 	1 pulse value = 4 CNT value
  */
void Encoder_Pulse_Counter(Encoder_HandleTypeDef *encoder)
{
	Encoder_CNT_Counter(encoder);
	encoder->pulse = encoder->CNT_value/4;
}

/**
  * @brief 	Counting encoder's pulse
	* @note 	Placing in the while loop to update all CNT value
	*					postition = wheel_circumference*pulse/resolution
	* @param	encoder		Pointer to the Encoder_HandleTypeDef structure.
	* @param	wheel_diameter
  */
void Encoder_Position_Handle(Encoder_HandleTypeDef *encoder, float wheel_diameter)
{
	Encoder_CNT_Calibration(encoder);
	Encoder_Pulse_Counter(encoder);
	encoder->position = encoder->asign_position + PI*wheel_diameter*encoder->pulse/encoder->resolution;
}

/** @brief    Encoder calibration funtion using Z pulse and GPIO interupt
  ==============================================================================
								##### Encoder Calibration Funtion #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
		(+) Detecting encoder's z pulse.
    (+) Counting encoder's rounds.
    (+) Finding offset value.
    (+) Calibrating encoder's pulse.
    (+) Calculating position.
  */

/**
  * @brief 	Detecting encoder's z pulse
	* @note 	Placing in GPIO interupt callback
	* @param	encoder		Pointer to the Encoder_HandleTypeDef structure
	* @param	GPIO_Pin	Pin that use to compare with encoder Z_pin
  */
void Encoder_Zpulse_Dectect(Encoder_HandleTypeDef *encoder, uint16_t GPIO_Pin)
{
	if (GPIO_Pin == encoder->Z_Pin)
		encoder->z_pulse_flag = 1;	//Update flag
}

/**
  * @brief 	Finding offset value
	* @param	encoder		Pointer to the Encoder_HandleTypeDef structure.
	* @return True: 		if first Z pulse was detected
	*					False: 		if this is the first time this function has been call  
  */
uint8_t Encoder_Offset_Detect(Encoder_HandleTypeDef *encoder)
{
	if (!encoder->offset_flag)
	{
		encoder->offset_value = encoder->CNT_value;
		encoder->last_direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(encoder->htim);
		encoder->offset_flag = 1;
		return 0;
	}
	else
		return 1; 
}

/**
  * @brief 	Counting encoder's round
	* @param	encoder		Pointer to the Encoder_HandleTypeDef structure.
  */
void Encoder_Round_Counter(Encoder_HandleTypeDef *encoder) 
{
	//Start counting after finding the first z pulse
	if (!Encoder_Offset_Detect(encoder))
		return;
	
	//Update current CNT direction
	uint8_t current_direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(encoder->htim);
	
	if (current_direction)		//if count down
		encoder->round_counter--;
	else											//if count up
		encoder->round_counter++;
	
	//Skip 1 round if change direction
	if(encoder->last_direction != current_direction)
	{
		if (current_direction)	//if count down
			encoder->round_counter++;
		else										//if count up
			encoder->round_counter--;
	}
	
	//Update last CNT direction
	encoder->last_direction = current_direction;
}

/**
  * @brief 	Calibrating encoder's CNT value
	* @note		Place this funtion in while loop to wait for z pluse detect
	*					CNT = round*4*res + offset
	* @param	encoder		Pointer to the Encoder_HandleTypeDef structure.
  */
void Encoder_CNT_Calibration(Encoder_HandleTypeDef *encoder)
{
	//Checking flag from GPIO interupt
	if (!encoder->z_pulse_flag)
		return;
	
	//Calibration
	Encoder_Round_Counter(encoder); 
	encoder->CNT_value = encoder->round_counter* 4 * encoder->resolution + encoder->offset_value;
	
	//Turn off flag
	encoder->z_pulse_flag = 0;
}

/** @brief    Encoder control funtion 
  ==============================================================================
								##### Encoder Control Funtion #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
		(+) Reset encoder value.
    (+) Assign new position value.
  */

/**
  * @brief 	Reset encoder's value
	* @param	encoder		Pointer to the Encoder_HandleTypeDef structure.
  */
void Encoder_Reset(Encoder_HandleTypeDef *encoder)
{
	encoder->htim->Instance->CNT	= 0;
	encoder->CNT_value 						= 0;
	encoder->last_CNT_value				= 0;
	encoder->pulse 								= 0;
	encoder->z_pulse_flag 				= 0;
	encoder->offset_flag 					= 0;
	encoder->offset_value 				= 0;
	encoder->last_direction 			= 0;
	encoder->round_counter 				= 0;
	encoder->asign_position 			= 0;
	encoder->position 						= 0;
}	

/**
  * @brief 	Assign new encoder's position value
	* @param	encoder				Pointer to the Encoder_HandleTypeDef structure.
	* @param	new_position	New position to assign.
  */
void Encoder_Assign_Position(Encoder_HandleTypeDef *encoder, float new_position)
{
	encoder->htim->Instance->CNT	= 0;
	encoder->CNT_value 						= 0;
	encoder->last_CNT_value				= 0;
	encoder->pulse 								= 0;
	encoder->z_pulse_flag 				= 0;
	encoder->offset_flag 					= 0;
	encoder->offset_value 				= 0;
	encoder->last_direction 			= 0;
	encoder->round_counter 				= 0;
	encoder->asign_position 			= new_position;
	encoder->position 						= 0;
}


