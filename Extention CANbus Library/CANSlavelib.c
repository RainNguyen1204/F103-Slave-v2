/**
  ******************************************************************************
  * @file    	CANSlavelib.c
  * @author  	Nguyen Vu
	*	@version 	1.0.0
  * @brief   	This file provides function for slave controling sensor
							and communicate to master
  *****************************************************************************/
	
/* Includes ------------------------------------------------------------------*/
#include "CANSlavelib.h"

static CAN_TxHeaderTypeDef 	Slave_TxHeader;
static uint32_t 						mailbox;

static CAN_TxQueue 		Slave_TxQueue;
static CAN_TxMessage 	Slave_TxMessage;
static CAN_RxQueue		Slave_RxQueue;
static CAN_RxMessage	Slave_RxMessage;

/** @brief    CAN Slave basic function for transmition and receiving
  ==============================================================================
										##### Slave Basic Functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
		(+) Initialize Sensor.
    (+) Combine Sensor Id and command Id to make StdId.
    (+) Gettinf sensor Id or command Id from a RxHeader.StdId.
  */
	
/**
  * @brief  Initializes Sensor Id.
	* @note		Initializes the Sensor in the main function and before while loop
  * @param	Sensor   	Pointer to the Sensor_HandleTypedef structure.
	* @param 	sensor_id Sensor ID for initializing.
  */
	void CAN_Sensor_Init(Sensor_HandleTypedef *Sensor, uint32_t sensor_id)
{
	Sensor->sensor_id 	= sensor_id;
	Sensor->freq				= 0;
	Sensor->start_flag	= 0;
}

/**
  * @brief  Create StdId by combining sensor id and cmd id.
	* @param 	Sensor_Id Sensor ID.
  * @param	Cmd_Id   	Command ID.
	* @return	New StdId by combining Sensor_Id and Cmd_Id
  */
uint32_t CAN_Command_StdId(uint32_t Sensor_Id, uint32_t Cmd_Id)
{
	return (Sensor_Id << 5) | Cmd_Id;
}

/**
  * @brief  Get Sensor Id in a RxHeader.
  * @param	RxHeader   	CAN RxHeader.
	* @return	Sensor_Id
  */
uint8_t getSensor_Id(CAN_RxHeaderTypeDef RxHeader)
{
	return RxHeader.StdId >> 5;
}

/**
  * @brief  Get Cmd Id in a RxHeader.
  * @param	RxHeader   	CAN RxHeader.
	* @return	Cmd_Id
  */
uint8_t getSensor_Cmd(CAN_RxHeaderTypeDef RxHeader)
{
	return RxHeader.StdId & 0x1F;
}

/** @brief    Slave feedback function to master
  ==============================================================================
									##### Slave Feedback Functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
		(+) Start a Sensor.
    (+) Reset a Sensor.
    (+) Assign new position for Encoder.
  */

/**
  * @brief  	Feedback frequency to master after start a sensor.
	* @param		hcan  Pointer to the CAN_HandleTypeDef structure.
  */
void CAN_Sensor_Start_fb(CAN_HandleTypeDef *hcan)
{
	//Checking if TxQueue created
	CAN_If_TxQueue_notCreate(&Slave_TxQueue);
	
	//Initialize TxHeader
	CAN_TxHeader_Init(&Slave_TxHeader, CAN_Command_StdId(getSensor_Id(CAN_RxQueue_getFront(&Slave_RxQueue).RxHeader), START_FB_ID), START_FB_DLC);
	
	//Find empty mailbox
	mailbox = get_Empty_Mailbox();
	
	//Sending message
	if (HAL_CAN_AddTxMessage(hcan, &Slave_TxHeader, CAN_RxQueue_getFront(&Slave_RxQueue).rxdata, &mailbox) != HAL_OK)
	{
		//If failed, store message in queue for next transmit
		CAN_TxHeader_Copy(&Slave_TxMessage.TxHeader, Slave_TxHeader);
		CAN_Data_Copy(Slave_TxMessage.txdata, CAN_RxQueue_getFront(&Slave_RxQueue).rxdata);
		CAN_EnTxQueue(&Slave_TxQueue, Slave_TxMessage); 
	}
}

/**
  * @brief  	Feedback reseting sensor complete to master.
	* @param		hcan  Pointer to the CAN_HandleTypeDef structure.
  */
void CAN_Sensor_Reset_fb(CAN_HandleTypeDef *hcan)
{
	//Checking if TxQueue created
	CAN_If_TxQueue_notCreate(&Slave_TxQueue);
	
	//Initialize TxHeader
	CAN_TxHeader_Init(&Slave_TxHeader, CAN_Command_StdId(getSensor_Id(CAN_RxQueue_getFront(&Slave_RxQueue).RxHeader), RESET_FB_ID), RESET_FB_DLC);
	
	//Find empty mailbox
	mailbox = get_Empty_Mailbox();
	
	//Sending message
	if (HAL_CAN_AddTxMessage(hcan, &Slave_TxHeader, NULL, &mailbox) != HAL_OK)
	{
		//If failed, store message in queue for next transmit
		CAN_TxHeader_Copy(&Slave_TxMessage.TxHeader, Slave_TxHeader);
		CAN_EnTxQueue(&Slave_TxQueue, Slave_TxMessage); 
	}
}

/**
  * @brief  	Feedback reseting sensor complete to master.
	* @param		hcan  Pointer to the CAN_HandleTypeDef structure.
  */
void CAN_Sensor_Stop_fb(CAN_HandleTypeDef *hcan)
{
	//Checking if TxQueue created
	CAN_If_TxQueue_notCreate(&Slave_TxQueue);
	
	//Initialize TxHeader
	CAN_TxHeader_Init(&Slave_TxHeader, CAN_Command_StdId(getSensor_Id(CAN_RxQueue_getFront(&Slave_RxQueue).RxHeader), STOP_FB_ID), STOP_FB_DLC);
	
	//Find empty mailbox
	mailbox = get_Empty_Mailbox();
	
	//Sending message
	if (HAL_CAN_AddTxMessage(hcan, &Slave_TxHeader, NULL, &mailbox) != HAL_OK)
	{
		//If failed, store message in queue for next transmit
		CAN_TxHeader_Copy(&Slave_TxMessage.TxHeader, Slave_TxHeader);
		CAN_EnTxQueue(&Slave_TxQueue, Slave_TxMessage); 
	}
}

/**
  * @brief  	Feedback new assign position to master.
	* @param		hcan  Pointer to the CAN_HandleTypeDef structure.
  */
void CAN_Sensor_Assign_fb(CAN_HandleTypeDef *hcan)
{
	//Checking if TxQueue created
	CAN_If_TxQueue_notCreate(&Slave_TxQueue);
	
	//Initialize TxHeader
	CAN_TxHeader_Init(&Slave_TxHeader, CAN_Command_StdId(getSensor_Id(CAN_RxQueue_getFront(&Slave_RxQueue).RxHeader), ASSIGN_FB_ID), ASSIGN_FB_DLC);
	
	//Find empty mailbox
	mailbox = get_Empty_Mailbox();
	
	//Sending message
	if (HAL_CAN_AddTxMessage(hcan, &Slave_TxHeader, CAN_RxQueue_getFront(&Slave_RxQueue).rxdata, &mailbox) != HAL_OK)
	{
		//If failed, store message in queue for next transmit
		CAN_TxHeader_Copy(&Slave_TxMessage.TxHeader, Slave_TxHeader);
		CAN_Data_Copy(Slave_TxMessage.txdata, CAN_RxQueue_getFront(&Slave_RxQueue).rxdata);
		CAN_EnTxQueue(&Slave_TxQueue, Slave_TxMessage); 
	}
}

/**
  * @brief  	Retransmit failed feedback.
	* @param		hcan  Pointer to the CAN_HandleTypeDef structure.
  */
void CAN_Slave_FIFO0_ReFb_Handle(CAN_HandleTypeDef *hcan)
{
	//No retransmit if queue is empty
	if (CAN_TxQueue_isEmpty(&Slave_TxQueue))
		return;
	
	//Prepare before transmit 
	CAN_TxHeader_Copy(&Slave_TxHeader, CAN_TxQueue_getFront(&Slave_TxQueue).TxHeader);
	mailbox = get_Empty_Mailbox();
	
	//Dequeue if transmit successed
	if(HAL_CAN_AddTxMessage(hcan, &Slave_TxHeader, CAN_TxQueue_getFront(&Slave_TxQueue).txdata, &mailbox) == HAL_OK)
		CAN_DeTxQueue(&Slave_TxQueue);
}

/** @brief    Slave contronling function for controling sensor
  ==============================================================================
									##### Slave Controling Functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
		(+) Start a Sensor.
    (+) Reset a Sensor.
    (+) Assign new position for Encoder.
  */

/** @brief    Slave function for starting Sensor
  ------------------------------------------------------------------------------
										##### Start Sensor Functions #####
  ------------------------------------------------------------------------------
  */

/**
  * @brief  Start Handle function.
	* @note 	Place this function beforn main function
	*					and call any start function in this.
  */
__weak void CAN_Sensor_Start_Handle(void)
{
	
}

/**
  * @brief  	Start IMU function.
	* @param		Sensor   	Pointer to the Sensor_HandleTypedef structure.
	* @param		huart    	UART port which connecting to IMU.
	* @param		rxdata		A pointer to store fisrt UARTT data from IMU
	* @note 		Call this function in CAN_Sensor_Start_Handle.
	* @warning	This function only start 1 time, second time only feedback the frequency.
  */
void CAN_Start_IMU(Sensor_HandleTypedef *Sensor ,UART_HandleTypeDef *huart, uint8_t *rxdata)
{
	static uint8_t first_time;
	if (getSensor_Id(CAN_RxQueue_getFront(&Slave_RxQueue).RxHeader) == IMU_ID)
	{
		Sensor->start_flag = 1;
		Sensor->freq = (uint16_t)((uint16_t)CAN_RxQueue_getFront(&Slave_RxQueue).rxdata[1] << 8 | CAN_RxQueue_getFront(&Slave_RxQueue).rxdata[0]);
		if (first_time) 
			return;
		HAL_UART_Receive_IT(huart, rxdata, 1);
		first_time = 1;
	}
}

/**
  * @brief 		Start Encoder function.
	* @param		Sensor   	Pointer to the Sensor_HandleTypedef structure.
	* @param		htim1    	A timer connect with an encoder.
	* @param		htim2    	Another timer connect with an encoder.
	* @note 		Call this function in CAN_Sensor_Start_Handle.
	* @warning	This function only start 1 time, second time only feedback the frequency.
  */
void CAN_Start_Encoder(Sensor_HandleTypedef *Sensor, TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2)
{
	static uint8_t first_time;
	if (getSensor_Id(CAN_RxQueue_getFront(&Slave_RxQueue).RxHeader) == ENC_ID)
	{
		Sensor->start_flag = 1;
		Sensor->freq = (uint16_t)((uint16_t)CAN_RxQueue_getFront(&Slave_RxQueue).rxdata[1] << 8 | CAN_RxQueue_getFront(&Slave_RxQueue).rxdata[0]);
		
		if (first_time)
			return;
		HAL_TIM_Encoder_Start(htim1, TIM_CHANNEL_ALL);
		HAL_TIM_Encoder_Start(htim2, TIM_CHANNEL_ALL);
		first_time = 1;
	}
}

/** @brief    Slave function for reseting Sensor
  ------------------------------------------------------------------------------
										##### Reset Sensor Functions #####
  ------------------------------------------------------------------------------
  */

/**
  * @brief  Reset Handle function.
	* @note 	Place this function beforn main function
	*					and call any reset function in this.
  */
__weak void CAN_Sensor_Reset_Handle(void)
{
	
}

/**
  * @brief  	Reset IMU function.
	* @param		hcan	   	Pointer to the CAN_HandleTypeDef structure.
	* @param		Sensor   	Pointer to the Sensor_HandleTypedef structure.
	* @note 		Call this function in CAN_Sensor_Reset_Handle.
	* @warning	This function only feedback after starting sensor.
  */
void CAN_Reset_IMU(CAN_HandleTypeDef *hcan, Sensor_HandleTypedef *Sensor)
{
	if (getSensor_Id(CAN_RxQueue_getFront(&Slave_RxQueue).RxHeader) == IMU_ID)
	{
		IMU_Reset_Flag();
		if (!Sensor->freq)
			return;
		Sensor->start_flag = 1;
		CAN_Sensor_Reset_fb(hcan);
	}
}

/**
  * @brief  	Reset Encoder function.
	* @param		hcan	   	Pointer to the CAN_HandleTypeDef structure.
	* @param		Sensor   	Pointer to the Sensor_HandleTypedef structure.
	*	@param		Encoderx	X axis encoder 
	*	@param		Encodery	Y axis encoder 
	* @note 		Call this function in CAN_Sensor_Reset_Handle.
	* @warning	This function only feedback after starting sensor.
  */
void CAN_Reset_Encoder(CAN_HandleTypeDef *hcan, Sensor_HandleTypedef *Sensor, Encoder_HandleTypeDef *Encoderx, Encoder_HandleTypeDef *Encodery)
{
	if (getSensor_Id(CAN_RxQueue_getFront(&Slave_RxQueue).RxHeader) == ENC_ID)
	{
		Encoder_Reset(Encoderx);
		Encoder_Reset(Encodery);
		if (!Sensor->freq)
			return;
		Sensor->start_flag = 1;
		CAN_Sensor_Reset_fb(hcan);
	}
}

/** @brief    Slave function for stop Sensor transmit data
  ------------------------------------------------------------------------------
										##### Stop Sensor Functions #####
  ------------------------------------------------------------------------------
  */

/**
  * @brief  Stop Handle function.
	* @note 	Place this function beforn main function
	*					and call any sensor stop function in this.
  */
__weak void CAN_Sensor_Stop_Handle(void)
{
	
}

/**
  * @brief  	Stop Sensor function.
	* @param		hcan	   	Pointer to the CAN_HandleTypeDef structure.
	* @param		Sensor   	Pointer to the Sensor_HandleTypedef structure.
	* @note 		Call this function in CAN_Sensor_Stop_Handle.
	* @warning	This function only feedback after starting sensor.
  */
void CAN_Stop_Sensor(CAN_HandleTypeDef *hcan, Sensor_HandleTypedef *Sensor)
{
	if (getSensor_Id(CAN_RxQueue_getFront(&Slave_RxQueue).RxHeader) == Sensor->sensor_id )
	{
		if (!Sensor->freq)
			return;
		Sensor->start_flag = 0;
		CAN_Sensor_Stop_fb(hcan);
	}
}

/** @brief    Slave function for assign new postion for Encoder
  ------------------------------------------------------------------------------
										##### Assign Encoder Functions #####
  ------------------------------------------------------------------------------
  */

/**
  * @brief  Assign Handle function.
	* @note 	Place this function beforn main function
	*					and call any encoder assign function in this.
  */
__weak void CAN_Encoder_Assign_Handle(void)
{
	
}

/**
  * @brief  	Assign Encoder function.
	* @param		hcan	   	Pointer to the CAN_HandleTypeDef structure.
	* @param		Sensor   	Pointer to the Sensor_HandleTypedef structure.
	*	@param		Encoderx	X axis encoder 
	*	@param		Encodery	Y axis encoder 
	* @note 		Call this function in CAN_Encoder_Assign_Handle.
	* @warning	This function only feedback after starting sensor.
  */
void CAN_Assign_Encoder(CAN_HandleTypeDef *hcan, Sensor_HandleTypedef Sensor, Encoder_HandleTypeDef *Encoderx, Encoder_HandleTypeDef *Encodery)
{
	if (getSensor_Id(CAN_RxQueue_getFront(&Slave_RxQueue).RxHeader) == ENC_ID)
	{
		//Combine uint8_t arry into uint32_t
		uint8_t data[8];
		CAN_Data_Copy(data, CAN_RxQueue_getFront(&Slave_RxQueue).rxdata);
		
		uint32_t x_raw = ((uint32_t)data[0]) | ((uint32_t)data[1] << 8) | 
																((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
		uint32_t y_raw = ((uint32_t)data[4]) | ((uint32_t)data[5] << 8) | 
																((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24);
		
		//Convert uint32_t to float data
		float x_pos = *(float *)&x_raw;
		float y_pos = *(float *)&y_raw;
		
		//Assign new value
		Encoder_Assign_Position(Encoderx, x_pos);
		Encoder_Assign_Position(Encodery, y_pos);
		
		//Feedback assign value
		if (!Sensor.freq)
			return;
		CAN_Sensor_Assign_fb(hcan);
	}
}

/** @brief    Slave receiving command from master
  ==============================================================================
								##### Slave Recieve Command Functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
		(+) Receiving data handling.
    (+) Feedback after receiving.
  */

/**
  * @brief  	Receiving command from master.
	* @param		hcan	   	Pointer to the CAN_HandleTypeDef structure.
	* @note 		Call this function in HAL_CAN_RxFifo0MsgPendingCallback.
	* @warning	Active CAN_IT_RX_FIFO0_MSG_PENDING at lest 1 time before.
  */
void CAN_Slave_FIFO0_RxMessage(CAN_HandleTypeDef *hcan)
{
	if ((HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Slave_RxMessage.RxHeader, Slave_RxMessage.rxdata) != HAL_OK))
		return;
	if ((getSensor_Id(Slave_RxMessage.RxHeader) == IMU_ID) || getSensor_Id(Slave_RxMessage.RxHeader) == ENC_ID)
	{
		CAN_If_RxQueue_notCreate(&Slave_RxQueue);
		CAN_EnRxQueue(&Slave_RxQueue, Slave_RxMessage);
	}
}

/**
  * @brief  	Receiving Start command handle.
	* @param		hcan	   	Pointer to the CAN_HandleTypeDef structure.
  */
void CAN_RxStart_RQ(CAN_HandleTypeDef *hcan)
{
	if (getSensor_Cmd(CAN_RxQueue_getFront(&Slave_RxQueue).RxHeader) == START_ID)
	{
		CAN_Sensor_Start_Handle();
		CAN_Sensor_Start_fb(hcan);
	}
}

/**
  * @brief  	Receiving Reset command handle.
  */
void CAN_RxReset_RQ(void)
{
	if (getSensor_Cmd(CAN_RxQueue_getFront(&Slave_RxQueue).RxHeader) == RESET_ID)
	{
		CAN_Sensor_Reset_Handle();
	}
}

/**
  * @brief  	Receiving Stop command handle.
  */
void CAN_RxStop_RQ(void)
{
	if (getSensor_Cmd(CAN_RxQueue_getFront(&Slave_RxQueue).RxHeader) == STOP_ID)
	{
		CAN_Sensor_Stop_Handle();
	}
}

/**
  * @brief  	Receiving Assign command handle.
  */
void CAN_RxEncoder_AssignRQ(void)
{
	if (getSensor_Cmd(CAN_RxQueue_getFront(&Slave_RxQueue).RxHeader) == ENC_ASSIGN_ID)
	{
		CAN_Encoder_Assign_Handle();
	}
}

/**
  * @brief  	Receiving command handle.
	* @param	hcan   		Pointer to the CAN_HandleTypeDef structure.
  */
void CAN_Slave_FIFO0_Recieve_Cmd_Handle(CAN_HandleTypeDef *hcan)
{
	if (Slave_RxQueue.used)
	{
		CAN_RxStart_RQ(hcan);
		CAN_RxReset_RQ();
		CAN_RxStop_RQ();
		CAN_RxEncoder_AssignRQ();

		CAN_DeRxQueue(&Slave_RxQueue);
	}
}

/** @brief    Slave transmiting data to master
  ==============================================================================
								##### Slave Transmit Data Functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
		(+) Transmiting data.
  */

/**
  * @brief  	Transmit IMU hex data.
	* @param		hcan	   	Pointer to the CAN_HandleTypeDef structure.
	* @param		IMU	   		Pointer to the Sensor_HandleTypedef structure.
	* @param		aData	   	IMU hex data array.
  */
void CAN_IMU_Data_Transmit(CAN_HandleTypeDef *hcan, Sensor_HandleTypedef *IMU, uint8_t aData[6])
{
	//Stop transmit for feedback first
	if (Slave_RxQueue.used	|| Slave_TxQueue.used)
		return;
	
	//No transmit before start sensor
	if((!IMU->freq) && (!IMU->start_flag))
		return;
	
	//Transmition handle
	static uint32_t time = 0;
	if ((HAL_GetTick() - time) > IMU->freq)
	{
		CAN_TxHeader_Init(&Slave_TxHeader, CAN_Command_StdId(IMU_ID, IMU_DATA), IMU_DATA_DLC);
		mailbox = get_Empty_Mailbox();
		HAL_CAN_AddTxMessage(hcan, &Slave_TxHeader, aData, &mailbox);
		time = HAL_GetTick();
	}
}

/**
  * @brief  	Transmit Encoder position.
	* @param		hcan	   	Pointer to the CAN_HandleTypeDef structure.
	* @param		IMU	   		Pointer to the Sensor_HandleTypedef structure.
	* @param		aData	   	IMU hex data array.
  */
void CAN_Encoder_Data_Transmit(CAN_HandleTypeDef *hcan, Sensor_HandleTypedef *Encoder, float x_pos, float y_pos)
{
	//Stop transmit for feedback first
	if (Slave_RxQueue.used || Slave_TxQueue.used)
		return;
	
	//No transmit before start sensor
	if ((!Encoder->freq) && (!Encoder->start_flag))
		return;
	
	//Transmition handle	
	static uint32_t time = 0;
	if ((HAL_GetTick() - time) > Encoder->freq)
	{
		//Convert float data to uint32_t data
		uint32_t *x_raw = (uint32_t *)&x_pos;
		uint32_t *y_raw = (uint32_t *)&y_pos;
		
		//Divide uint32_t data to uint8_t array
		uint8_t data[8];
		data[0] = (*x_raw >> 0) & 0xFF;  
    data[1] = (*x_raw >> 8) & 0xFF;
    data[2] = (*x_raw >> 16) & 0xFF;
    data[3] = (*x_raw >> 24) & 0xFF;
		data[4] = (*y_raw >> 0) & 0xFF;  
    data[5] = (*y_raw >> 8) & 0xFF;
    data[6] = (*y_raw >> 16) & 0xFF;
    data[7] = (*y_raw >> 24) & 0xFF;
		
		//Initialize TxHeader
		CAN_TxHeader_Init(&Slave_TxHeader, CAN_Command_StdId(ENC_ID, ENC_DATA), ENC_DATA_DLC);
		
		//Find empty mailbox
		mailbox = get_Empty_Mailbox();
		HAL_CAN_AddTxMessage(hcan, &Slave_TxHeader, data, &mailbox);
		time = HAL_GetTick();
	}
}

/** @brief    Slave report error to master
  ==============================================================================
								##### Error Report Functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
		(+) Send Error Report.
  */

/**
  * @brief  	Send error report.
	* @param		hcan	   	Pointer to the CAN_HandleTypeDef structure.
	* @param		Sensor 		Pointer to the Sensor_HandleTypedef structure.
  */
void CAN_Sensor_ErrorFb(CAN_HandleTypeDef *hcan, Sensor_HandleTypedef Sensor)
{
	//Initialize TxHeader
	CAN_TxHeader_Init(&Slave_TxHeader, CAN_Command_StdId(Sensor.sensor_id, ERROR_ID), ERROR_DLC);
	
	//Find empty mailbox
	mailbox = get_Empty_Mailbox();
	if (HAL_CAN_AddTxMessage(hcan, &Slave_TxHeader, NULL, &mailbox) != HAL_OK)
	{
		//If failed, store message in queue for next transmit
		CAN_TxHeader_Copy(&Slave_TxMessage.TxHeader, Slave_TxHeader);
		CAN_EnTxQueue(&Slave_TxQueue, Slave_TxMessage);
	}
}

