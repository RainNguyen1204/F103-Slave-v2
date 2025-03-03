/**
  ******************************************************************************
  * @file    	bxCANlib.c
  * @author  	Nguyen Vu
	*	@version 	1.0.0
  * @brief   	This file provides function to config CANbus variable easier
  *****************************************************************************/
	
/* Includes ------------------------------------------------------------------*/
#include "bxCANlib.h"
#include "stdlib.h"

/** @brief    Basic CANbus function for initialize and configuration
  ==============================================================================
										##### CANbus Basic Functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Initialize TxHeader.
    (+) Configuration CAN FIFO filter (now only for Fifo0).
    (+) Finding empty mailbox for sending message.
    (+) Copying TxHeader, RxHeader, arrayData[8].
  */
	
/**
  * @brief 		Initialize TxHeader.
	* @param		TxHeader  Pointer to the CAN_TxHeaderTypeDef structure.
	* @param		StdId    	TxHeader's StdId.
	* @param		DLC				TxHeader's DLC.
  */
void CAN_TxHeader_Init(CAN_TxHeaderTypeDef *TxHeader, uint32_t StdId, uint32_t DLC)
{
	TxHeader->DLC 								= DLC;
	TxHeader->ExtId 							= 0;
	TxHeader->IDE 								= CAN_ID_STD;
	TxHeader->RTR 								= CAN_RTR_DATA;
	TxHeader->StdId								= StdId;
	TxHeader->TransmitGlobalTime	= DISABLE;
}

/**
  * @brief 		Configuration Rx FIFO0 Filter.
	* @param		hcan		  			Pointer to the CAN_HandleTypeDef structure.
	* @param		canfilter				Pointer to the CAN_FilterTypeDef structure.
	* @param		FilterBank			CAN FilerBank (F103 among 0-13).
	* @param		Filter_Id				CAN Filer ID.
	* @param		Filter_Id_Mask	CAN Filer ID Mask.
  */
void CAN_Fifo0_Filter_Config(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *canfilter, uint32_t FilterBank, 
																uint32_t Filter_Id, uint32_t Filter_Id_Mask)
{
	canfilter->FilterActivation 		= CAN_FILTER_ENABLE;
	canfilter->FilterBank 					= FilterBank;
	canfilter->FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilter->FilterIdHigh 				= Filter_Id<<5;
	canfilter->FilterIdLow					= 0x0000;
	canfilter->FilterMaskIdHigh			= Filter_Id_Mask<<5;
	canfilter->FilterMaskIdLow			=	0x0000;
	canfilter->FilterMode						= CAN_FILTERMODE_IDMASK;
	canfilter->FilterScale					=	CAN_FILTERSCALE_32BIT;
	canfilter->SlaveStartFilterBank	= 0;
	
	HAL_CAN_ConfigFilter(hcan, canfilter);
}

/**
  * @brief 		Finding empty mailbox.
	* @return		empty mailbox or no mailbox
  */
uint32_t get_Empty_Mailbox(void)
{
	if (CAN1->TSR & CAN_TSR_TME0)
		return CAN_TX_MAILBOX0;
	else if (CAN1->TSR & CAN_TSR_TME1)
		return CAN_TX_MAILBOX1;
	else if (CAN1->TSR & CAN_TSR_TME2)
		return CAN_TX_MAILBOX2;
	return HAL_BUSY;
}

/**
  * @brief 		Copy TxHeader from a TxHeader.
	* @param		TxHeader					A pointer to store the copy data.
	* @param		TxHeader_Sample		TxHeader want to copy.
  */
void CAN_TxHeader_Copy(CAN_TxHeaderTypeDef *TxHeader, CAN_TxHeaderTypeDef TxHeader_Sample)
{
	TxHeader->DLC									= TxHeader_Sample.DLC;
	TxHeader->ExtId								= TxHeader_Sample.ExtId;
	TxHeader->IDE 								= TxHeader_Sample.IDE;
	TxHeader->RTR								 	= TxHeader_Sample.RTR;
	TxHeader->StdId 							= TxHeader_Sample.StdId;
	TxHeader->TransmitGlobalTime	= TxHeader_Sample.TransmitGlobalTime;
}

/**
  * @brief 		Copy RxHeader from a RxHeader.
	* @param		RxHeader					A pointer to store the copy data.
	* @param		RxHeader_Sample		RxHeader want to copy.
  */
void CAN_RxHeader_Copy(CAN_RxHeaderTypeDef *RxHeader, CAN_RxHeaderTypeDef RxHeader_Sample)
{
	RxHeader->DLC				= RxHeader_Sample.DLC;
	RxHeader->ExtId			= RxHeader_Sample.ExtId;
	RxHeader->IDE 			= RxHeader_Sample.IDE;
	RxHeader->RTR				= RxHeader_Sample.RTR;
	RxHeader->StdId 		= RxHeader_Sample.StdId;
	RxHeader->Timestamp	= RxHeader_Sample.Timestamp;
}

/**
  * @brief 		Copy arrayData from a arrayData.
	* @param		Data					An arry to store the copy data.
	* @param		Data_Sample		Data array want to copy.
  */
void CAN_Data_Copy(uint8_t *Data, uint8_t *Data_Sample)
{
	Data[0] = Data_Sample[0];
	Data[1] = Data_Sample[1];
	Data[2] = Data_Sample[2];
	Data[3] = Data_Sample[3];
	Data[4] = Data_Sample[4];
	Data[5] = Data_Sample[5];
	Data[6] = Data_Sample[6];
	Data[7] = Data_Sample[7];
}

/** @brief    Basic CANbus function for creating queue
  ==============================================================================
										##### CANbus Queue Functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Creating CAN_Queue for Transmiting and Receiving data.
    (+) Checking CAN_Queue Status.
    (+) Enqueue and Dequeue.
  */

/** @brief    Basic CANbus function for creating Transmition queue
  ------------------------------------------------------------------------------
							##### CANbus Transmition Queue Functions #####
  ------------------------------------------------------------------------------
  */

/**
  * @brief 		Creating CAN Tx Queue.
	* @param 		queue			A TxQueue
	* @param		capacity	Queue Size.
  */
void CAN_Create_TxQueue(CAN_TxQueue *queue, uint8_t capacity)
{
  queue->front = 0;
  queue->rear = -1;
  queue->used = 0;
  queue->capacity = capacity; 
	queue->TxMessage = (CAN_TxMessage *)malloc(capacity * sizeof(CAN_TxMessage));
}

/**
  * @brief 		Check a TxQueue has been created.
	* @param 		queue			A TxQueue
  */
void CAN_If_TxQueue_notCreate(CAN_TxQueue *queue)
{
	if (!queue->is_create)
	{
		CAN_Create_TxQueue(queue, CAN_QUEUE_CAPACITY);
		queue->is_create = 1;
	}
}

/**
  * @brief 		Check a TxQueue is empty.
	* @param 		queue			A TxQueue
	* @return		Empty (1) or not Empty (0)
  */
uint8_t CAN_TxQueue_isEmpty(CAN_TxQueue *queue)
{
  return (queue->used == 0);
}

/**
  * @brief 		Check a TxQueue is full.
	* @param 		queue			A TxQueue
	* @return		Full (1) or not Full (0)
  */
uint8_t CAN_TxQueue_isFull(CAN_TxQueue *queue)
{
  return (queue->used == queue->capacity);
}

/**
  * @brief 		Get Front data from a TxQueue.
	* @param 		queue			A TxQueue
	*	@return		A Tx_Mesage
  */
CAN_TxMessage CAN_TxQueue_getFront(CAN_TxQueue *queue)
{
	return queue->TxMessage[queue->front];
}

/**
  * @brief 		Get Rear data from a TxQueue.
	* @param 		queue			A TxQueue
	*	@return		A Tx_Mesage
  */
CAN_TxMessage CAN_TxQueue_getRear(CAN_TxQueue *queue)
{
	return (queue->TxMessage[queue->rear]);
}

/**
  * @brief 		Enqueue TxQueue.
	* @param 		queue			A TxQueue
	*	@param 		item	a can tx messgage data
  */
int CAN_EnTxQueue(CAN_TxQueue *queue, CAN_TxMessage item)
{
  if (CAN_TxQueue_isFull(queue))
    return -1;
  queue->rear = (queue->rear + 1)%queue->capacity;
  queue->TxMessage[queue->rear] = item;
  queue->used++;
  return 0;
}

/**
  * @brief 		Dequeue TxQueue.
	* @param 		queue			A TxQueue
  */
int CAN_DeTxQueue(CAN_TxQueue *queue)
{
  if (CAN_TxQueue_isEmpty(queue))
    return -1;
	queue->front = (queue->front + 1)%queue->capacity;
	queue->used--;
	return 0;
}

/** @brief    Basic CANbus function for creating Receiving queue
  ------------------------------------------------------------------------------
							##### CANbus Receiving Queue Functions #####
  ------------------------------------------------------------------------------
  */

/**
  * @brief 		Creating CAN Rx Queue.
	* @param 		queue			A RxQueue
	* @param		capacity	Queue Size.
  */
void CAN_Create_RxQueue(CAN_RxQueue *queue, uint8_t capacity)
{
  queue->front = 0;
  queue->rear = -1;
  queue->used = 0;
  queue->capacity = capacity; 
	queue->RxMessage = (CAN_RxMessage *)malloc(capacity * sizeof(CAN_RxMessage));
}

/**
  * @brief 		Check a RxQueue has been created.
	* @param 		queue			A RxQueue
  */
void CAN_If_RxQueue_notCreate(CAN_RxQueue *queue)
{
	if (!queue->is_create)
	{
		CAN_Create_RxQueue(queue, CAN_QUEUE_CAPACITY);
		queue->is_create = 1;
	}
}

/**
  * @brief 		Check a RxQueue is empty.
	* @param 		queue			A RxQueue
	* @return		Empty (1) or not Empty (0)
  */
uint8_t CAN_RxQueue_isEmpty(CAN_RxQueue *queue)
{
  return (queue->used == 0);
}

/**
  * @brief 		Check a RxQueue is full.
	* @param 		queue			A RxQueue
	* @return		Full (1) or not Full (0)
  */
uint8_t CAN_RxQueue_isFull(CAN_RxQueue *queue)
{
  return (queue->used == queue->capacity);
}

/**
  * @brief 		Get Front data from a RxQueue.
	* @param 		queue			A RxQueue
	*	@return		A Rx_Mesage
  */
CAN_RxMessage CAN_RxQueue_getFront(CAN_RxQueue *queue)
{
	return queue->RxMessage[queue->front];
}

/**
  * @brief 		Get Rear data from a RxQueue.
	* @param 		queue			A RxQueue
	*	@return		A Rx_Mesage
  */
CAN_RxMessage CAN_RxQueue_getRear(CAN_RxQueue *queue)
{
	return (queue->RxMessage[queue->rear]);
}

/**
  * @brief 		Enqueue RxQueue.
	* @param 		queue			A RxQueue
	*	@param 		item	a can rx messgage data
  */
int CAN_EnRxQueue(CAN_RxQueue *queue, CAN_RxMessage item)
{
  if (CAN_RxQueue_isFull(queue))
      return -1;
  queue->rear = (queue->rear + 1)%queue->capacity;
  queue->RxMessage[queue->rear] = item;
  queue->used++;
  return 0;
}

/**
  * @brief 		Dequeue RxQueue.
	* @param 		queue			A RxQueue
  */
int CAN_DeRxQueue(CAN_RxQueue *queue)
{
  if (CAN_RxQueue_isEmpty(queue))
      return -1;
	queue->front = (queue->front + 1)%queue->capacity;
	queue->used--;
	return 0;
}









