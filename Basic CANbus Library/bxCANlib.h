/**
  ******************************************************************************
  * @file    	bxCANlib.h
  * @author  	Nguyen Vu
  * @brief   	This file contains all the functions prototypes 
	*						for the basic CANbus protocol
  *****************************************************************************/
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BXCANLIB_H_
#define BXCANLIB_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "CANConfig.h"

/**
  * @brief  TxMessage struct
  */
typedef struct
{
	CAN_TxHeaderTypeDef TxHeader;
	uint8_t 						txdata[8];
}CAN_TxMessage;

/**
  * @brief  RxMessage struct
  */
typedef struct
{
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t 						rxdata[8];
}CAN_RxMessage;

/**
  * @brief  TxQueue struct
  */
typedef struct
{
  int8_t 				front;
  int8_t 				rear;
  uint8_t 			used;
  uint8_t 			capacity;
	uint8_t 			is_create;
	CAN_TxMessage	*TxMessage;
}CAN_TxQueue;

/**
  * @brief  RxQueue struct
  */
typedef struct
{
  int8_t 				front;
  int8_t 				rear;
  uint8_t 			used;
  uint8_t 			capacity;
	uint8_t 			is_create;
	CAN_RxMessage	*RxMessage;
}CAN_RxQueue;

/* Initialization and basic support functions  ********************************/
void CAN_TxHeader_Init(CAN_TxHeaderTypeDef *TxHeader, uint32_t StdId, uint32_t DLC);
uint32_t get_Empty_Mailbox(void);
void CAN_Fifo0_Filter_Config(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *canfilter, uint32_t FilterBank, 
																uint32_t Filter_Id, uint32_t Filter_Id_Mask);


/* Copy data functions  *******************************************************/
void CAN_TxHeader_Copy(CAN_TxHeaderTypeDef *TxHeader, CAN_TxHeaderTypeDef TxHeader_Sample);
void CAN_RxHeader_Copy(CAN_RxHeaderTypeDef *RxHeader, CAN_RxHeaderTypeDef RxHeader_Sample);
void CAN_Data_Copy(uint8_t *Data, uint8_t *Data_Sample);


/* Creating, enqueue and dequeue CAN_TxQueue functions  ***********************/
void CAN_Create_TxQueue(CAN_TxQueue *queue, uint8_t capacity);
void CAN_If_TxQueue_notCreate(CAN_TxQueue *queue);

uint8_t CAN_TxQueue_isEmpty(CAN_TxQueue *queue);
uint8_t CAN_TxQueue_isFull(CAN_TxQueue *queue);

CAN_TxMessage CAN_TxQueue_getFront(CAN_TxQueue *queue);
CAN_TxMessage CAN_TxQueue_getRear(CAN_TxQueue *queue);

int CAN_EnTxQueue(CAN_TxQueue *queue, CAN_TxMessage item);
int CAN_DeTxQueue(CAN_TxQueue *queue);


/* Creating, enqueue and dequeue CAN_RxQueue functions  ***********************/
void CAN_Create_RxQueue(CAN_RxQueue *queue, uint8_t capacity);
void CAN_If_RxQueue_notCreate(CAN_RxQueue *queue);

uint8_t CAN_RxQueue_isEmpty(CAN_RxQueue *queue);
uint8_t CAN_RxQueue_isFull(CAN_RxQueue *queue);

CAN_RxMessage CAN_RxQueue_getFront(CAN_RxQueue *queue);
CAN_RxMessage CAN_RxQueue_getRear(CAN_RxQueue *queue);

int CAN_EnRxQueue(CAN_RxQueue *queue, CAN_RxMessage item);
int CAN_DeRxQueue(CAN_RxQueue *queue);


#endif







