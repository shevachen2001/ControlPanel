/*******************************************************************************************************************
|  File Name: Proto_CommPack.h
|
|  Description:
|
|-----------------------------------------------------------------------------
|               C O P Y R I G H T
|-----------------------------------------------------------------------------
| Copyright (c) 2002-2012 by DAHAO TECHNOLOGY Co., Ltd.All rights reserved.
|
|               This software is copyright protected and proprietary 
|               to DAHAO TECHNOLOGY Co., Ltd. DAHAO TECHNOLOGY Co., Ltd 
|               grants to you only those rights as set out in the 
|               license conditions. All other rights remain with 
|               DAHAO TECHNOLOGY Co., Ltd.
|-----------------------------------------------------------------------------
|               A U T H O R   I D E N T I T Y
|-----------------------------------------------------------------------------
| Initials     Name                      Company
| --------     ---------------------     ------------------------------------------
|                          DAHAO TECHNOLOGY Co., Ltd
|-----------------------------------------------------------------------------
|               R E V I S I O N   H I S T O R Y
|-----------------------------------------------------------------------------
| Date                 Ver     Author     Description
| ---------------------------- ------------------------------------------------
| 2013-10-23  1.00                  Initial version 
|******************************************************************************************************************/

#ifndef MOAPNWK_H
#define MOAPNWK_H

#include"PUBLIC\Std_Types.h"
#include "protocol\Proto_NetComm.h"

/******************************************************************************
**********************Macro definitiont*******************************************
******************************************************************************/
#define PROTO_COMM_COMD_MACLENTH    4
#define PROTO_COMM_COMD_OFFSET        2
#define PROTO_COMM_COMD_LENTHADD    2
#define PROTO_COMM_COMD_CMDADD        3

#define PROTO_COMM_BRG_BLE_LENTH        60

#define PROTO_COMM_BRG_LENTH            60//255


#define      PROTO_COMMPACK_HOST_DAT_IN          0xAA55
#define      PROTO_COMMPACK_HOST_DAT_OUT          0x55AA

#define      PROTO_COMMPACK_SLAVE_DAT_IN          0x55AA
#define      PROTO_COMMPACK_SLAVE_DAT_OUT          0xAA55
#define      PROTO_COMMPACK_SLAVE_DAT_ACTIVE_IN  0xAA33


#define     PROTO_PUBLIC                        STD_TRUE

/******************************************************************************
**********************Type statement*******************************************
******************************************************************************/

typedef struct {
/*    Proto_NetcommType Tc_SourceAdd1;
    Proto_NetcommType Tc_SourceAdd2;
    Proto_NetcommType Tc_TargeAdd;*/
    
    Std_ReturnType (*PublicProcCallBack)(Proto_NetcommType netype,uint8 *pindat,uint16 lenth,uint8 *poutdat);
    Std_ReturnType (*MacProcCallBack)(Proto_NetcommType netype,uint8 *pindat,uint8 *poutdat);
    Std_ReturnType (*AppProcCallBack)(Proto_NetcommType netype,uint8 *pindat,uint8 *poutdat);
    void (*ProcNetUpdateCallBack)(Proto_NetcommType netype);
    void (*ProcEndCallBack)(Proto_NetcommType netype,uint8 cmd);
}Proto_PackfunCallBack;
/******************************************************************************
**********************Variable definition***************************************
******************************************************************************/
    

/******************************************************************************
**********************Function declared****************************************
******************************************************************************/
#if 0
extern     void Proto_CommUpSend(Proto_NetcommType netype,uint8_t* pData); 

extern  void Proto_GetCommUpSendBuf(uint8_t **pDataOut); 
extern  void Proto_CommUpSetCommType(Proto_NetcommType  commType); 
extern  uint16 Proto_CommUpRec(uint8_t **pDataIn,uint8_t **pDataOut ); 
extern  uint16 Proto_CommBtRec(uint8_t **pDataIn,uint8_t **pDataOut ) ;

extern  void Proto_GetCommDownSendTBuf(uint8_t **pDataOut); 
extern  uint16 Proto_CommSlaceRecH(uint8_t **pDataIn,uint8_t **pDataOut ); 

extern  uint16 Proto_CommTcpRec(uint8 **pDataIn,uint8 **pDataOut,uint8 *pOrig,uint16 templenth); 
extern  void Proto_GetCommRecBuf(uint8_t **pDataIn);
extern uint16 Proto_CommRec(uint8 **pDataIn,uint8 **pDataOut) ; 
extern void Proto_CommSend(Proto_NetcommType netype,uint8_t* pData); 
extern  void Proto_GetCommSendBuf(uint8 **pDataOut); 
extern  void Proto_GetCommTcpSendBuf(uint8 **pDataOut);
extern  void Proto_CommTcpSend(Proto_NetcommType netype,uint8_t* pData) ;
#endif

extern void Proto_CommHostUnPackRec(Proto_NetcommType nettype,uint8* pindat,uint16 lenth) ;
extern void Proto_CommPrackInit(Proto_PackfunCallBack *funcalback);
extern void Proto_CommSend(Proto_NetcommType nettype,uint8_t* pData); 
extern void Proto_CommHostPackRec(Proto_NetcommType nettype,uint8* pindat,uint16 lenth);
extern void Proto_UpdateLearnRecord(uint8* pKeyId,uint32 time ,uint8 type, uint8 action);
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

