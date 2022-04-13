/*******************************************************************************************************************
|  File Name: Sys_NetComm.c
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
| 2013-11  1.00                  Initial version 
|******************************************************************************************************************/

#include "protocol\Proto_NetComm.h"
#include "COMMU\Lock_Commu.h"
#include "lora\radio.h"
#include "protocol\Proto_ComDef.h"

extern _Bool revertOperation;
extern void performRevertOperation(void);
extern uint8_t CommandId;

// ÍøÂç´«Êä²ã

/****************************************************************************************************
**Function:
    void Proto_NetCommSend(Proto_NetcommType CommType,uint8_t* pData, int16 Len) 
**Author: LXW
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Proto_NetCommSend(Proto_NetcommType CommType,uint8_t* pData, uint16 Len) 
{
    switch(CommType)
    {
        case CC2540:
        {        
            LockCommu_SendStart(pData, Len);
        }break;
#if (defined LORA_ENABLE) && (LORA_ENABLE == STD_TRUE)
        case LORA:
        {        
            Radio_ComType radiodat;
            radiodat.Lenth = Len;
            radiodat.pData= pData;
            RadioDrive.write(&radiodat);
        }break;
#endif
        default:break;
    }

    if(revertOperation)
    {
        revertOperation = false;
        if((COMD_UPDATE_PASSKEY == CommandId) &&
            (COMD_SET_LIST      == CommandId) &&
            (COMD_REMOVE_ID     == CommandId))
        {
            performRevertOperation();
        }
    }
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


