/*****************************************************************************
|  File Name: Sys_TimeBase.c
|
|  Description: 
|
|-----------------------------------------------------------------------------
|               C O P Y R I G H T
|-----------------------------------------------------------------------------
| Copyright (c) 2002-2012 by DAHAO TECHNOLOGY Co., Ltd. All rights reserved.
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
| --------     ---------------------     -------------------------------------
| LXW          LXW               DAHAO TECHNOLOGY Co., Ltd
|-----------------------------------------------------------------------------
|               R E V I S I O N   H I S T O R Y
|-----------------------------------------------------------------------------
| Date        Ver     Author  Description
| ---------   ----    ------  ------------------------------------------------
| 2013-10 1.00    lxw     Initial version 
|****************************************************************************/

/****************************************************************************/ 
/* Include file                                                             */                                                 
/****************************************************************************/
#include "SYSTEM\Sys_TimeBase.h"
#include "Key\Key.h"
#include "Access\Access.h"
#include  "KEY\Touch.h"
#include "Beep\Beep.h"
#include "protocol\Proto_Analy.h"
#include "Access\Access.h"
#include "ble_dahao.h"
#include "lora\radio.h"
#include "SYSTEM\sys.h"
#include "COMMU\Lock_Commu.h"
#include "Flash\Flash_Nrf.h"
#include "FACTORY\Factory.h"

static Std_Bit8Byte Sys_TimeBaseFlag;
static uint8        Sys_TimeBaseConut;
static uint8        Sys_TimeBaseBackUp;
Std_Bit8Byte        Sys_TimeBaseSys32msFlag;  


extern uint8  Access_ReadCardFLag ;

#define OS_CLOCK       32768
#define OS_TICK        1
#define frequency      1000//hz
#define timervalue     1//s

#define OS_TRV          ((uint32_t)(((double)OS_CLOCK*(double)OS_TICK)/frequency)-1)

extern void Lora_ReordSendProc(void);
/****************************************************************************************************
**Function:
  void Sys_SetWkUpTime (uint8 channel,uint32 wktime) 
**Author: lxw 
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Sys_SetWkUpTime (uint8 channel,uint32 wktime) 
{
    NRF_RTC1->CC[channel] = wktime;//
    switch(channel)
    {
        case 0:
        {
            NRF_RTC1->EVTENSET |= RTC_EVTENSET_COMPARE0_Msk;
            NRF_RTC1->INTENSET |= RTC_INTENCLR_COMPARE0_Msk;
        }break;
        case 1:        
        {
            NRF_RTC1->EVTENSET |= RTC_EVTENSET_COMPARE1_Msk;
            NRF_RTC1->INTENSET |= RTC_INTENCLR_COMPARE1_Msk;

        }break;
        case 2:        
        {
            NRF_RTC1->EVTENSET |= RTC_EVTENSET_COMPARE2_Msk;
            NRF_RTC1->INTENSET |= RTC_INTENCLR_COMPARE2_Msk;

        }break;
        case 3:        
        {
            NRF_RTC1->EVTENSET |= RTC_EVTENSET_COMPARE3_Msk;
            NRF_RTC1->INTENSET |= RTC_INTENCLR_COMPARE3_Msk;
        }break;
        default:break;
    }
}
#if 0
/****************************************************************************************************
**Function:
  void Sys_TimeBaseInit (void) 
**Author: lxw 
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Sys_TimeBaseInit (void) 
{   

    NRF_TIMER2->TASKS_STOP = 1;
    // Stop timer 
    NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;            //timerģʽ
    NRF_TIMER2->BITMODE = (TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos);         // ����32bit
    NRF_TIMER2->PRESCALER  = 8;//8;     //2^8    256��Ƶ�õ�62500hz��ʱ�ӣ���������ΪTIMER1��TIMER2�ļ��������ֻ����?6λ�����Լ��������ֵ��?5536,���Է�Ƶ֮���ʱ�Ӳ��ܳ���?553
    NRF_TIMER2->TASKS_CLEAR = 1; // Clear timer
    NRF_TIMER2->CC[0] = 2000;//4000;//62500/10;//;      //64ms
    NRF_TIMER2->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);  // taken from Nordic dev zone
    NRF_TIMER2->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
    //���timerģ��

    NRF_TIMER2->TASKS_START = 1;
    NVIC_SetPriority(TIMER2_IRQn, 3);
    NVIC_ClearPendingIRQ(TIMER2_IRQn);
    NVIC_EnableIRQ(TIMER2_IRQn);
}
#endif
/****************************************************************************************************
**Function:
    void Sys_TimeBaseX2ms(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Sys_TimeBaseX64ms(void)
{
    Sys_TimeBaseConut++; 
    Sys_TimeBaseFlag.byte = Sys_TimeBaseConut ^ Sys_TimeBaseBackUp;
    Sys_TimeBaseBackUp = Sys_TimeBaseConut;
}
/****************************************************************************************************
**Function:
    void Sys_TimeBaseX2msTask(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Sys_TimeBaseX64msTask(void)
{
     if(!SYS_TIMEBASE_64MS)
    {
        return;
    }
    SYS_TIMEBASE_SYSBASE_64MS = STD_TRUE;
    if(!SYS_TIMEBASE_128MS)
    {
        return;
    }
    SYS_TIMEBASE_SYSBASE_128MS = STD_TRUE;

}
/****************************************************************************************************
**Function:
    void Sys_TimeBaseX2msProc(void)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Sys_TimeBaseX64msProc(void)
{
//    Access_DetCardProc();
//    LockCommu_Proc();
    if(SYS_TIMEBASE_SYSBASE_64MS == STD_TRUE)
    {
#if (defined LORA_ENABLE) && (LORA_ENABLE == STD_TRUE)
//        Lora_ReordSendProc();
#endif
//        ble_dahao_timer_proc();
//        Access_TimerProc();
//        Touch_TimerProc();
//        Sys_RestProc();
    #if ((defined USE_FLASH_NRF) && (USE_FLASH_NRF == STD_TRUE))
    //    NrfFlashDrive.ioctl(NRFFLASH_SCAN, NULL);
    #endif
    //    Factory_Proc();
        SYS_TIMEBASE_SYSBASE_64MS = STD_FALSE;
    }
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
