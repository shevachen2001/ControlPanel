/*****************************************************************************
|  File Name: Key.c
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
| YJW          YJW                DAHAO TECHNOLOGY Co., Ltd
|-----------------------------------------------------------------------------
|               R E V I S I O N   H I S T O R Y
|-----------------------------------------------------------------------------
| Date        Ver     Author  Description
| ---------   ----    ------  ------------------------------------------------
| 2016-05   1.00    yjw     Initial version 
|****************************************************************************/

#define SJT8012_I2C_ADDRESS     0x13

/****************************************************************************/ 
/* Include file                                                             */                                                 
/****************************************************************************/
#include  "KEY\key.h"
#include  "SYSTEM\sys.h"
#include  "SYSTEM\Sys_GenFun.h"
#include  "I2C\I2c_Soft.h"
#include  "KEY\Touch.h"
#include  "Beep\Beep.h"
#include  "Access\Access.h"
#include  "protocol\Proto_Analy.h"
#include  "Access\LockKey.h"
#include  "Encpt\Encpt.h"
#include  "KEY\TSMxx.h"
#include  "HC595\HC595.h"
#include "Light\light.h"
#include "Encpt\Encpt.h"
#include "Flash\Flash.h"
#include "nrf_delay.h"
#include "BLE_DAHAO.h"

extern unsigned char Verify_offlineCodeFlag;
extern unsigned char Verify_offlineCodeValue[4];
extern unsigned char LogsPswTemp[];
extern _Bool RtcReadStatus;

extern uint8 sys_CheckGetDynamicHotel_V8(uint32 DeviceId, uint32 DevicePassword, uint8* pTouchKey, uint8  length);

/*static */uint8 TouchBuf[(TOUCH_KEY_BUF_MAX >> 1) + 1];


_Bool DeleteOfflineFlag = false;
uint8 TouchState;
uint8 TouchKeyIndex;
uint8 TouchTimer64ms=200;         // °´¼ü³¬Ê±
//uint8 TouchCfgTiems =0;
uint16 TouchRstTimes =0;
uint32 DeleteOfflineEndtime = 0;
const uint16 TouchLightStable[]=
{
    // 0  1 2 3   4 5 6   7 8 9   * 0 #
    //~(0xF800),~(0xF004),~(0xF002),~(0xF001),~(0xF010),~(0xF008),~(0xF100),~(0xF040),~(0xF020),~(0xF200),~(0xF080),~(0xF400),~(0xF400)
      ~(0x0001),~(0x0010),~(0x0008),~(0x0100),~(0x0020),~(0x0004),~(0x1000),~(0x0040),~(0x0002),~(0x0800),~(0x0080),~(0x0400),~(0x0400)
};
const uint16_t LightOffTable[] =
{
    (KEY_1 | KEY_2 | KEY_4 | KEY_7 | KEY_5 | KEY_3 | KEY_STAR | KEY_8 | KEY_6 | KEY_0 | KEY_9 | KEY_HASH),
    (KEY_1 | KEY_2 | KEY_4 | KEY_7 | KEY_5 | KEY_3 | KEY_STAR | KEY_8 | KEY_6 | KEY_0 | KEY_9),
    (KEY_1 | KEY_2 | KEY_4 | KEY_7 | KEY_5 | KEY_3 | KEY_STAR | KEY_8 | KEY_6),
    (KEY_1 | KEY_2 | KEY_4 | KEY_7 | KEY_5 | KEY_3),
    (KEY_1 | KEY_2 | KEY_4),
    (KEY_1),
     0
};
FactoryTestType FactoryAppTest;
uint8_t debugPasscodeFail = 2;
uint8_t TouchMaxCnt64ms;
#define SIZE_LED_OFF_SEQ_TABLE  (sizeof(LightOffTable) / sizeof(LightOffTable[0]))

//HC595_LedWriteData(0x0010|0x0008|0x0100|0x0004|0x0001|0x0002);

void Touch_Clear(void);
char Check_FactoryAppTset(void)
{
     if( FactoryAppTest.testFlag.FT_FobFlag==1 ||
         FactoryAppTest.testFlag.FT_MotorFlag==1 ||
         FactoryAppTest.testFlag.FT_KeyFlag==1
       )
     {
         return( true );
     }
     return( false );
}

char FactoryAppTest_DuplicateKeyValue(char keyValue)
{
   unsigned char i=0, temp=0;
    
   for(i=0; i<6; i++)
   {
       temp=FactoryAppTest.FactoryTestKeyIndex[i];
       if( (((temp&0xF0)>>4) == keyValue)  || (((temp&0x0F)>>0) == keyValue) )
       {
           return( true );
       }
   }    
   return(  false );
}	

void FactoryAppTest_ReadKey(char keyvalue)
{
   unsigned char  i=0, k=0, temp=0;
    
   if(FactoryAppTest_DuplicateKeyValue(keyvalue) == false)
   {
       for(i=0; (i<6)&&(k==0); i++)
       {
           temp = FactoryAppTest.FactoryTestKeyIndex[i];
           
           if( temp==0xFF )
           {
              FactoryAppTest.FactoryTestKeyIndex[i] = keyvalue; 
              FactoryAppTest.FactoryTestKeyIndex[i] <<= 4;
              FactoryAppTest.FactoryTestKeyIndex[i] |= 0x0F;  
              k = 1;                
           }
           else if( ((temp&0x0F)>>0)==0x0F )
           {
              FactoryAppTest.FactoryTestKeyIndex[i] &= 0xF0;
              FactoryAppTest.FactoryTestKeyIndex[i] |= keyvalue; 
              k = 1;
           }
       }
   }
}	


/****************************************************************************************************
**Function:
   void Touch_Init( void )
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Touch_Init( void )
{
    Tsmxx_Open();
    Touch_Clear();
    TouchTimer64ms = 0;
}
void Touch_Clear(void)
{
    TouchState = TOUCH_STATE_IDLE;
    TouchKeyIndex = 0;
    for(uint8 i=0; i<(TOUCH_KEY_BUF_MAX>>1); i++)
    {
        TouchBuf[i] = KEY_NULL;
    }
    //TouchCfgTiems = 0;
}

void Touch_TimerProc(void)
{
#if (defined TOUCH_ENABLE) && (TOUCH_ENABLE == STD_TRUE)
    if(TouchTimer64ms != 0)
    {
        TouchTimer64ms--;
        if(TouchTimer64ms == 0)
        {
            Event_Set(EVE_TOUCH_TIMEOUT);
            if(Sys_EnableBlueTooth())
            {
                ble_dahao_start_advert(1);
            }
        }
        if(Event_Get(EVE_ALARM_SHORT))
        {
            Event_Remove(EVE_ALARM_SHORT);
            Access_BeepStart(BEEP_FAST, 6);

            return;
        }
        if(Event_Get(EVE_MANUAL_SHORT))
        {
            Event_Remove(EVE_MANUAL_SHORT);
#ifdef ENABLE_CONSOLE_LOG
            printf("unlock EVE: 0x%x\n\r", EVE_MANUAL_SHORT);
#endif
            Access_Unlock();
        }
    }
#endif
}

uint8 initpsw[3] = {0x12,0x34,0x56};    //  {0x12,0x33,0x21};
uint8 initageing[5] = {0x98,0x76,0x54,0x32,0x10};    //  {0x12,0x33,0x21};
extern uint8     AgeingFlag;
extern uint32    Ageingcnt;

/****************************************************************************************************
**Function:
    void HomeLock_Unlock(void)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Touch_KeyProc(uint8* pTouchKey,uint8 lenth)
{
    uint8 ret = E_NOT_OK;
    _Bool logStatus = false;

    LogsPswTemp[0] = pTouchKey[0];  LogsPswTemp[1] = pTouchKey[1];
    LogsPswTemp[2] = pTouchKey[2];  LogsPswTemp[3] = pTouchKey[3];

#if (defined RTC_EN) && (RTC_EN == STD_TRUE)
    ProtoAnaly_UpdateTime();
#endif
    if(false == RtcReadStatus) debugPasscodeFail = 18;
#if (defined DEBUG_ENABLE) && (DEBUG_ENABLE == STD_TRUE)
    Sys_Debug(STD_OFF);
#endif
    if (0 == memcmp(initpsw,pTouchKey,3 )&& (Sys_PataCfg.Pswd == 0x12345678))
      {
        Access_Unlock();
#ifdef ENABLE_CONSOLE_LOG
        printf("Write Log1: %d\n\r", KEY_TYPE_PASSWORD);
#endif // ENABLE_CONSOLE_LOG
        // Log activity for access via pass-code
        //
        Access_WriteRecordFlash(initpsw, ProtoAnaly_RtcLocalTime, KEY_TYPE_PASSWORD, true);
        return;
      }
    if (0 == memcmp(initageing,pTouchKey,5 )&& (Sys_PataCfg.Pswd == 0x12345678))
      {
		AgeingFlag = 1;
		Ageingcnt = 0;
		Access_BeepStart(BEEP_OPEN,2);
		HC595_LedWriteData(0x1fff);
        return;
      }
	
    if(lenth>=4)
    {
#ifdef ENABLE_CONSOLE_LOG
        for(int i  = 0; i < lenth; i++)
        {
            printf("key[%d]: 0x%x\n\r", i, pTouchKey[i]);
        }
#endif // ENABLE_CONSOLE_LOG
        ret = LockKey_Check_PaswdKey(pTouchKey, lenth);
#ifdef ENABLE_CONSOLE_LOG
        printf("LockKey_Check_PaswdKey: %d\n\r", ret);
#endif // ENABLE_CONSOLE_LOG
    }
    if(ret == E_NOT_OK)
    {
        if(lenth>=8)
        {
            DeleteOfflineFlag = false;

            ret = sys_CheckGetDynamicDate(Sys_PataCfg.Mac, Sys_PataCfg.Pswd, pTouchKey, lenth);
            if(ret == E_NOT_OK)
            {
                ret = sys_CheckGetDynamicFlat(Sys_PataCfg.Mac, Sys_PataCfg.Pswd, pTouchKey, lenth);
            }
            if(ret == E_NOT_OK)
            {
                ret = sys_CheckGetDynamicHotel_V8(Sys_PataCfg.Mac, Sys_PataCfg.Pswd, pTouchKey, lenth);
            }
        }
    }
#if (defined DEBUG_ENABLE) && (DEBUG_ENABLE == STD_TRUE)
    Sys_Debug(STD_ON);
#endif
    if(ret == E_OK)
    {
        Access_Unlock();
        logStatus = true;
        debugPasscodeFail = 2;
    }
    else if(ret == E_END)
    {
        Access_BeepStart(BEEP_VERY_FAST,4);
#ifdef ENABLE_CONSOLE_LOG
        printf("K7\n\r");
#endif // ENABLE_CONSOLE_LOG
        Access_LightStart(LIGHT_NG,LIGHT_SLOW,1,0);
    }
    else if(ret == E_TIME_OUT)
    {
#ifdef ENABLE_CONSOLE_LOG
        printf("K1\n\r");
#endif // ENABLE_CONSOLE_LOG
        Access_LightStart(LIGHT_NG,LIGHT_SLOW,1,0);
        Access_BeepStart(BEEP_FAST,BEEP_ERROR_TIME_OUT);
    }
    else
    {
#ifdef ENABLE_CONSOLE_LOG
        printf("OpErr3\n\r");
#endif // ENABLE_CONSOLE_LOG
        Access_OpenError();
        Access_Globle.ErrorTimes++;
        if(Access_Globle.ErrorTimes >= 5)
        {
            Access_LockDeviceTimer64ms = 180000/64;
        }
    }
    if(true == Verify_offlineCodeFlag)
    {
        Verify_offlineCodeFlag = false;
        LogsPswTemp[0]=Verify_offlineCodeValue[0]; LogsPswTemp[1]=Verify_offlineCodeValue[1];
        LogsPswTemp[2]=Verify_offlineCodeValue[2]; LogsPswTemp[3]=Verify_offlineCodeValue[3];
    }
#ifdef ENABLE_CONSOLE_LOG
    printf("Write Log2: %d: Status: %s: length: %d\n\r", 
            KEY_TYPE_PASSWORD, logStatus ? "true" : "false", lenth);
#endif // ENABLE_CONSOLE_LOG
    // Fill lower nibble for odd digit code
    //
    // logStatus = false  open fail
    // logStatus = true   open sucess  
	if ( E_NOT_OK == ret )
	{
	    switch(lenth)
	    {
	        // For pass-code 12345 entered
	        // it should be logged as 12 34 5F in the activity log
	        //
	        case 5:
#ifdef ENABLE_CONSOLE_LOG
	            printf("B10\n\r");
#endif // ENABLE_CONSOLE_LOG
	            LogsPswTemp[2] |= 0x0F;
	            break;
	        case 7:
	        // For pass-code 1234567 entered
	        // it should be logged as 12 34 56 7F in the activity log
	        //
#ifdef ENABLE_CONSOLE_LOG
	            printf("B9\n\r");
#endif // ENABLE_CONSOLE_LOG
	            LogsPswTemp[3] |= 0x0F;
	            break;
	    }
	}


    // Log activity
    //
    Access_WriteRecordFlash(LogsPswTemp, ProtoAnaly_RtcLocalTime, debugPasscodeFail, logStatus);
    // Reset debug flag
    //
#ifdef ENABLE_CONSOLE_LOG
    printf("Debug Code: %u\n\r", debugPasscodeFail);
#endif // ENABLE_CONSOLE_LOG
    debugPasscodeFail = 2;
}

void KeyboardLedOff(void)
{
    uint32_t i = 0;

    for(i = 0; i < SIZE_LED_OFF_SEQ_TABLE; i++)
    {
        HC595_LedWriteData(LightOffTable[i]);
        nrf_delay_ms(100);
    }
}
uint8 TT;
void Touch_Proc(void)
{
    uint8 dat = 0xff;

#if ((defined TOUCH_WAKEUP_PANEL) && (TOUCH_WAKEUP_PANEL == STD_TRUE))
    if(TouchState == TOUCH_STATE_IDLE)
    {
        TouchTimer64ms = TOUCH_TIME_OUT_DATA;
        TouchState = TOUCH_STATE_INPUT;
        return;
    }
#endif
    if(Event_Get(EVE_TOUCH_TIMEOUT))
    {
        Event_Remove(EVE_TOUCH_TIMEOUT);
        Touch_Clear();
        KeyboardLedOff();
        return;
    }
    if(Event_Get(EVE_0_SHORT))
    {
        Event_Remove(EVE_0_SHORT);
        dat = 0;
    }
    else if(Event_Get(EVE_1_SHORT))
    {
        Event_Remove(EVE_1_SHORT);
        dat = 1;
    }
    else if(Event_Get(EVE_2_SHORT))
    {
        Event_Remove(EVE_2_SHORT);
        dat = 2;
    }
    else if(Event_Get(EVE_3_SHORT))
    {
        Event_Remove(EVE_3_SHORT);
        dat = 3;
    }
    else if(Event_Get(EVE_4_SHORT))
    {
        Event_Remove(EVE_4_SHORT);
        dat = 4;
    }
    else if(Event_Get(EVE_5_SHORT))
    {
        Event_Remove(EVE_5_SHORT);
        dat = 5;
    }
    else if(Event_Get(EVE_6_SHORT))
    {
        Event_Remove(EVE_6_SHORT);
        dat = 6;
    }
    else if(Event_Get(EVE_7_SHORT))
    {
        Event_Remove(EVE_7_SHORT);
        dat = 7;
    }
    else if(Event_Get(EVE_8_SHORT))
    {
        Event_Remove(EVE_8_SHORT);
        dat = 8;
    }
    else if(Event_Get(EVE_9_SHORT))
    {
        Event_Remove(EVE_9_SHORT);
        dat = 9;
    }
    if(dat != 0xff)
    {
        FactoryAppTest_ReadKey(dat);
#ifdef ENABLE_CONSOLE_LOG
        printf("dat: %d::Count: %d\n\r", dat, TouchKeyIndex);
#endif
        if(TouchKeyIndex >= TOUCH_KEY_BUF_MAX )
        {
            Access_OpenError();
			memset(TouchBuf, 0xff, sizeof(TouchBuf));
			TouchKeyIndex = 0;
			TouchMaxCnt64ms = 2500/64;
			if( Check_FactoryAppTset() == false )
			{
				Access_Globle.ErrorTimes++;
	            if(Access_Globle.ErrorTimes >= 5)
	            {
	                Access_LockDeviceTimer64ms = 180000/64;
	            }
			}
            
        }
        else
        {
            TouchTimer64ms = TOUCH_TIME_OUT_DATA;
            TouchBuf[(TouchKeyIndex>>1)]<<= 4;
            TouchBuf[(TouchKeyIndex>>1)] |= dat;
            TouchKeyIndex++;
            Access_BeepStart(BEEP_FAST,1);         
        }
		
        if(Sys_EnableBlueTooth())
        {
            if(TouchState == TOUCH_STATE_IDLE)
            {
                TouchState = TOUCH_STATE_INPUT;
                ble_dahao_start_advert(1);
            }
            else
            {
                ble_dahao_start_advert(0);
            }
        }
        return;
    }
    if(Event_Get(EVE_DOORBELL_SHORT))
    {
        if( Check_FactoryAppTset() == true )
        {
            FactoryAppTest_ReadKey(0x0A);
        }    
        Event_Remove(EVE_DOORBELL_SHORT);
        memset(TouchBuf,0xff,sizeof(TouchBuf));
        TouchKeyIndex = 0;
        Access_BeepStart(BEEP_FAST,1);
    }
    if(Event_Get(EVE_ENTER_SHORT))
    {
        Event_Remove(EVE_ENTER_SHORT);
        if( Check_FactoryAppTset() == true )
        {
            Access_BeepStart(BEEP_FAST,1);
            FactoryAppTest_ReadKey(0x0B);
            FactoryAppTest.testFlag.FT_KeyFinished = 1;
            return;
        }       
#ifdef ENABLE_CONSOLE_LOG
        printf("Enter key pressed\n\r");
#endif
        if(Sys_PataCfg.State == WORK_NORMALLY_OPEN)
        {
            Access_BeepStart(BEEP_LOW,BEEP_ERROR_EMERGENCY);
            return;
        }
        if(TouchKeyIndex <4)
        {
            Access_OpenError();
            memset(TouchBuf, 0xff, sizeof(TouchBuf));
            TouchKeyIndex = 0;
            TouchMaxCnt64ms = 2500/64;
			
            Access_Globle.ErrorTimes++;
            if(Access_Globle.ErrorTimes >= 5)
            {
                Access_LockDeviceTimer64ms = 180000/64;
            }
            
            return;
        }
        if(TouchKeyIndex&0x01)
        {
            TouchBuf[(TouchKeyIndex>>1)]<<= 4;
        }
        Touch_KeyProc(TouchBuf,TouchKeyIndex);
        memset(TouchBuf,0xff,sizeof(TouchBuf));
        TouchKeyIndex = 0;
    }
    else if(Event_Get(EVE_ENTER_LONG))
    {
        Event_Remove(EVE_ENTER_LONG);
        Access_Lock();
        return;
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
