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
#include  "KEY\key.h"
#include  "SYSTEM\sys.h"
#include  "SYSTEM\Sys_GenFun.h"
#include  "KEY\TSMxx.h"
#include  "..\EVENT\Event.h"
#include  "HC595\HC595.h"
#include  "KEY\Touch.h"
#include  "Light\light.h"
#include  "Access\Access.h"
#include  "ble_dahao.h"

extern uint8 Access_KeyDatTimex64ms;
extern uint8 TouchMaxCnt64ms;

#if (KEY_EN == STD_TRUE)
static Key_StateType    KeyScanState;
static Key_DataType    KeyData;
static Key_FlagType     KeyFlag;

uint8 KeyScanStateX64Ms;
uint8 Key_TouchBuffIndex;
uint8_t keySet = 0;

const Eve_IndexType Key_ShortEventTable[]=
{
/*KEY_NO_RESULT*/(Eve_IndexType)KEY_EVNET_NULL,
/*KEY_0*/(Eve_IndexType)EVE_0_SHORT,
/*KEY_1*/(Eve_IndexType)EVE_1_SHORT,
/*KEY_2*/(Eve_IndexType)EVE_2_SHORT,
/*KEY_3*/(Eve_IndexType)EVE_3_SHORT,
/*KEY_4*/(Eve_IndexType)EVE_4_SHORT,
/*KEY_5*/(Eve_IndexType)EVE_5_SHORT,
/*KEY_6*/(Eve_IndexType)EVE_6_SHORT,
/*KEY_7*/(Eve_IndexType)EVE_7_SHORT,
/*KEY_8*/(Eve_IndexType)EVE_8_SHORT,
/*KEY_9*/(Eve_IndexType)EVE_9_SHORT,
/*KEY_CLEAR*/(Eve_IndexType)EVE_DOORBELL_SHORT,
/*KEY_ENTER*/(Eve_IndexType)EVE_ENTER_SHORT,
/*KEY_CFG*/(Eve_IndexType)EVE_CONFIG_SHORT,
};

const Eve_IndexType Key_LongEventTable[]=
{
/*KEY_NO_RESULT*/(Eve_IndexType)KEY_EVNET_NULL,
/*KEY_0*/(Eve_IndexType)KEY_EVNET_NULL,
/*KEY_1*/(Eve_IndexType)KEY_EVNET_NULL,
/*KEY_2*/(Eve_IndexType)KEY_EVNET_NULL,
/*KEY_3*/(Eve_IndexType)KEY_EVNET_NULL,
/*KEY_4*/(Eve_IndexType)KEY_EVNET_NULL,
/*KEY_5*/(Eve_IndexType)KEY_EVNET_NULL,
/*KEY_6*/(Eve_IndexType)KEY_EVNET_NULL,
/*KEY_7*/(Eve_IndexType)KEY_EVNET_NULL,
/*KEY_8*/(Eve_IndexType)KEY_EVNET_NULL,
/*KEY_9*/(Eve_IndexType)KEY_EVNET_NULL,
/*KEY_CLEAR*/(Eve_IndexType)KEY_EVNET_NULL,
/*KEY_ENTER*/(Eve_IndexType)EVE_ENTER_LONG,
/*KEY_CFG*/(Eve_IndexType)EVE_CONFIG_LONG,
};

static void readResetKey(void)
{
    // Check if reset key is pressed
    //
    if(0 == CfgKeyGetData())
    {
        while(!CfgKeyGetData()){
            // wait for key release
        }
        keySet++;
    }

    // Check if reset button is pressed atleast 5 times
    //
    if(keySet >= 5)
    {
        Access_Globle.DefaultTimes = 0;
        // Initiate reset operation
        //
        Sys_Parainit();
        Sys_StorePara();
        Access_BeepStart(BEEP_NORMAL, 3);

        // Clear reset button press count to 0
        //
        keySet = 0;
    }

}
/****************************************************************************************************
**Function:
   Key_DataType  Key_scan(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
Key_DataType  Key_scan(void)
{
    Key_DataType keyData = KEY_NO_RESULT;
#if (defined TOUCH_ENABLE) && (TOUCH_ENABLE == STD_TRUE)
    keyData = Tsmxx_ReadIndex();
#endif
    if(keyData == KEY_NO_RESULT)
    {
        // Check if reset button is enabled
        //
        if(Sys_ConfigResetButton())
        {
            // Check event on reset key
            //
            readResetKey();
        }
    }

    return keyData;
}
 /****************************************************************************************************
**Function:
   Std_JudgeType  Key_LongKeyEnable(Key_DataType temp)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
Std_JudgeType  Key_LongKeyEnable(Key_DataType TempKey)
{
    Std_JudgeType State;

    State = STD_FALSE;
    if((TempKey == KEY_ENTER) || (TempKey == KEY_CFG))
    {
        State = STD_TRUE;
    }
    return(State);
}
/****************************************************************************************************
 **Function:
Std_ReturnType    Key_ShortKeyQuickPress(Key_DataType temp)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
Std_ReturnType  Key_ShortQKeyEnable(Key_DataType TempKey)
{
    Std_JudgeType State;
    State = STD_FALSE;

    return(State);
}

uint16 keylongcnt;
 /****************************************************************************************************
 **Function:
    void  Key_Main(void)
 **Author: lxw
 **Description:////////
 **Input: 
 **Output: 
 ****************************************************************************************************/
 void  Key_IoCtlScan(void)
 {
     Key_DataType tempkey;
     Eve_IndexType Tempevent;
     if ( TouchMaxCnt64ms )
     {
         TouchMaxCnt64ms--;
         return;
     }
     if(Access_KeyDatTimex64ms)
     {
         Access_KeyDatTimex64ms--;
         if(Access_KeyDatTimex64ms == 0)
         {
//             Touch_Init();
//             Tsm_Init();
         }
         return;
     }
     if(KeyScanStateX64Ms )
     {
         KeyScanStateX64Ms--;
     }
     tempkey = Key_scan();
     switch(KeyScanState )
     {
         case KEY_IDLE:
         {
             if(tempkey != KEY_NO_RESULT)
             {
            //     KeyScanStateX64Ms = 0;//64/64;
                 KeyScanState = KEY_PRESS_ONCE;
                 KeyData = tempkey;
             }
         }return;
         case KEY_PRESS_ONCE:
         {
             if(KeyData != tempkey)
             {
                 KeyScanState = KEY_IDLE;
             }
            // else if(KeyScanStateX64Ms == 0)
             {     
                  /*if(Sys_EnableBlueTooth())
                 {
                        ble_dahao_start_advert();
                 }*/
                #if 1
                 if(Access_Globle.ErrorTimes >= 5)
                {
#ifdef ENABLE_CONSOLE_LOG
                    printf("OpErr2\n\r");
#endif // ENABLE_CONSOLE_LOG
                    Access_OpenError();
                    KeyScanState = KEY_WAIT_SHORT_RELEASE;
                    return;//连续错误5次锁住,锁住不能再输入密码
                }
                 #endif
                 KeyScanStateX64Ms = 2000/64;
                 KeyScanState = KEY_PRESSING;
                 if(Key_LongKeyEnable(KeyData) == STD_TRUE)
                 {
                     KeyScanStateX64Ms = 2000/64;
                 }
                 else
                 {
                    KeyScanState = KEY_WAIT_SHORT_RELEASE;
                    Tempevent = Key_ShortEventTable[KeyData];
                    if( Tempevent != KEY_EVNET_NULL)
                    {
                        Event_Set(Tempevent);
                        if(KeyData != KEY_CFG)
                        {
                            Access_LightStart(LIGHT_KEY_LIGHT,LIGHT_VERY_FAST,2,TouchLightStable[KeyData-1]&(0x1FFF));
                        }
                        break;
                    }
                 }
             } 
         }return;
         case KEY_PRESSING:
         {
             if((KeyScanStateX64Ms == 0) && (KeyData == tempkey))
             {
                KeyScanState = KEY_WAIT_SHORT_RELEASE;                  
                Tempevent = Key_LongEventTable[KeyData];
                if( Tempevent != KEY_EVNET_NULL)
                {
                    Event_Set(Tempevent);
                    break;
                }
                 break;
             }
             else if(tempkey == KEY_NO_RESULT)
             {
                KeyScanState = KEY_IDLE;
                Tempevent = Key_ShortEventTable[KeyData];
                KeyFlag = KEY_SHORT;
                Event_Set(Tempevent);
                if(KeyData != KEY_CFG)
                {
                    Access_LightStart(LIGHT_KEY_LIGHT,LIGHT_VERY_FAST,2,TouchLightStable[KeyData-1]&(0x1FFF));
                }
                break;
             }
         }return;
         case KEY_WAIT_SHORT_RELEASE:
         {
              if ( ++keylongcnt >= 8000/64 )
              {
                  Touch_Init();
                  Tsm_Init();
                  keylongcnt = 0;
                  KeyScanState = KEY_IDLE;
              }
              if((KeyData == tempkey) && (KeyScanStateX64Ms == 0))
             {
                 KeyScanStateX64Ms = 200/64;
                 if(Key_ShortQKeyEnable(tempkey) == STD_TRUE)
                 {
                     Tempevent = Key_ShortEventTable[tempkey];
                     if( Tempevent != KEY_EVNET_NULL)
                     {
                         Event_Set(Tempevent);
                         if(KeyData != KEY_CFG)
                        {
                             Access_LightStart(LIGHT_KEY_LIGHT,LIGHT_VERY_FAST,2,TouchLightStable[KeyData-1]&(0x1FFF));
                         }
                         break;
                     }
                 }
            }
            else if(tempkey != KeyData)
            {
                 keylongcnt = 0;
                 KeyScanState = KEY_IDLE;
            }
         }return;
         default : KeyScanState = KEY_IDLE;return;     
     }
 }
/****************************************************************************************************
**Function:
   Std_ReturnType Key_Open(void *pData)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
 void Key_Open(void *pData)
{
    nrf_gpio_cfg_input(CFG_KEY_PIN,NRF_GPIO_PIN_PULLUP);
    //HC595_Init();
    Touch_Init();
}
/****************************************************************************************************
**Function:
   Std_ReturnType Key_Close(void *pData)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
 void Key_Close(void *pData)
{
    I2c_SoftClockSetOutput();
    I2c_SoftDatSetOutput();
    I2c_SoftClockhigh();
    I2c_SoftDathigh();
    Touch_I2cEnHigh();
//    Hc595_PowerOff();
}
/****************************************************************************************************
**Function:
   Std_ReturnType Key_Open(void *pData)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
Std_ReturnType Key_Read(void *pData)
{
    uint8 tempkey;
    if( tempkey != 0)
    {
        ((Key_ResultKeyType*)pData)->KeyType = KeyFlag;
        ((Key_ResultKeyType*)pData)->Keydata = KeyData;
        KeyFlag = KEY_STATE_NULL;
        return E_OK;
    }
    else 
    {
        return E_NOT_OK;
    }
}
/****************************************************************************************************
**Function:
   Std_ReturnType Key_IoCtl(Key_CmdType Cmd,void *pData)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
Std_ReturnType Key_IoCtl(uint8 Cmd,void *pData)
{
    if(Cmd >= KEY_CMD_MAX)
    {
        return E_INPAR_ERR;
    }
    else 
    {
        switch(Cmd)
        {
            case KEY_SCAN:
            {
                 Key_IoCtlScan();
            }break;
            case BEEP_CMD_GET_STATE:
            {
                if(KeyScanState == KEY_IDLE)
                {
                    return E_OK;
                }
                return E_NOT_OK;
            }
            default: break;
         }
    }
      return E_OK;
}

const Dr_OpertonType KeyDrive =
{
    Key_Open,Key_Close, Key_Read,NULL,Key_IoCtl
};
#if (KEY_CHECK == STD_TRUE)
/*******************************************************************************
* Function Name  : 
void Can_Check(void)
* Description    : 
*                  reset values.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/

void Key_Check(void)
{
    static uint16 count;
    static uint16 testbuff1[100];
    static uint16 testbuff2[100];
    uint8 index;
    index =0;
    KeyDrive.open(NULL);
    for(;;)
    {
        Sys_GenFunDelayMs(16);
        
    }
}
#endif

#endif 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
