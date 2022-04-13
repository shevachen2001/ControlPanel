/*****************************************************************************
|  File Name: Iop.c
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
| 2012-09 1.00    lxw     Initial version 
|****************************************************************************/

/****************************************************************************/ 
/* Include file                                                             */                                                 
/****************************************************************************/

#include "Light\light.h"
#include "HC595\HC595.h"

Light_BkComType Light_BkComBuf[LIGHT_NAME_MAX];


Std_Bit8Byte    TouchLightFlag;  

#define     TouchLight_Batt        TouchLightFlag.bits.bit0
#define     TouchLight_Conn        TouchLightFlag.bits.bit1
#define     TouchLight_Ok        TouchLightFlag.bits.bit3
#define     TouchLight_Ng        TouchLightFlag.bits.bit2


const Light_WkParaType Light_WkParaTable[]=
{
//wkmode,lighton time,lightoff time//
{LIGHT_NORMAL,        1000/64,1000/64},

{LIGHT_VERY_FAST,    256/64,6000/64},
{LIGHT_FAST,        500/64,500/64},
{LIGHT_SLOW,        3000/64,100/64},
{LIGHT_VERY_SLOW,    5000/64,100/64},


{LIGHT_OPEN,        1000/64,1000/64},
{LIGHT_T_ERR,        2000/64,2000/64},
{LIGHT_S_ERR,        500/64,500/64},
};
/****************************************************************************************************
**Function:
   void Light_BattOff(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Light_BattOff(void)
{
    HC595_LedWriteData(0x0000);
}
/****************************************************************************************************
**Function:
   void Light_LanOff(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Light_BattOn(void)
{
    //HC595_LedWriteData(0x0004|0x0010|0x0040|0x0080|0x0800|0x0400);
      HC595_LedWriteData(0x0010|0x0020|0x0040|0x0080|0x0001|0x0400);
}
/****************************************************************************************************
**Function:
   void Light_LanOff(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Light_BattTogg(void)
{
    //TouchLight_Batt = ~TouchLight_Batt;
    HC595_LedWriteData(Light_BkComBuf[LIGHT_KEY_LIGHT].Hc595dat|(((uint16)TouchLightFlag.byte)<<12));
}
/****************************************************************************************************
**Function:
   void Light_BattOff(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Light_ConnOff(void)
{
    HC595_LedWriteData(0x0000);
}
/****************************************************************************************************
**Function:
   void Light_LanOff(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Light_ConnOn(void)
{
    //HC595_LedWriteData(0x0004|0x0002|0x0001|0x0008|0x0800|0x0020);
      HC595_LedWriteData(0x0010|0x0008|0x0100|0x0004|0x0001|0x0002);
}
/****************************************************************************************************
**Function:
   void Light_LanOff(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Light_ConnTogg(void)
{
    //TouchLight_Conn = ~TouchLight_Conn;
    HC595_LedWriteData(Light_BkComBuf[LIGHT_KEY_LIGHT].Hc595dat|(((uint16)TouchLightFlag.byte)<<12));
}
/****************************************************************************************************
**Function:
   void Light_OkOff(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Light_OkOff(void)
{
    HC595_LedWriteData(0);
}
/****************************************************************************************************
**Function:
   void Light_OkOn(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Light_OkOn(void)
{
    HC595_LedWriteData(0x0400|0x8000|0x0200);  //  # green logo
}
/****************************************************************************************************
**Function:
   void Light_OkTogg(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Light_OkTogg(void)
{
    //TouchLight_Ok = ~TouchLight_Ok;
    HC595_LedWriteData(Light_BkComBuf[LIGHT_KEY_LIGHT].Hc595dat|(((uint16)TouchLightFlag.byte)<<12));
}
/****************************************************************************************************
**Function:
   void Light_NgOff(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Light_NgOff(void)
{
    HC595_LedWriteData(0x0000);
}
/****************************************************************************************************
**Function:
   void Light_OkOn(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Light_NgOn(void)
{
    HC595_LedWriteData(0x0004|0x0010|0x0040|0x0100|0x0800|0x2000|0x0200);  // red
}
/****************************************************************************************************
**Function:
   void Light_NgTogg(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Light_NgTogg(void)
{
    //TouchLight_Ng = ~TouchLight_Ng;
    HC595_LedWriteData(Light_BkComBuf[LIGHT_KEY_LIGHT].Hc595dat|(((uint16)TouchLightFlag.byte)<<12));
}

/****************************************************************************************************
**Function:
   void Light_CardOn(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Light_CardOn(void)
{
    Hc595_Lclk_High();  
}

/****************************************************************************************************
**Function:
   void Light_CardOn(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Light_CardOff(void)
{
    Hc595_Lclk_Low();  
}

/****************************************************************************************************
**Function:
   void Light_CardOn(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Light_CardTogg(void)
{
    Hc595_Lclk_Low();  
}

/****************************************************************************************************
**Function:
   void Light_NgOff(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Light_AllOff(void)
{    
    if(Light_BkComBuf[LIGHT_KEY_LIGHT].wkTimes > 1)
    {
        Light_BkComBuf[LIGHT_KEY_LIGHT].Hc595dat = 0x1fff;
        HC595_LedWriteData(Light_BkComBuf[LIGHT_KEY_LIGHT].Hc595dat|(((uint16)TouchLightFlag.byte)<<12));
    }
}
/****************************************************************************************************
**Function:
   void Light_OkOn(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Light_AllOn(void)
{
    if(Light_BkComBuf[LIGHT_KEY_LIGHT].wkTimes >= 2)
    {
        HC595_LedWriteData(Light_BkComBuf[LIGHT_KEY_LIGHT].Hc595dat|(((uint16)TouchLightFlag.byte)<<12));
    }
    else if(Light_BkComBuf[LIGHT_KEY_LIGHT].wkTimes >= 1)
    {
        Light_BkComBuf[LIGHT_KEY_LIGHT].Hc595dat = 0;
        HC595_LedWriteData(Light_BkComBuf[LIGHT_KEY_LIGHT].Hc595dat|(((uint16)TouchLightFlag.byte)<<12));
    }
}
/****************************************************************************************************
**Function:
   void Light_NgTogg(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Light_AllTogg(void)
{
    Light_BkComBuf[LIGHT_KEY_LIGHT].Hc595dat = (~Light_BkComBuf[LIGHT_KEY_LIGHT].Hc595dat)&0XFFFF;
    HC595_LedWriteData(Light_BkComBuf[LIGHT_KEY_LIGHT].Hc595dat|(((uint16)TouchLightFlag.byte)<<12));
}
const Light_CntfunType    Light_CntfunTable[]=
{
#if (defined TOUCH_ENABLE) && (TOUCH_ENABLE == STD_TRUE)
    {&Light_BattOn,&Light_BattOff,&Light_BattTogg}, /* Low battery */
    {&Light_ConnOn,&Light_ConnOff,&Light_ConnTogg},
    {&Light_OkOn,&Light_OkOff,&Light_OkTogg},
    {&Light_NgOn,&Light_NgOff,&Light_NgTogg},
#else
    {&Light_CardOn,&Light_CardOff,&Light_CardTogg},
    {&Light_CardOn,&Light_CardOff,&Light_CardTogg},
    {&Light_CardOn,&Light_CardOff,&Light_CardTogg},
    {&Light_CardOn,&Light_CardOff,&Light_CardTogg},
#endif
    {&Light_AllOn,&Light_AllOff,&Light_AllTogg},
};
/****************************************************************************************************
**Function:
   int8 Light_Open(void *pData)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Light_Open(void *pData)
{
    HC595_Init();
    memset(&Light_BkComBuf,0,sizeof(Light_BkComBuf));
}
/****************************************************************************************************
**Function:
    Std_ReturnType Light_Write(uint8 Cmd,void *pData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
Std_ReturnType Light_Write(void *pData)
{
/*#if (defined KEY_BOARD) && (KEY_BOARD == KEY_LIGHT_13)

    if((((Light_ComType*)pData)->lightname < LIGHT_NAME_MAX)&&(((Light_ComType*)pData)->state < LIGHT_MODE_MAX))
    {
        Light_NameType name;

        name = ((Light_ComType*)pData)->lightname;
        if((name == LIGHT_CONN)||(name == LIGHT_OK)
            ||(name == LIGHT_NG))
        {
            name = LIGHT_BATT;
        }
        Light_BkComBuf[name].Hc595dat= ((Light_ComType*)pData)->Hc595dat;
        Light_BkComBuf[name].state = ((Light_ComType*)pData)->state;
        Light_BkComBuf[name].wkTimes = ((Light_ComType*)pData)->wkTimes;

        Light_BkComBuf[name].lighttime = 1;
        Light_BkComBuf[name].lightfg = STD_OFF;
        return E_OK;
    }
#elif(defined KEY_BOARD) && (KEY_BOARD == KEY_LIGHT_16)*/
    TouchLightFlag.byte = 0;
    memset(Light_BkComBuf,0,sizeof(Light_BkComBuf));
    if(((Light_ComType*)pData)->lightname < LIGHT_NAME_MAX)
    {
        Light_BkComBuf[((Light_ComType*)pData)->lightname].Hc595dat= ((Light_ComType*)pData)->Hc595dat;
        Light_BkComBuf[((Light_ComType*)pData)->lightname].state = ((Light_ComType*)pData)->state;
        Light_BkComBuf[((Light_ComType*)pData)->lightname].wkTimes = ((Light_ComType*)pData)->wkTimes;
        Light_BkComBuf[((Light_ComType*)pData)->lightname].lighttime = 1;
        Light_BkComBuf[((Light_ComType*)pData)->lightname].lightfg = STD_OFF;
        return E_OK;
    }
//#endif
    return E_NOT_OK;
}
/****************************************************************************************************
**Function:
    Std_ReturnType Light_IoCtl(uint8 Cmd,void *pData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
 Std_ReturnType Light_IoCtl(uint8 Cmd,void *pData)
 {
     switch(Cmd)
     {
        case LIGHT_CMD_SCAN:
        {
            uint8 i;
            for(i= 0;i< LIGHT_NAME_MAX;i++)
            {
                if(Light_BkComBuf[i].lighttime != 0)
                {
                    Light_BkComBuf[i].lighttime--;
                    if(Light_BkComBuf[i].lighttime == 0)
                    {
                        if(Light_BkComBuf[i].wkTimes != 0)
                        {
                            if(Light_BkComBuf[i].lightfg == STD_OFF)
                            {
                                Light_BkComBuf[i].lightfg = STD_ON;
                                Light_CntfunTable[i].lighton();
                                Light_BkComBuf[i].lighttime = Light_WkParaTable[Light_BkComBuf[i].state- 1].on_Timesx64ms;
                            }
                            else 
                            {
                                Light_BkComBuf[i].lightfg = STD_OFF;
                                Light_CntfunTable[i].lightoff();
                                if(Light_BkComBuf[i].wkTimes != 0Xff)
                                {
                                    Light_BkComBuf[i].wkTimes--;
                                }
                                if(Light_BkComBuf[i].wkTimes != 0)
                                {
                                    Light_BkComBuf[i].lighttime = Light_WkParaTable[Light_BkComBuf[i].state- 1].off_Timesx64ms;
                                }
                            }
                        }
                    }
                }
            }
        }break;
        case LIGHT_CMD_GET_STATE:
        {    
            uint8 i;
            for(i= 0;i< LIGHT_NAME_MAX;i++)
            {
                if((Light_BkComBuf[i].lighttime != 0)||(Light_BkComBuf[i].wkTimes != 0))
                {
                    return E_NOT_OK;
                }
            }
            return E_OK;
        }
        default:break;
    }
    return E_OK;
 }

const Dr_OpertonType LightDrive =
{
    Light_Open,NULL, NULL, Light_Write,Light_IoCtl
};

#if (LIGHT_CHECK == STD_TRUE)
/*******************************************************************************
* Function Name  : 
void Can_Check(void)
* Description    : 
*                  reset values.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Light_Check(void)
{
    LightDrive.open(NULL);
    for(;;)
    {
        /*iodata.IoSelet = IOP_DOOR_BELL;
        iodata.IoState = STD_LOW;
        IopDrive.write(&iodata);

        iodata.IoSelet = IOP_DOOR_BELL;
        iodata.IoState = STD_HIGH;
        IopDrive.write(&iodata);
        Sys_GenFunDelayMs(1000);*/
    }
}
#endif 



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
