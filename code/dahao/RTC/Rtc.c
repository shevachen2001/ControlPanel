/*****************************************************************************
|  File Name: Rtc.c
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
| 2013-11 1.00    lxw     Initial version 
|****************************************************************************/

#include "RTC\Rtc.h"
#include"I2C\I2c_Soft.h"
#include "Access\Access.h"
#include "SYSTEM\sys.h"
#include "protocol\Proto_Analy.h"
#include "Access\LockKey.h"

uint8 Rtc_Wlarm_Enable;
/****************************************************************************************************
**Function:
Std_ReturnType Rtc_ReadNBytes(Rtc_CommType Cmd, BYTE *pDataBuf, uint8 NumBytes)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
Std_ReturnType Rtc_ReadNBytes(Rtc_CommType Cmd, BYTE *pDataBuf, uint8 NumBytes)
{
    if (0 < NumBytes)
    {
        if ((RTC_SUCCESS == I2C_SOFT_Start())&& (RTC_SUCCESS == I2C_SOFT_Byte_Tx(0xa2))
			&&(I2C_SOFT_Byte_Tx((uint8)Cmd)==RTC_SUCCESS))
        {
            I2C_SOFT_Stop();
		    if((I2C_SOFT_Start() == RTC_SUCCESS)&&(I2C_SOFT_Byte_Tx(0xa3) == RTC_SUCCESS))
		    {

	            while (1 < NumBytes--)
	            {
	                if (RTC_SUCCESS != I2C_SOFT_Byte_Rx(pDataBuf, I2C_SOFT_RX_NOT_LAST_BYTE))
	                {
	                    I2C_SOFT_Stop();
	                    return RTC_FAILED;
	                }
	                ++pDataBuf;
	            }
	            if (RTC_SUCCESS == I2C_SOFT_Byte_Rx(pDataBuf, I2C_SOFT_RX_LAST_BYTE))
	            {
	                I2C_SOFT_Stop();

	                return RTC_SUCCESS;
	            }
		    }
        }
        I2C_SOFT_Stop();
        return RTC_FAILED;
    }
    return RTC_SUCCESS;
}
/****************************************************************************************************
**Function:
Std_ReturnType Rtc_WriteNBytes(Rtc_CommType Cmd, BYTE *pDataBuf, uint16 NumBytes)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
Std_ReturnType Rtc_WriteNBytes(Rtc_CommType Cmd, BYTE *pDataBuf, uint16 NumBytes)
{
    if((RTC_SUCCESS == I2C_SOFT_Start())&& (RTC_SUCCESS == I2C_SOFT_Byte_Tx(0xa2))
		&&(I2C_SOFT_Byte_Tx( (BYTE)Cmd) == RTC_SUCCESS))
    {
        while (0 < NumBytes--)
        {
            if (RTC_SUCCESS != I2C_SOFT_Byte_Tx(*pDataBuf++))
            {
                I2C_SOFT_Stop();
                return RTC_FAILED;
            }
        }
        I2C_SOFT_Stop();
        return RTC_SUCCESS;
    }
    I2C_SOFT_Stop();
    return RTC_FAILED;
}
/****************************************************************************************************
**Function:
void Rtc_Init(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Rtc_Init(void)
{
    uint8 DataBuffer[2];
    I2C_SOFT_Init();
	
    DataBuffer[0] = 0x00;
	Rtc_WriteNBytes(0x00,  DataBuffer,  1 ); 	

}


/******************************************************************************
************************Function:
uint8 Main_GetWeekFromDay(unsigned char year,unsigned char month,unsigned 
char day) 
**Author: lxw
**Description:
**Input: 
**Output: 
*******************************************************************************
*********************/
// 计算2000～2099年任一天星期几
// year    : 00-99 
// month: 01-12 
// day     : 01-31 
uint8 Main_GetWeekFromDay(unsigned char year,unsigned char month,unsigned char day) 
{ 
    if( month == 1 || month == 2 )   
    { 
        month += 12; 
        if( year > 0 ) 
            year--; 
        else 
            year = 4; 
    } 
     // 返回星期几(星期一用1表示，而星期天用7表示)
    return 1+(( day + 2*month + 3*(month+1)/5 + year + year/4 ) %7); 
}


/****************************************************************************************************
**Function:
   void  Rtc_Set(Rtc_Type *pData)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
Std_ReturnType  Rtc_Set(Rtc_Type *pData)
{
    uint8 DataBuffer[8];
    Std_ReturnType status = RTC_FAILED;

    pData->DayOfWeek = Main_GetWeekFromDay(pData->Year,pData->Month,pData->DayOfMonth);

	DataBuffer[0] = (((pData->Second)/10)<<4) | ((pData->Second)%10);
	DataBuffer[1] = (((pData->Minute)/10)<<4) | ((pData->Minute)%10);
	DataBuffer[2] = (((pData->Hour)/10)<<4)  | ((pData->Hour)%10);
	DataBuffer[3] = (((pData->DayOfMonth)/10)<<4)   | ((pData->DayOfMonth)%10);
	DataBuffer[4] = (((pData->DayOfWeek)/10)<<4)  | ((pData->DayOfWeek)%10);
	DataBuffer[5] = (((pData->Month)/10)<<4) | ((pData->Month)%10);
	DataBuffer[6] = (((pData->Year)/10)<<4)  | ((pData->Year)%10);

	Rtc_WriteNBytes(0x02, DataBuffer,7);


    I2c_SoftClockSetOutput();
    I2c_SoftDatSetOutput();
    I2c_SoftClockhigh();
    I2c_SoftDathigh();
    memset(DataBuffer,0,sizeof(DataBuffer));

	status = RTC_SUCCESS;

	return status;
}
/****************************************************************************************************
**Function:
   void  Rtc_WlarmSet(Rtc_WlarmType *pData)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void  Rtc_WlarmSet(Rtc_WlarmType *pData)
{

}
/****************************************************************************************************
**Function:
   int Rtc_Open(void *pData)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Rtc_Open(void *pData)
{
    Rtc_Type Rtc_data;
    
    Rtc_Init();
    RtcDrive.read(&Rtc_data);
    if ( Sys_PataCfg.settimeBak == 0xffffffff 
    || 0 == Sys_PataCfg.settimeBak  )
    {
        Sys_PataCfg.settimeBak =  1632855680;
		Sys_StoreFlag = STD_TRUE;
    }

	if ( Sys_PataCfg.lockDSTFlag == 0xff )	
	{	  
	    if( summer_winter_time_check() == SUMMER_TIMER  )
	    {
	      Sys_PataCfg.lockDSTFlag = SUMMER_TIMER_FLAG;
	    }
		else
		{
	      Sys_PataCfg.lockDSTFlag = SUMMER_TIMER_FLAG;
		}
	  Sys_StoreFlag = STD_TRUE;
	}	

    if((Rtc_data.Year   < 21)    || (Rtc_data.Year>=64) ||   
    (Rtc_data.Month  > 12)   || (Rtc_data.Month==0) ||   
    (Rtc_data.DayOfMonth   > 31)   || (Rtc_data.DayOfMonth==0)  ||   
    (Rtc_data.Hour   >=24)   ||     
    (Rtc_data.Minute >=60)   ||   
    (Rtc_data.Second >=60)      
    )
    {
        localtime(Sys_PataCfg.settimeBak,&Rtc_data );
        Rtc_data.Month = Rtc_data.Month + 1;
        Rtc_Ioctl(RTC_CLOCKSET,&Rtc_data);		
        RtcLocalTimebak = 0;
        
        Vdd_PowerOn();
        //HC595_LedWriteData(0x0040);    // 7	
    }
    I2c_SoftClockSetOutput();
    I2c_SoftDatSetOutput();
    I2c_SoftClockhigh();
    I2c_SoftDathigh();
}
/****************************************************************************************************
**Function:
   int8 Rtc_Read(void *pData)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
Std_ReturnType Rtc_Read(void *pData)
{
    uint8 DataBuffer[8];
	Rtc_ReadNBytes( 0x02,DataBuffer,7);

	((Rtc_Type *)pData)->Year   =   (((0xff&DataBuffer[6])&0xF0)>>4)*10 + ((0xff&DataBuffer[6])&0x0F);
	((Rtc_Type *)pData)->Month  =   (((0x1f&DataBuffer[5])&0xF0)>>4)*10 + ((0x1f&DataBuffer[5])&0x0F);
	((Rtc_Type *)pData)->DayOfWeek =(((0x07&DataBuffer[4])&0xF0)>>4)*10 + ((0x07&DataBuffer[4])&0x0F);
	((Rtc_Type *)pData)->DayOfMonth=(((0x3f&DataBuffer[3])&0xF0)>>4)*10 + ((0x3f&DataBuffer[3])&0x0F);
	((Rtc_Type *)pData)->Hour   =   (((0x3f&DataBuffer[2])&0xF0)>>4)*10 + ((0x3f&DataBuffer[2])&0x0F);
	((Rtc_Type *)pData)->Minute =   (((0x7f&DataBuffer[1])&0xF0)>>4)*10 + ((0x7f&DataBuffer[1])&0x0F); 
	((Rtc_Type *)pData)->Second =   (((0x7f&DataBuffer[0])&0xF0)>>4)*10 + ((0x7f&DataBuffer[0])&0x0F); 

    I2c_SoftClockSetOutput();
    I2c_SoftDatSetOutput();
    I2c_SoftClockhigh();
    I2c_SoftDathigh();

    return E_OK;

}
/****************************************************************************************************
**Function:
   Std_ReturnType Rtc_Ioctl(Rtc_CmdType Cmd,void *pData)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
Std_ReturnType Rtc_Ioctl(uint8 Cmd,void *pData)
{
    Std_ReturnType Err = E_RTC_WRITE_ERR;

    if(Cmd < SETRTCCONTROLMAX)
    {
        switch(Cmd)
        {
            case RTC_CLOCKSET:
            {
                if(RTC_SUCCESS == Rtc_Set(pData))
                {
                    Err = E_OK;
                    RecordList_StorePara(0); 
                }
            }break;
            case RTC_WLARMSET:
            {
                Err = E_OK;
                Rtc_WlarmSet(pData);
            }break;
            default:break; 
        }
    }
    return Err;
}

const Dr_OpertonType RtcDrive=
{
    Rtc_Open,NULL, Rtc_Read, NULL,Rtc_Ioctl,
};

