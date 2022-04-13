/*****************************************************************************
|  File Name: I2c_Soft.c
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
#include"I2C\I2c_Soft.h"
#include "SYSTEM\Sys_GenFun.h"

////////////////////////////////////////////////////////////////////////////////
void I2C_SOFT_Init(void)
////////////////////////////////////////////////////////////////////////////////
{
/*    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = I2C_SOFT_PORT_CLK_PIN ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(I2C_SOFT_PORT_CLK_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = I2C_SOFT_PORT_DAT_PIN ;
    GPIO_Init(I2C_SOFT_PORT_DAT_GPIO, &GPIO_InitStructure);
*/

    I2c_SoftClockSetOutput();
    I2c_SoftDatSetOutput();

    I2C_SOFT_Stop();
}
////////////////////////////////////////////////////////////////////////////////
I2c_Soft_ResultType I2C_SOFT_Start(void)
////////////////////////////////////////////////////////////////////////////////
{
    uint16 TimeOutCtr;
//    GPIO_InitTypeDef GPIO_InitStructure;

    I2c_SoftClockLow();
    I2C_SOFT_BIT_DELAY();
    I2c_SoftDathigh();
    I2c_SoftDatSetInput();
    TimeOutCtr = I2C_SOFT_TIMEOUT_CNT;
    do
    {
    } while ((0 == I2c_SoftDatGetData()) && (0 < --TimeOutCtr));
    I2c_SoftDatSetOutput();
    I2c_SoftDathigh();
    if (0 < TimeOutCtr)
    {
        I2c_SoftClockhigh();
        I2c_SoftClockSetInput();
        TimeOutCtr = I2C_SOFT_TIMEOUT_CNT;
        do
        {
        } while ((0 == I2c_SoftClockGetData()) && (0 < --TimeOutCtr));
        I2c_SoftClockSetOutput();
        I2c_SoftClockhigh();
        if (0 < TimeOutCtr)
        {
            I2c_SoftDatLow();
            I2C_SOFT_BIT_DELAY();
            return I2C_SOFT_SUCCESS;
        }
    }
    return I2C_SOFT_FAILED;
}

////////////////////////////////////////////////////////////////////////////////
I2c_Soft_ResultType I2C_SOFT_Stop(void)
////////////////////////////////////////////////////////////////////////////////
{
    uint16 TimeOutCtr;
//    GPIO_InitTypeDef GPIO_InitStructure;

    I2c_SoftClockLow();
    I2C_SOFT_BIT_DELAY();
    I2c_SoftDatLow();
    I2C_SOFT_BIT_DELAY();
    I2c_SoftClockhigh();
    I2c_SoftClockSetInput();
    TimeOutCtr = I2C_SOFT_TIMEOUT_CNT;
    do
    {
    } while ((0 == I2c_SoftClockGetData()) && (0 < --TimeOutCtr));
    I2c_SoftClockSetOutput();
    I2c_SoftClockhigh();
    if (0 < TimeOutCtr)
    {
        I2c_SoftDathigh();
        I2C_SOFT_BIT_DELAY();
        I2c_SoftDatSetInput();
        TimeOutCtr = I2C_SOFT_TIMEOUT_CNT;
        do
        {
        } while ((0 == I2c_SoftDatGetData()) && (0 < --TimeOutCtr));
        I2c_SoftDatSetOutput();
        if (0 < TimeOutCtr)
        {
            return I2C_SOFT_SUCCESS;
        }
    }

    return I2C_SOFT_FAILED;
}

////////////////////////////////////////////////////////////////////////////////
I2c_Soft_ResultType I2C_SOFT_Byte_Rx(BYTE *pRxData, BYTE LastByte)
////////////////////////////////////////////////////////////////////////////////
{
    uint8 BitNum;
    BYTE RxDataLocal;
    uint16 TimeOutCtr;
//    GPIO_InitTypeDef GPIO_InitStructure;

    I2c_SoftClockhigh();
    I2c_SoftClockSetInput();
    TimeOutCtr = I2C_SOFT_TIMEOUT_CNT;
    do
    {
    } while ((0 == I2c_SoftClockGetData()) && (0 < --TimeOutCtr));
    I2c_SoftClockSetOutput();
    if (0 < TimeOutCtr)
    {
        I2c_SoftClockLow();
        I2C_SOFT_BIT_DELAY();
        I2c_SoftDathigh();
        I2c_SoftDatSetInput();

        RxDataLocal = (BYTE)0;
        BitNum = 7;
        do
        {
            I2c_SoftClockhigh();
            I2C_SOFT_BIT_DELAY();
            I2C_SOFT_BIT_DELAY();
            RxDataLocal |= ((I2c_SoftDatGetData()) << BitNum);
            I2c_SoftClockLow();
            I2C_SOFT_BIT_DELAY();
            I2C_SOFT_BIT_DELAY();
        } while (0 < BitNum--);
        I2c_SoftDatSetOutput();
        (I2C_SOFT_RX_NOT_LAST_BYTE == LastByte) ? I2c_SoftDatLow():I2c_SoftDathigh();
        I2C_SOFT_BIT_DELAY();
        I2c_SoftClockhigh();

        *pRxData = RxDataLocal;

        return I2C_SOFT_SUCCESS;
    }

    return I2C_SOFT_FAILED;
}

////////////////////////////////////////////////////////////////////////////////
I2c_Soft_ResultType I2C_SOFT_Byte_Tx(BYTE TxData)
////////////////////////////////////////////////////////////////////////////////
#if 1
{
    uint8 BitNum;
    uint16 TimeOutCtr;
    //GPIO_InitTypeDef GPIO_InitStructure;

    I2c_SoftClockLow();
    I2c_SoftDathigh();
    I2c_SoftDatSetInput();
    TimeOutCtr = I2C_SOFT_TIMEOUT_CNT;
    do
    {
    } while ((0 == I2c_SoftDatGetData()) && (0 < --TimeOutCtr));
    I2c_SoftDatSetOutput();
    if (0 < TimeOutCtr)
    {
        BitNum = 7;
        do
        {
            I2c_SoftClockLow();
            //SOFT_IIC_DELAY(1);
            if(0 == ((1 << BitNum) & TxData)) 
            {
                I2c_SoftDatLow();
            }
            else
            {
                I2c_SoftDathigh();
            }
            //SOFT_IIC_DELAY(1);
            I2C_SOFT_BIT_DELAY();
            I2c_SoftClockhigh();
//            I2C_SOFT_BIT_DELAY();
            I2C_SOFT_BIT_DELAY();
        } while (0 < BitNum--);
        I2c_SoftClockLow();
        I2c_SoftDathigh();
        I2c_SoftDatSetInput();
        I2C_SOFT_BIT_DELAY();
        I2c_SoftClockhigh();
        TimeOutCtr = 10;
              do
              {
                  I2C_SOFT_BIT_DELAY();
              } while ((1 == I2c_SoftDatGetData()) && (0 < --TimeOutCtr));
        if (0 == I2c_SoftDatGetData())
        {
            I2c_SoftDatSetOutput();
                     I2c_SoftDatLow();
            return I2C_SOFT_SUCCESS;
        }
        I2c_SoftDatSetOutput();
        I2c_SoftDatLow();
    }
    return I2C_SOFT_FAILED;
}
#else
{
    uint8 BitNum;
    uint16 TimeOutCtr;
    GPIO_InitTypeDef GPIO_InitStructure;

    I2c_SoftClockLow();
    I2C_SOFT_BIT_DELAY();
    I2c_SoftDathigh();
    I2c_SoftDatSetInput();
    TimeOutCtr = I2C_SOFT_TIMEOUT_CNT;
    do
    {
    } while ((0 == I2c_SoftDatGetData()) && (0 < --TimeOutCtr));
    I2c_SoftDatSetOutput();
    if (0 < TimeOutCtr)
    {
        BitNum = 7;
        do
        {
            I2c_SoftClockLow();
            I2C_SOFT_BIT_DELAY();
            (0 == (((BYTE)1 << BitNum) & TxData)) ?  I2c_SoftDatLow() :I2c_SoftDathigh();
            I2C_SOFT_BIT_DELAY();
            I2c_SoftClockhigh();
            I2C_SOFT_BIT_DELAY();
        } while (0 < BitNum--);
        I2c_SoftClockLow();
        I2C_SOFT_BIT_DELAY();
        I2c_SoftDathigh();
        I2c_SoftDatSetInput();
        I2c_SoftClockhigh();
              TimeOutCtr = 5;
              do
              {
                  I2C_SOFT_BIT_DELAY();
              } while ((1 == I2c_SoftDatGetData()) && (0 < --TimeOutCtr));
        if (0 == I2c_SoftDatGetData())
        {
            I2c_SoftDatSetOutput();
                     I2c_SoftDatLow();
            return I2C_SOFT_SUCCESS;
        }
        I2c_SoftDatSetOutput();
        I2c_SoftDatLow();
    }
    return I2C_SOFT_FAILED;
}
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
