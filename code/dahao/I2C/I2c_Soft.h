/*****************************************************************************
|  File Name: I2c_Soft.h
|
|  Description: Header File defined
|
|-----------------------------------------------------------------------------
|               C O P Y R I G H T
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
|               A U T H O R   I D E N T I T Y
|-----------------------------------------------------------------------------
| Initials     Name                      Company
| --------     ---------------------     -------------------------------------
| LXW          LXW               
|-----------------------------------------------------------------------------
|               R E V I S I O N   H I S T O R Y
|-----------------------------------------------------------------------------
| Date        Ver     Author  Description
| ---------   ----    ------  ------------------------------------------------
| 2013-10      LXW     Initial version 
|****************************************************************************/

#ifndef I2C_SOFT_H
#define I2C_SOFT_H

#include"PUBLIC\Std_Types.h"
/******************************************************************************
**********************Macro definition*******************************************
******************************************************************************/
#define I2C_SOFT_FAILED                    (I2c_Soft_ResultType)(0)
#define I2C_SOFT_SUCCESS                (I2c_Soft_ResultType)(1)
    
#define I2C_SOFT_RX_NOT_LAST_BYTE        (BYTE)(0)
#define I2C_SOFT_RX_LAST_BYTE            (BYTE)(1)

#define I2C_SOFT_BIT_DELAY()             Sys_GenFunDelayUs(10) 
#define I2C_SOFT_TIMEOUT_CNT            (uint16)(6000)            /*    wait 4.4ms*/

#define I2C_SOFT_PORT_CLK_BIT_POS        (BYTE)(1)
#define I2C_SOFT_PORT_CLK_BIT_MASK        (BYTE)(1 << (I2C_SOFT_PORT_CLK_BIT_POS))

#define I2C_SOFT_PORT_DAT_BIT_POS        (BYTE)(0)
#define I2C_SOFT_PORT_DAT_BIT_MASK        (BYTE)(1 << (I2C_SOFT_PORT_DAT_BIT_POS))


/******************************************************************************
**********************Type statement*******************************************
******************************************************************************/
typedef Std_ReturnType I2c_Soft_ResultType;

/******************************************************************************
**********************Variable definition*******************************************
******************************************************************************/



/******************************************************************************
**********************Function declared*******************************************
******************************************************************************/
extern void I2C_SOFT_Init(void);
extern I2c_Soft_ResultType I2C_SOFT_Start(void);
extern I2c_Soft_ResultType I2C_SOFT_Stop(void);
extern I2c_Soft_ResultType I2C_SOFT_Byte_Rx(BYTE *pRxData, BYTE LastByte);
extern I2c_Soft_ResultType I2C_SOFT_Byte_Tx(BYTE TxData);

#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


