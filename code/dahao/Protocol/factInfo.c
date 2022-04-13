/*****************************************************************************
|  File Name: FactInfo.c
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
#include"Protocol\factInfo.h"

//版本定义
uint8 const FactInfo_SoftVersion[]    ={0x01, 0x00};
uint8 const FactInfo_HwVersion[]     = {0x01, 0x00};

//设备制造商描述
uint8 const FactInfo_Fact[] = 
{
    'X', 0,
    'X', 0,
    'X', 0,
    'X', 0,
    'X', 0,
    'X', 0,
    'X', 0,
    'X', 0,
     0, 0
};
//设备描述
uint8 const FactInfo_DeviceInfo[] = 
{
    'X', 0,
    'X', 0,
    'X', 0,
    'X', 0,
    'X', 0,
    'X', 0,
     0, 0
};





