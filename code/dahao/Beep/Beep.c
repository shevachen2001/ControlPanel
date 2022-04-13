/*****************************************************************************
|  File Name: Beep.c
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
 #include "Beep\Beep.h"
 #include "Pwm\Pwm.h"

extern void beepOnceTime(void);

 #if (BEEP_EN == STD_TRUE)
 Beep_BkComType Beep_BkCom;
 Beep_WksubParaType *pBeep_WksubPara;
#if 1
const Beep_WksubParaType Beep_WksubParaNoarma[]=
{
{250/64,500/64,70,2000},
{NULL,NULL,NULL,NULL},
};
#endif



const Beep_WksubParaType Beep_WksubParaVeryFast[]=
{
{64/64,200/64,70,2200},
{NULL,NULL,NULL,NULL},
};
 const Beep_WksubParaType Beep_WksubParaFast[]=
{
{128/64,200/64,70,2200},
{NULL,NULL,NULL,NULL},
};
 const Beep_WksubParaType Beep_WksubParaSlow[]=
{
{64/64,200/64,70,1700},
{NULL,NULL,NULL,NULL},

};
const Beep_WksubParaType Beep_WksubParaVerySlow[]=
{
{500/64,4000/64,70,2000},
{NULL,NULL,NULL,NULL},
};

const Beep_WksubParaType Beep_WksubParalow[]=
{ 
{64/64,200/64,70,1700},
{NULL,NULL,NULL,NULL},
};

const Beep_WksubParaType Beep_WksubParaAL1[]=
 {
{7000/64,3000/64,70,1000},
{NULL,NULL,NULL,NULL},
};

const Beep_WksubParaType Beep_WksubParaAL2[]=
    {
{7000/64,3000/64,70,2000},
{NULL,NULL,NULL,NULL},
};
const Beep_WksubParaType Beep_WksubParaAL3[]=
{
{7000/64,3000/64,60,3000},
{NULL,NULL,NULL,NULL},
};
 
const Beep_WksubParaType Beep_WksubParaOpenErr[]=
{
{200/64,0,70,2100},
{100/64,0,0,0},
{200/64,0,70,2100},
{100/64,0,0,0},
{200/64,0,70,2100},
//{2000/64,100/64,65,2000},
{NULL,NULL,NULL,NULL},
};

#if 0
const Beep_WksubParaType Beep_WksubParaOther[]=
{
#ifdef    LEZUBEEEP
/*    {200/64,0,70,2100},
    {100/64,0,0,0},
    {200/64,0,70,2100},
    {100/64,0,0,0},
    {200/64,0,70,2100},*/
    {1500/64,100/64,65,2000},
#else         
    {128/64,0,50,2200},
    {128/64,0,50,1400},
    {64/64,0,50,1500},
    {64/64,0,50,1600},
    {128/64,0,50,1500},
    {128/64,0,50,1300},
#endif
    {NULL,NULL,NULL,NULL},
    
};
#endif

 const Beep_WksubParaType Beep_WksubParaOther[]=
 {
#ifdef    LEZUBEEEP
 /*  {200/64,0,70,2100},
     {100/64,0,0,0},
     {200/64,0,70,2100},
     {100/64,0,0,0},
     {200/64,0,70,2100},*/
     {1500/64,100/64,65,2000},
#else         
     {128/64,0,50,2200},
     {128/64,0,50,1400},
     {64/64,0,50,1500},
     {64/64,0,50,1600},
     {128/64,0,50,1500},
     {128/64,0,50,1300},
#endif
     {NULL,NULL,NULL,NULL},
     
 };


 const Beep_WksubParaType Beep_WksubParaTest1[]=
 {
  {128/64,0,50,3000},
  {NULL,NULL,NULL,NULL},
 };
 const Beep_WksubParaType Beep_WksubParaTest2[]=
 {
    {128/64,0,50,3000},
    {128/64,0,0,0},
    {128/64,0,50,6000},
    {128/64,0,50,3000},
  {NULL,NULL,NULL,NULL},
 };
 const Beep_WksubParaType Beep_WksubParaTest3[]=
 {
    {128/64,0,50,3000},
    {128/64,0,0,0},
    {128/64,0,50,6000},    
    {128/64,0,0,0},
    {128/64,0,50,6000},
    {128/64,0,50,3000},
  {NULL,NULL,NULL,NULL},
 };

 const Beep_WkParaType Beep_WkParaTable[]=
 {
        {BEEP_NORMAL,          (Beep_WksubParaType *)Beep_WksubParaNoarma},
        {BEEP_VERY_FAST,    (Beep_WksubParaType *)Beep_WksubParaVeryFast},
        {BEEP_FAST,        (Beep_WksubParaType *)Beep_WksubParaFast},
        {BEEP_SLOW,        (Beep_WksubParaType *)Beep_WksubParaSlow},
        {BEEP_VERY_SLOW,    (Beep_WksubParaType *)Beep_WksubParaVerySlow},
        {BEEP_OPEN,        (Beep_WksubParaType *)Beep_WksubParaOther},
        {BEEP_LOW,            (Beep_WksubParaType *)Beep_WksubParalow},
        {BEEP_AL1,         (Beep_WksubParaType *)Beep_WksubParaAL1},
        {BEEP_AL2,         (Beep_WksubParaType *)Beep_WksubParaAL2},
        {BEEP_AL3,         (Beep_WksubParaType *)Beep_WksubParaAL3},
        {BEEP_OPEN_ERR,      (Beep_WksubParaType *)Beep_WksubParaOpenErr},

        
        {BEEP_TEST1,      (Beep_WksubParaType *)Beep_WksubParaTest1},
};

/****************************************************************************************************
 **Function:
   Std_ReturnType Beep_Open(void *pData)
 **Author: rory
 **Description:
 **Input: 
 **Output: 
 ****************************************************************************************************/
 void Beep_Open(void *pData)
{
    PwmDrive.open(NULL);
}
/****************************************************************************************************
 **Function:
   Std_ReturnType Beep_Open(void *pData)
 **Author: rory
 **Description:
 **Input: 
 **Output: 
 ****************************************************************************************************/
Std_ReturnType Beep_Write(void *pData)
{
    if((Beep_BkCom.beeptime != 0)&&(Beep_BkCom.state == BEEP_OPEN))
    {
        return E_NOT_OK;
    }
    Beep_BkCom.beepfg= STD_OFF;
    Beep_BkCom.beeptime = 0;
    if(((Beep_ComType*)pData)->state != BEEP_IDLE)
    {
        Beep_BkCom.beeptime = 1;
        Beep_BkCom.state = ((Beep_ComType*)pData)->state;
        Beep_BkCom.wkTimes = ((Beep_ComType*)pData)->wkTimes;
        pBeep_WksubPara = Beep_WkParaTable[((Beep_ComType*)pData)->state-1].WksubPara;
    }
    return E_OK;
}
/****************************************************************************************************
 **Function:
    Std_ReturnType Beep_Open(void *pData)
 **Author: rory
 **Description:
 **Input: 
 **Output: 
 ****************************************************************************************************/
 Std_ReturnType Beep_IoCtl(uint8 Cmd,void *pData)
{

    switch(Cmd)
    {
        case BEEP_CMD_SCAN:
        {
            if(Beep_BkCom.beeptime == 0)
            {
                break;
            }
            Beep_BkCom.beeptime--;
            if(Beep_BkCom.beeptime== 0)
            {
                if(Beep_BkCom.beepfg == STD_OFF)
                {
                    Pwm_ParmType pwmpara;
                    Beep_BkCom.beepfg = STD_ON;
                    Beep_BkCom.beeptime = pBeep_WksubPara->on_Timesx64ms;
                    pwmpara.duty = pBeep_WksubPara->Duty;
                    pwmpara.Freq= pBeep_WksubPara->Freq;
                    PwmDrive.ioctl(PWM_BEEP,&pwmpara);
                }
                else
                {
                    if(pBeep_WksubPara->off_Timesx64ms == 0)
                    {
                        pBeep_WksubPara++;
                        if((pBeep_WksubPara!= NULL)&&(pBeep_WksubPara->on_Timesx64ms != 0))
                        {
                            Pwm_ParmType pwmpara;
                            Beep_BkCom.beepfg = STD_ON;
                            Beep_BkCom.beeptime = pBeep_WksubPara->on_Timesx64ms;

                            pwmpara.duty = pBeep_WksubPara->Duty;
                            pwmpara.Freq= pBeep_WksubPara->Freq;
                            PwmDrive.ioctl(PWM_BEEP,&pwmpara);
                            break;
                        }
                        if(pBeep_WksubPara->off_Timesx64ms== NULL)
                        {
                            if((Beep_BkCom.wkTimes != 0Xff)&&(Beep_BkCom.wkTimes != 0))
                            {
                                Beep_BkCom.wkTimes--;
                            }
                            if(Beep_BkCom.wkTimes != 0)
                            {
                                Pwm_ParmType pwmpara;
                                pBeep_WksubPara = Beep_WkParaTable[Beep_BkCom.state-1].WksubPara;                            
                                Beep_BkCom.beepfg = STD_ON;
                                Beep_BkCom.beeptime = pBeep_WksubPara->on_Timesx64ms;
                                pwmpara.duty = pBeep_WksubPara->Duty;
                                pwmpara.Freq= pBeep_WksubPara->Freq;
                                PwmDrive.ioctl(PWM_BEEP,&pwmpara);
                                break;
                            }
                        }
                    }
                    Beep_BkCom.beepfg = STD_OFF;
                    if((Beep_BkCom.wkTimes != 0Xff)&&(Beep_BkCom.wkTimes != 0))
                    {
                        Beep_BkCom.wkTimes--;
                    }
                    if(Beep_BkCom.wkTimes  != 0)
                    {
                        Beep_BkCom.beeptime = pBeep_WksubPara->off_Timesx64ms;
                    }
                    PwmDrive.ioctl(PWM_BEEP_STOP, NULL);
                }
            }
        }break;
        case BEEP_CMD_GET_STATE:
        {
            if((Beep_BkCom.wkTimes == 0)&&(Beep_BkCom.beeptime == 0))
            {
                return E_OK;
            }
            return E_NOT_OK;
        }
        default:break;
    }
    return E_OK;
}
const Dr_OpertonType BeepDrive =
{
    Beep_Open,NULL, NULL, Beep_Write, Beep_IoCtl
};

void Beep_once(void)
{
    beepOnceTime();
}

#if (BEEP_CHECK == STD_TRUE)
/*******************************************************************************
* Function Name  : 
void Beep_Check(void)
* Description    : 
*                  reset values.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Beep_Check(void)
{
    Eep_ComType eepdata;
    uint8 Databuffer[10];
    
    eepdata.Add = 0;
    eepdata.Lenth = 5;
    eepdata.pData = Databuffer;
    EepDrive.open(NULL);
    Databuffer[0] = 1;
    Databuffer[1] = 2;
    Databuffer[2] = 3;
    Databuffer[3] = 4;
    Databuffer[4] = 5;
    eepdata.pData = Databuffer;
    EepDrive.write(&eepdata);
    Databuffer[0] = 0;
    Databuffer[1] = 0;
    Databuffer[2] = 0;
    Databuffer[3] = 0;
    Databuffer[4] = 0;
    for(;;)
    {
        
        
        eepdata.pData = &Databuffer[5];
        EepDrive.read(&eepdata);
        /*if (EEP_SUCCESS == Eep_StartComm())
        {
            Sys_GenFunDelayMs(200);
        }
        else
        {
            I2C_SOFT_Stop();
        }*/
        Sys_GenFunDelayMs(400);
    }
}
#endif

#endif 


/************************ (C) COPYRIGHT DahaoMicroelectronics *****END OF FILE****/


