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
#include "Pwm\Pwm.h"
#include "app_util_platform.h"
#include "app_pwm.h"
#include "nrf_delay.h"
#include "System\Sys.h"

APP_PWM_INSTANCE(PWM2,1);                   // 使用TIMER1创建一个PWM实例
/****************************************************************************************************
**Function:
   Std_ReturnType Pwm_Open(void *pData)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
 void Pwm_Open(void *pData)
{ 
    nrf_gpio_cfg_output(BEEP_PIN);
}
/****************************************************************************************************
**Function:
   void PwmUser_SetPameterBuzz(Pwm_ParmType *msg)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void PwmUser_SetPameterBuzz(Pwm_ParmType *msg)
{
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(1000000/msg->Freq, BEEP_PIN);
    /* Switch the polarity of the second channel. */
    pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;
    //按照设置的参数初始化PWM，设置PWM的回调函数 */
 //   if(ready_flag == 1)
    {
//        app_pwm_disable(&PWM2);
        app_pwm_uninit(&PWM2);
    }
//    beepopenflag = 1;
//    ready_flag= 0;
    app_pwm_init(&PWM2,&pwm1_cfg,NULL);
    app_pwm_enable(&PWM2);
    app_pwm_channel_duty_set(&PWM2, 0, msg->duty);
//    while (app_pwm_channel_duty_set(&PWM2, 0, msg->duty) == NRF_ERROR_BUSY);
    //等待操作完成
//    while(!ready_flag);
//    Sys_GenFunDelayMs(100);
}
/****************************************************************************************************
**Function:
   void PwmUser_SetPameterBuzz(Pwm_ParmType *msg)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void PwmUser_SetPameter125K(Pwm_ParmType *msg)
{

}
/****************************************************************************************************
 **Function:
    Std_ReturnType Pwm_IoCtl(void *pData)
 **Author: lxw
 **Description:
 **Input: 
 **Output: 
 ****************************************************************************************************/
 Std_ReturnType Pwm_IoCtl(uint8 Cmd,void *pData)
 {
 //    GPIO_InitTypeDef GPIO_InitStructure;
     if((Cmd == PWM_BEEP)&&(((Pwm_ParmType*)pData)->Freq != 0))
    {
        PwmUser_SetPameterBuzz((Pwm_ParmType*)pData);
    }
    else if((Cmd == PWM_BEEP_STOP)||(((Pwm_ParmType*)pData)->Freq == 0))
   {
//        NRF_TIMER1->TASKS_STOP = 1;

        app_pwm_disable(&PWM2);
        Beep_OutPutLow();
    }
/*    else if(Cmd == PWM_125K)
    {
        GPIO_InitStructure.GPIO_Pin = PWM225K_PIN;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(PWM225K_GPIO, &GPIO_InitStructure);
        PwmUser_SetPameter125K((Pwm_ParmType*)pData);
    }    
    else if(Cmd == PWM_125K_STOP)
    {
         TIM_Cmd(TIM1, DISABLE);
         GPIO_InitStructure.GPIO_Pin = PWM225K_PIN;
         GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
         GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
         GPIO_Init(PWM225K_GPIO, &GPIO_InitStructure);
         Pwm125kLow();
    }*/
     return E_OK;
 }


const Dr_OpertonType PwmDrive =
{
    Pwm_Open,NULL, NULL, NULL, Pwm_IoCtl
};

void BuzzerOnce_2(unsigned int fre, unsigned char duty)
{
	 app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(1000000/fre, 5);
   pwm1_cfg.pin_polarity[1]  = APP_PWM_POLARITY_ACTIVE_HIGH;
	 app_pwm_uninit(&PWM2);
	 app_pwm_init(&PWM2, &pwm1_cfg, NULL);
	 app_pwm_enable(&PWM2);
	 app_pwm_channel_duty_set(&PWM2, 0, duty);
}


void StopBuzzer(void)
{
	 app_pwm_disable(&PWM2);
	 nrf_gpio_cfg_output(BeepIO);   
	 Beep_OUT_0();
}

void beepOnceTime(void)
{
   	unsigned int a=0;
	  unsigned char i=0;

    if(IS_SET(Sys_PataCfg.configMode, VOICEMODE)) return;

	  while(i<1)
		{
			  a++;
			  if(a==1)
					BuzzerOnce_2(1700, 70);
				else if(a>=64)
				{
           i++;
					 a=0;
					 StopBuzzer();
				}
				nrf_delay_ms(1);
		}
}

#if (PWM_CHECK == STD_TRUE)
/*******************************************************************************
* Function Name  : 
void Pwm_Check(void)
* Description    : 
*                  reset values.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Pwm_Check(void)
{
    Pwm_ParmType parmeter;
    
    parmeter.duty = 50;
    parmeter.Freq = 125000;
    PwmDrive.open(NULL);
    PwmDrive.ioctl(PWM_125K, &parmeter);
    for(;;)
    {    
        /*Beep_OutPutHigh();
        Sys_GenFunDelayMs(1);
        Beep_OutPutLow();*/
                Sys_GenFunDelayMs(100);
    }
}
#endif



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


