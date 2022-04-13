/*****************************************************************************
|  File Name: Adc_Nrf.c
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

#include "adc\adc_Nrf.h"
#include "nrf_adc.h"


#if 0
/****************************************************************************************************
**Function:
   int8 PcAdc_NrfOpen(void *pData)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Adc_NrfOpen(void *pData)
{
	nrf_adc_config_t nrf_adc_config;

	//Vdd_PowerOn();
	nrf_gpio_cfg_input(ADC_PIN, NRF_GPIO_PIN_NOPULL); 
	nrf_adc_config.reference = ADC_CONFIG_REFSEL_VBG;
	nrf_adc_config.resolution= NRF_ADC_CONFIG_RES_10BIT;
	nrf_adc_config.scaling = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD;

	nrf_adc_configure(&nrf_adc_config);
}
/****************************************************************************************************
**Function:
   Std_ReturnType Adc_NrfRead(void *pData)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
Std_ReturnType Adc_NrfRead(void *pData)
{
	static int32 adcdata;
	adcdata =nrf_adc_convert_single(NRF_ADC_CONFIG_INPUT_4);
	
	adcdata = (float)adcdata*3* ADC_NRF_STEP;
	*((uint16*)pData) =adcdata;
	return E_OK;
}
/****************************************************************************************************
**Function:
   Std_ReturnType Adc_NrfRead(void *pData)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Adc_NrfClose(void *pData)
{

	//Vdd_PowerOff();
}
const Dr_OpertonType AdcNrfDrive =
{
	Adc_NrfOpen,Adc_NrfClose, Adc_NrfRead, NULL,NULL
};
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

