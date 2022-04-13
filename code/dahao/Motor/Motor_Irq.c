
/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/

#include "Tsmxx.h"
#include  "SYSTEM\Sys_GenFun.h"
#include  "I2C\I2c_Soft.h"
#include  "KEY\key.h"
#include  "KEY\TSMxx.h"
#include "Access\Access.h"
#include "Light\light.h"
#include  "KEY\Touch.h"
#if 0
/****************************************************************************************************
**Function:
   void Motor_Irq_Init(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Motor_Irq_Init(void)
{
	nrf_gpio_cfg_input(DOOR_SW, GPIO_PIN_CNF_PULL_Disabled);// 配置按键BUTTON_0为输入  
	// 配置BUTTON_1管脚为DETECT信号输出，该句很重要，是CPU被GPIO唤醒的必要条件  
	nrf_gpio_cfg_sense_input(DOOR_SW, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_LOW);	 
	NRF_GPIOTE->INTENSET = 1<< 31;
	NVIC_SetPriority(GPIOTE_IRQn, 3);
	NVIC_ClearPendingIRQ(GPIOTE_IRQn);
	NVIC_EnableIRQ(GPIOTE_IRQn);
}
/****************************************************************************************************
**Function:
   void Tsmxx_Irq_Disable(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Motor_Irq_Disable(void)
{
	nrf_gpio_cfg_sense_input(DOOR_SW, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_LOW);	 
	NVIC_DisableIRQ(GPIOTE_IRQn);
}
#endif
