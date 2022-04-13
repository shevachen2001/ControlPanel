#include"PUBLIC\Std_Types.h"
#include "HC595\HC595.h"
#include "nrf_drv_common.h"
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "Card\Rc522.h"

extern void Sys_SpiOpen(void);

extern const nrf_drv_spi_t spi;
/****************************************************************************************************
**Function:
   void HC595_Init(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void HC595_Init(void)
{
    nrf_gpio_cfg_output(HC595_LCLK);
    HC595_LedWriteData(0x1FFF);
}
/****************************************************************************************************
**Function:
   void HC595_LedWriteData(uint8 Data)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
/*    const uint16 TouchLightStable[]=
    {
        // 0  1 2 3   4 5 6   7 8 9   * 0 #
        // 0  1 2 3   4 5 6   7 8 9   10 11 12
        ~(0xF400),~(0xF001),~(0xF002),~(0xF004), ~(0xF008),~(0xF010),~(0xF020), ~(0xF040),~(0xF080),~(0xF100), ~(0xF200),~(0xF800),~(0xF000)
    };
*/
void HC595_LedWriteData(uint16 Data)
{
    Sys_SpiOpen();
    Hc595_Lclk_Low();
    Sys_GenFunDelayUs(1);
    Data = Sys_GenFunhtons(Data);
    nrf_drv_spi_transfer(&spi, (uint8*)&Data, 2, NULL, 0);
    Hc595_Lclk_High();
    Hc595_Lclk_Low();
    //Sys_SpiClose();
}


