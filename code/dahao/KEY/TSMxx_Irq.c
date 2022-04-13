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

#include"PUBLIC\Std_Types.h"


extern Std_ReturnType power_out_sleep(void);
/****************************************************************************************************
**Function:
   void Tsmxx_Irq_Init(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Tsmxx_Irq_Init(void)
{
    // 配置BUTTON_1管脚为DETECT信号输出，该句很重要，是CPU被GPIO唤醒的必要条件  
    nrf_gpio_cfg_sense_input(TOUCH_INTT_PIN, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_LOW);     
    NRF_GPIOTE->INTENSET = 1U << 31;
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
void Tsmxx_Irq_Disable(void)
{
    nrf_gpio_cfg_sense_input(TOUCH_INTT_PIN, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_LOW);     
    NVIC_DisableIRQ(GPIOTE_IRQn);
}

void Wiegand_Init(void) 
{
  nrf_gpio_cfg_output(WIEGAND_DATA0_PIN);
  nrf_gpio_cfg_output(WIEGAND_DATA1_PIN);
}

void WG_send_bit_1(void)
{
    //WG_DATA1 = 0;
    //----------------------延时100us
    //delay_100us();
    //WG_DATA1 = 1;
    //-------------------------------延时一个发送周期
    //delay_1500us(); 
    WIEGAND_DATA1Low(); 
    Sys_GenFunDelayUs(100);
    WIEGAND_DATA1High(); 
    Sys_GenFunDelayUs(1500); 
}
void WG_send_bit_0(void)
{
    //WG_DATA0 = 0;    
    //----------------------延时100us
    //delay_100us();
    //WG_DATA0 = 1;
    //-------------------------------延时一个发送周期
    //delay_1500us();    

    WIEGAND_DATA0Low(); 
    Sys_GenFunDelayUs(100);
    WIEGAND_DATA0High(); 
    Sys_GenFunDelayUs(1500);    
}

    unsigned char i;
    unsigned char check_temp; //韦根包奇偶效验中间暂存
    unsigned char even; //韦根包前12位偶效验
    unsigned char odd; //韦根包后12位齐效验
    unsigned char wiegand[3]; //韦根包数据24位

void Send_Weigand34(uint8_t *str)
{
    //| wiegand[0] | wiegand[1] | wiegand[2] |
    //| *str *(str + 1) | *(str + 2) *(str + 3)| *(str + 4) *(str + 5)|
 
 
    //--------------------------------端口方向定义
    Wiegand_Init();

    //================================数组到韦根包的转化
    wiegand[0] = wiegand[0]|((*str)<<4);//原理是把每个字节的低4位取出，来计算这个字节的值
    wiegand[0] = wiegand[0]|(*(str+1)&0x0f);
    wiegand[1] = wiegand[1]|(*(str+2)<<4);
    wiegand[1] = wiegand[1]|(*(str+3)&0x0f);
    wiegand[2] = wiegand[2]|(*(str+4)<<4);
    wiegand[2] = wiegand[2]|(*(str+5)&0x0f);
 
    //--------------------------------计算前12位1的个数是否为偶数，为偶效验用
    check_temp = wiegand[1]&0xf0;
    check_temp ^= wiegand[0];
    check_temp ^= check_temp>>4;
    check_temp ^= check_temp>>2;
    check_temp ^= check_temp>>1;
    even=!(check_temp&1);
 
    //--------------------------------计算后12位1的个数是否为偶数，为奇效验用
    check_temp = wiegand[1]&0x0f;
    check_temp ^= wiegand[2];
    check_temp ^= check_temp>>4;
    check_temp ^= check_temp>>2;
    check_temp ^= check_temp>>1;
    odd=check_temp&1;
 
    //================================启动发送，用定时器做时间延时
    //--------------------------------韦根 输出端初始化
    WIEGAND_DATA0High(); 
    WIEGAND_DATA1High(); 
    //--------------------------------发送偶效验
    if(even)
    {
        WG_send_bit_1();
    }
    else
    {
        WG_send_bit_0();
    }
    //-------------------------------发送24位数据
    for(i = 0;i<24;i++)
    {
        //---------------------------韦根 输出端初始化
        if((wiegand[0])&0x80)
        {
            WG_send_bit_1();
        }
        else
        {
            WG_send_bit_0();
        }
        (*(long*)&wiegand[0]) <<= 1;
    }
    //==============================发送奇效验位
    if(odd)
    {
        WG_send_bit_1();
    }
    else
    {
        WG_send_bit_0();
    }
}
