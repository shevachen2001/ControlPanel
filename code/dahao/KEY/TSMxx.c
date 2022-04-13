


/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/

#include "Tsmxx.h"
#include  "SYSTEM\Sys_GenFun.h"
#include  "I2C\I2c_Soft.h"
#include  "KEY\key.h"
#include  "KEY\TSMxx.h"
#include "SYSTEM\sys.h"








#define    WADD    0xD0
#define    RADD    0xD1
#define    SE        0xff
#define Rdata    0x10
#define    SENS1    0x02
#define    SENS2    0x03
#define    SENS3    0x04
#define    SENS4    0x05
#define    SENS5    0x06
#define    SENS6    0x07
#define    CTR1    0x08
#define    CTR2    0x09
#define    RST1    0x0A
#define    RST2    0x0B
#define    CH_HOLD1    0x0C
#define    CH_HOLD2    0x0D
#define    CAL_HOLD1    0x0E
#define    CAL_HOLD2    0x0F
#define    OUTPUT1        0x10
#define    OUTPUT2        0x11
#define    OUTPUT3        0x12

/*
#define    TSM_SCL()    MCU_IO_OUTPUT(0,3,2)
#define TSM_SCL_H()    MCU_IO_SET_HIGH(0, 3)
#define TSM_SCL_L()    MCU_IO_SET_LOW(0, 3)
#define    TSM_SDA()    MCU_IO_OUTPUT(0,2,2)
#define TSM_SDA_H()    MCU_IO_SET_HIGH(0, 2)
#define TSM_SDA_L()    MCU_IO_SET_LOW(0, 2)
#define    TSM_INT()    MCU_IO_INPUT(0,0,2)
#define    TSM_EN()    MCU_IO_OUTPUT(1,2,2)
#define    TSM_EN_L()    MCU_IO_SET_LOW(1,2)
#define    TSM_EN_H()    MCU_IO_SET_HIGH(1,2)
*/




#define TsmDataMax 3

uint8 TsmDataBuffer[TsmDataMax];
uint8 TSdata;

#if 0
const uint8 TSM_TOUCH_TAB[]=
{
    /*Data0*/KEY_CLEAR,
    /*Data1*/KEY_7,
    /*Data2*/KEY_4,
    /*Data3*/KEY_1,
    /*Data4*/KEY_3,
    /*Data5*/KEY_6,
    /*Data6*/KEY_9,
    /*Data7*/KEY_ENTER,
    /*Data8*/KEY_0,
    /*Data9*/KEY_8,
    /*Data10*/KEY_5,
    /*Data11*/KEY_2,
    /*Data12*/KEY_NULL,
    /*Data13*/KEY_NULL,
    /*Data14*/KEY_NULL,
    /*Data15*/KEY_NULL,
    /*Data*/KEY_NULL,
        
};
#endif

const Key_DataType TSM_TOUCH_TAB[]=
{
    /*Data0*/KEY_7,        //KEY_CLEAR,
    /*Data1*/KEY_CLEAR,    //KEY_7,
    /*Data2*/KEY_4,
    /*Data3*/KEY_2,        //KEY_1,
    /*Data4*/KEY_1,        //KEY_3, 
    /*Data5*/KEY_3,
    /*Data6*/KEY_5,
    /*Data7*/KEY_6,
    /*Data8*/KEY_8,
    /*Data9*/KEY_9,
    /*Data10*/KEY_0,
    /*Data11*/KEY_ENTER,
    /*Data12*/KEY_NULL,
    /*Data13*/KEY_NULL,
    /*Data14*/KEY_NULL,
    /*Data15*/KEY_NULL,
    /*Data*/KEY_NULL,
};


void Tsmxx_Read_Nyetes(uint8 *poutdat,uint8 subadd,uint8 num);
void Tsmxx_Write_Nyetes(uint8 *pindat,uint8 subadd,uint8 num);
extern void Tsmxx_Irq_Init(void);


const uint8 Tsmxx_Key_SensitivityTable[]=
{
#if 0
/*key 1*/0x06|0x08,
/*key 2*/0x06|0x08,
/*key 3*/0x06|0x08,
/*key 4*/0x06|0x08,
/*key 5*/0x06|0x08,
/*key 6*/0x06|0x08,
/*key 7*/0x06|0x08,
/*key 8*/0x06|0x08,
/*key 9*/0x06|0x08,
/*key 10*/0x06|0x08,
/*key 11*/0x06|0x08,
/*key 12*/0x06|0x08,
#endif
#if 1
/*key 1*/0x03|0x08,
/*key 2*/0x03|0x08,
/*key 3*/0x03|0x08,
/*key 4*/0x03|0x08,
/*key 5*/0x03|0x08,
/*key 6*/0x03|0x08,
/*key 7*/0x01|0x08,
/*key 8*/0x03|0x08,
/*key 9*/0x03|0x08,
/*key 10*/0x03|0x08,
/*key 11*/0x03|0x08,
/*key 12*/0x03|0x08,
#endif
#if 0
/*key 1*/0x01|0x08,
/*key 2*/0x01|0x08,
/*key 3*/0x01|0x08,
/*key 4*/0x01|0x08,
/*key 5*/0x01|0x08,
/*key 6*/0x01|0x08,
/*key 7*/0x01|0x08,
/*key 8*/0x01|0x08,
/*key 9*/0x01|0x08,
/*key 10*/0x01|0x08,
/*key 11*/0x01|0x08,
/*key 12*/0x01|0x08,
#endif
};

const uint8 Tsmxx_Key_SensitivityTable1[]=
{
/*key 1*/0x06|0x08,
/*key 2*/0x06|0x08,
/*key 3*/0x06|0x08,
/*key 4*/0x06|0x08,
/*key 5*/0x06|0x08,
/*key 6*/0x06|0x08,
/*key 7*/0x06|0x08,
/*key 8*/0x06|0x08,
/*key 9*/0x06|0x08,
/*key 10*/0x06|0x08,
/*key 11*/0x06|0x08,
/*key 12*/0x06|0x08,
};

const uint8 Tsmxx_Key_SensitivityTable2[]=
{
/*key 1*/0x05|0x08,
/*key 2*/0x05|0x08,
/*key 3*/0x05|0x08,
/*key 4*/0x05|0x08,
/*key 5*/0x05|0x08,
/*key 6*/0x05|0x08,
/*key 7*/0x05|0x08,
/*key 8*/0x05|0x08,
/*key 9*/0x05|0x08,
/*key 10*/0x05|0x08,
/*key 11*/0x05|0x08,
/*key 12*/0x05|0x08,
};

const uint8 Tsmxx_Key_SensitivityTable3[]=
{
/*key 1*/0x03|0x08,
/*key 2*/0x03|0x08,
/*key 3*/0x03|0x08,
/*key 4*/0x03|0x08,
/*key 5*/0x03|0x08,
/*key 6*/0x03|0x08,
/*key 7*/0x03|0x08,
/*key 8*/0x03|0x08,
/*key 9*/0x03|0x08,
/*key 10*/0x03|0x08,
/*key 11*/0x03|0x08,
/*key 12*/0x03|0x08,
};

const uint8 Tsmxx_Key_SensitivityTable4[]=
{
/*key 1*/0x02|0x08,
/*key 2*/0x02|0x08,
/*key 3*/0x02|0x08,
/*key 4*/0x02|0x08,
/*key 5*/0x02|0x08,
/*key 6*/0x02|0x08,
/*key 7*/0x02|0x08,
/*key 8*/0x02|0x08,
/*key 9*/0x02|0x08,
/*key 10*/0x02|0x08,
/*key 11*/0x02|0x08,
/*key 12*/0x02|0x08,
};

const uint8 Tsmxx_Key_SensitivityTable5[]=
{
/*key 1*/0x01|0x08,
/*key 2*/0x01|0x08,
/*key 3*/0x01|0x08,
/*key 4*/0x01|0x08,
/*key 5*/0x01|0x08,
/*key 6*/0x01|0x08,
/*key 7*/0x01|0x08,
/*key 8*/0x01|0x08,
/*key 9*/0x01|0x08,
/*key 10*/0x01|0x08,
/*key 11*/0x01|0x08,
/*key 12*/0x01|0x08,
};
const uint8 Tsmxx_Key_SensitivityTable6[]=
{
/*key 1*/0x00|0x08,
/*key 2*/0x00|0x08,
/*key 3*/0x00|0x08,
/*key 4*/0x00|0x08,
/*key 5*/0x00|0x08,
/*key 6*/0x00|0x08,
/*key 7*/0x00|0x08,
/*key 8*/0x00|0x08,
/*key 9*/0x00|0x08,
/*key 10*/0x00|0x08,
/*key 11*/0x00|0x08,
/*key 12*/0x00|0x08,
};
/****************************************************************************************************
**Function:
void Tsmxx_Write_Nyetes(void)
**Author: lxw
**Description:
**Input: mode
**Output: 
****************************************************************************************************/
void Tsmxx_Write_Nyetes(uint8 *pindat,uint8 subadd,uint8 num)
{
    if((I2C_SOFT_Start() == 1)&&(I2C_SOFT_Byte_Tx(WADD) == 1)&&(I2C_SOFT_Byte_Tx(subadd) == 1))
    {
        while (0 < num--)
        {
            if(I2C_SOFT_Byte_Tx(*pindat++) == 0)
            {
                break;
            }
        }
    }
    I2C_SOFT_Stop();
}
/****************************************************************************************************
**Function:
void Tsmxx_Write_Nyetes(void)
**Author: lxw
**Description:
**Input: mode
**Output: 
****************************************************************************************************/
void Tsmxx_Read_Nyetes(uint8 *poutdat,uint8 subadd,uint8 num)
{
    if((I2C_SOFT_Start() == 1)&&(I2C_SOFT_Byte_Tx(WADD)==1)&&(I2C_SOFT_Byte_Tx((uint8)subadd)==1))
    {
        I2C_SOFT_Stop();
        if((I2C_SOFT_Start() == 1)&&(I2C_SOFT_Byte_Tx(RADD) == 1))
        {
            
            while (1 < num--)
            {
                if(I2C_SOFT_Byte_Rx(poutdat, I2C_SOFT_RX_NOT_LAST_BYTE) == 0)
                {
                    
                    I2C_SOFT_Stop();
                    return ;
                }
                ++poutdat;
            }
            I2C_SOFT_Byte_Rx(poutdat, I2C_SOFT_RX_LAST_BYTE);
        }
        else 
        {
            I2C_SOFT_Stop();
            return ;
        }
    }
    I2C_SOFT_Stop();
}
/****************************************************************************************************
**Function:
    void Tsm_Init(void)
**Author: lxw
**Description:
**Input: mode
**Output: 
****************************************************************************************************/
void Tsm_Init(void)
{
    uint8 buf[2];
    const uint8 *pSensitivityTable;
    if(Sys_PataCfg.touchSensitive==0)
    {
        pSensitivityTable =Tsmxx_Key_SensitivityTable1;
    }
    else if(Sys_PataCfg.touchSensitive==1)
    {
        pSensitivityTable =Tsmxx_Key_SensitivityTable2;
    }
    else if(Sys_PataCfg.touchSensitive==2)
    {
        pSensitivityTable =Tsmxx_Key_SensitivityTable3;
    }    
    else if(Sys_PataCfg.touchSensitive==3)
    {
        pSensitivityTable =Tsmxx_Key_SensitivityTable4;
    }    
    else if(Sys_PataCfg.touchSensitive==4)
    {
        pSensitivityTable =Tsmxx_Key_SensitivityTable5;
    }
    else if(Sys_PataCfg.touchSensitive==5)
    {
        pSensitivityTable =Tsmxx_Key_SensitivityTable6;
    }
    else 
    {
        pSensitivityTable =Tsmxx_Key_SensitivityTable3;
    }
    buf[0] = 0x09;
    Tsmxx_Write_Nyetes(buf,CTR2,1);

    buf[0] = pSensitivityTable[0]|(pSensitivityTable[1]<<4);
    buf[1] = pSensitivityTable[2]|(pSensitivityTable[3]<<4);
    Tsmxx_Write_Nyetes(buf,SENS1,2);
	

    buf[0] = pSensitivityTable[4]|(pSensitivityTable[5]<<4);
    buf[1] = pSensitivityTable[6]|(pSensitivityTable[7]<<4);
    Tsmxx_Write_Nyetes(buf,0x06,2);  //  SENS3


    buf[0] = pSensitivityTable[8]|(pSensitivityTable[9]<<4);
    buf[1] = pSensitivityTable[11]|(pSensitivityTable[11]<<4);
    Tsmxx_Write_Nyetes(buf,0x22,2);  //   SENS5
/*
MS[0]  FTC[10] ILC[10] RTC[210]   
   0        11          00        010  
*/    
    buf[0] = 0x33;
//    buf[0] = 0x03;
    Tsmxx_Write_Nyetes(buf,CTR1,1);

    buf[0] = 0x0;
    buf[1] = 0x0;
    Tsmxx_Write_Nyetes(buf,RST1,2);
    
    buf[0] = 0x0;
    buf[1] = 0x0;
    Tsmxx_Write_Nyetes(buf,CH_HOLD1,2);
    
//    buf[0] = 0x62;
    // Set MSB to 1 to enable fast mode
    //
    buf[0] = 0x82;
    buf[1] = 0x07;
    Tsmxx_Write_Nyetes(buf,CTR1,2);
    
}
/****************************************************************************************************
**Function:
    void Tsm_Init(void)
**Author: lxw
**Description:
**Input: mode
**Output: 
****************************************************************************************************/
void Tsm_Rest(void)
{    
    Touch_I2cEnLow();
    Touch_SetRstHigh();
    Sys_GenFunDelayMs(200);
    Touch_SetRstLow();
    Sys_GenFunDelayMs(200);
    I2C_SOFT_Init();
    Tsm_Init();
}
/****************************************************************************************************
**Function:
void Tsm_Open(void)
**Author: lxw
**Description:
**Input: mode
**Output: 
****************************************************************************************************/
void Tsmxx_Open(void)
{
    nrf_gpio_cfg_output(TOUCH_SET_RST_PIN);
    nrf_gpio_cfg_input(TOUCH_INTT_PIN,NRF_GPIO_PIN_NOPULL) ;
    nrf_gpio_cfg_output(TOUCH_I2CEN_PIN);

    I2c_SoftClockSetOutput();
    I2c_SoftDatSetOutput();
    I2c_SoftClockLow();
    I2c_SoftDatLow();
    
    Tsm_Rest();
    Tsmxx_Irq_Init();
    Touch_I2cEnHigh();
}
/****************************************************************************************************
**Function:
uint8  Tsm(void)
**Author:lxw
**Description:
**Input: mode
**Output: 

****************************************************************************************************/

uint8 Tsmxx_ReadDat(uint8 num,uint8 *poutdat)
{    
    uint8 i;
    uint8 keydat[3] = {0};
    uint8 keydatbk[3] = {0};
    poutdat[0] = 0;
    poutdat[1] = 0;
    
    memset(keydat,0,3);
    memset(keydatbk,0,3);
    Touch_I2cEnLow();
    Sys_GenFunDelayUs(10);
	
    //Tsmxx_Read_Nyetes(keydat,OUTPUT1,3);

	Tsmxx_Read_Nyetes(&keydat[0], OUTPUT1, 1);
	Tsmxx_Read_Nyetes(&keydat[1], OUTPUT3, 2);
    
    I2c_SoftClockSetOutput();
    I2c_SoftDatSetOutput();
    I2c_SoftClockhigh();
    I2c_SoftDathigh();
    Touch_I2cEnHigh();
     if(((keydat[0] == 0)&&(keydat[1] == 0)&&(keydat[2] == 0))||((keydat[0] == 0xff)||(keydat[1] == 0xff)&&(keydat[2] == 0xff)))
    {
        return 0;
    }
    memcpy(keydatbk,keydat,3);
    for(i= 0;i< 4;i++)
    {
        poutdat[1]<<= 1;
        if((keydat[2]&0xC0) == 0xC0)
        {
            poutdat[1] |= 0x01; 
        }
        keydat[2]<<= 2;
    }
    for(i= 0;i< 4;i++)
    {
        poutdat[0]<<= 1;
        if((keydat[1]&0xC0) == 0xC0)
        {
            
            poutdat[0] |= 0x01; 
        }
        keydat[1]<<= 2;
    }
    for(i= 0;i< 4;i++)
    {
        poutdat[0]<<= 1;
        if((keydat[0]&0xC0) == 0xC0)
        {
            
            poutdat[0] |= 0x01; 
        }
        keydat[0]<<= 2;
    }    
    if((poutdat[0] != 0)||(poutdat[1] != 0)||(poutdat[2] != 0))
    {
        return 2;
    }
    memcpy(keydat,keydatbk,3);
    for(i= 0;i< 4;i++)
    {
        poutdat[1]<<= 1;
        if((keydat[2]&0xC0) == 0x80)
        {
            poutdat[1] |= 0x01; 
        }
        keydat[2]<<= 2;
    }
    for(i= 0;i< 4;i++)
    {
        poutdat[0]<<= 1;
        if((keydat[1]&0xC0) == 0x80)
        {
            
            poutdat[0] |= 0x01; 
        }
        keydat[1]<<= 2;
    }
    for(i= 0;i< 4;i++)
    {
        poutdat[0]<<= 1;
        if((keydat[0]&0xC0) == 0x80)
        {
            
            poutdat[0] |= 0x01; 
        }
        keydat[0]<<= 2;
    }        
    if((poutdat[0] != 0)||(poutdat[1] != 0)||(poutdat[2] != 0))
    {
        return 2;
    }    
    memcpy(keydat,keydatbk,3);
    for(i= 0;i< 4;i++)
    {
        poutdat[1]<<= 1;
        if((keydat[2]&0xC0) == 0x40)
        {
            poutdat[1] |= 0x01; 
        }
        keydat[2]<<= 2;
    }
    for(i= 0;i< 4;i++)
    {
        poutdat[0]<<= 1;
        if((keydat[1]&0xC0) == 0x40)
        {
            
            poutdat[0] |= 0x01; 
        }
        keydat[1]<<= 2;
    }
    for(i= 0;i< 4;i++)
    {
        poutdat[0]<<= 1;
        if((keydat[0]&0xC0) == 0x40)
        {
            
            poutdat[0] |= 0x01; 
        }
        keydat[0]<<= 2;
    }        
    return 2;
}

/****************************************************************************************************
**Function:
uint8  Tsm(void)
**Author:lxw
**Description:
**Input: mode
**Output: 
****************************************************************************************************/
Key_DataType Tsmxx_ReadIndex( void )
{
    uint8 KeyBuf[2];
    uint16 KeyData;
    uint16 BitIndex;
    uint8 i=0;
    uint8 ret;

    ret = Tsmxx_ReadDat(2, KeyBuf);
    if(ret == 2)
    {
        KeyData = BUILD_UINT16(KeyBuf[0], KeyBuf[1]);
        if(KeyData == 0)
        {
            return KEY_NO_RESULT;
        }
        BitIndex = 0x0001;
        for(i=0; i<16; i++)
        {
            if(KeyData & BitIndex)
            {
                return(TSM_TOUCH_TAB[i]);
            }
            BitIndex <<= 1;
        }
    }
    return KEY_NO_RESULT;
}

