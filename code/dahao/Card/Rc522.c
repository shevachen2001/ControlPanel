/*****************************************************************************
|  File Name: Rc522.c
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
| 2016-05 1.00    Rory     Initial version 
|****************************************************************************/

/****************************************************************************/ 
/* Include file                                                             */                                                 
/****************************************************************************/
#include "Card\Rc522.h"
#include "Access\Access.h"
#include "nrf_drv_common.h"
#include "nrf_drv_spi.h"
#include "app_util_platform.h"

//#define     RC522_SPI            SPI1






#define RC_ANTENNA_ON            (0x00)                                         // 天线开
#define RC_ANTENNA_OFF           (0x01)                                         // 天线关

#define RC_ISO14443_A            (0x00)                                         // A协议类
#define RC_ISO14443_B            (0x01)                                         // B协议类

#define RC_MAX_RLEN              (18)                                           // RC522接收最大长度


#define RC_WriteRawRC            WriteRawRC
#define RC_ReadRawRC            ReadRawRC
//const uint8 Card_Pswd[6]={0x33,0x55,0x77,0x11,0x22,0x88}; 
const uint8 Card_PswdDef[6]={0xff,0xff,0xff,0xff,0xff,0xff}; 
const uint8 Card_CntDef[4]={0xff,0x07,0x80,0x69}; 
#define RC522_DAT_BLOCK        4*Card_Page

uint8 Card_Pswd[6]={0xff,0xff,0xff,0xff,0xff,0xff};//{0x33,0x55,0x77,0x11,0xff,0xff}; 
uint8 Card_Page; 


Std_ReturnType Rc522_DetTypeB(Card_ParmType *pIndat);


unsigned char PcdComMF522(unsigned char Command, 
                 unsigned char *pInData, 
                 unsigned char InLenByte,
                 unsigned char *pOutData, 
                 unsigned int  *pOutLenBit,
                 unsigned int dlnum);
char PcdWrite(unsigned char addr,unsigned char *pData);
char PcdRead(unsigned char addr,unsigned char *pData);
char PcdSelect(unsigned char *pSnr);
char PcdAuthState(unsigned char auth_mode,unsigned char addr,unsigned char *pKey,unsigned char *pSnr);



 bool spi_xfer_done;  //SPI数据传输完成标志
 const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */


/****************************************************************************************************
**Function:
    void Rc522_SpiWriteReadData(uint8 *P_data,uint8 Num,uint8 R_W)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Rc522_SpiWriteReadData(uint8 *P_data,uint8 Num,uint8 R_W)
{
    #if 0
        for(;;)
        {
            uint8 i;
            uint8 dattemp =  0;
            if(R_W == STD_READ)
            {
                RC522_MosiLow();
                for(i= 0;i< 8;i++)
                {
                    dattemp<<= 1;
                    RC522_ClkLow();            
                    RC522_ClkHigh();                    
                    if(RC522_GetMisoDat())
                    {
                        dattemp|= 0x01;
                    }                                
                }
                *P_data++=dattemp;
            }
            else
            {
                dattemp = *P_data++;
                for(i= 0;i< 8;i++)
                {
                    RC522_ClkLow();
                    if(dattemp&0x80)
                    {
                        RC522_MosiHigh();
                    }
                    else 
                    {
                        RC522_MosiLow();
                    }
                    RC522_ClkHigh();
                    dattemp<<=1;
                }
            }
            if(--Num==0)return;
        }
    #else 
/*    for(;;)
    {
        uint8 i;
        uint8 dattemp =  0;
        if(R_W == STD_READ)
        {
            APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi,&dattemp, 1, P_data, 1));
            P_data++;
        }
        else
        {
            APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, P_data, 1,&dattemp, 1));
            P_data++;
        }
        if(--Num==0)return;
    }*/
//    uint8 spidatbuf[40];
//    memset(spidatbuf,0,sizeof(spidatbuf));
    if(R_W == STD_READ)
    {
//        spi_xfer_done = false;
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi,NULL, 0, P_data, Num));
//        while(spi_xfer_done);
    }
    else
    {
//        spi_xfer_done = false;
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, P_data, Num, NULL, 0));
//        while(spi_xfer_done);
    }
    #endif
}

/****************************************************************************************************
**Function:
uint8 Flash_SpiSendByte(uint8 byte)
**Author: lxw 
**Description:
**Input: 
**Output: 
****************************************************************************************************/
uint8 Rc522_SpiSendByte(uint8 byte)
{
#if 0
    uint8 i;
    uint8 dattemp =  0;
    RC522_MosiLow();
    for(i= 0;i< 8;i++)
    {
        dattemp<<= 1;
        RC522_ClkLow();            
        if(byte&0x80)
        {
            RC522_MosiHigh();
        }
        else 
        {
            RC522_MosiLow();
        }        
        byte<<=1;
        RC522_ClkHigh();                    
        if(RC522_GetMisoDat())
        {
            dattemp|= 0x01;
        }                                
    }
    return dattemp;
#else 
//    uint8 spi_rx_buf = 0xff;
//    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &byte, 1, &spi_rx_buf, 1));
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &byte, 1, NULL, 0));
    return (0xff);
#endif
}

/****************************************************************************************************
**Function:
uint8 Rc522_SpiReadByte(void)
**Author: lxw 
**Description:
**Input: 
**Output: 
****************************************************************************************************/
uint8 Rc522_SpiReadByte(void)
{
    uint8 data;
    (nrf_drv_spi_transfer(&spi, NULL, 0, &data, 1));
    return data;//(Rc522_SpiSendByte(0xA5));
}
//******************************************************************/
//功    能：写RC522寄存器
//参数说明：Address[IN]:寄存器地址
//          value[IN]:写入的值
//******************************************************************/
void WriteRawRC(unsigned char Address, unsigned char Value)
{  
     uint8 wbuf[2];
    RC522_SetCsLow();
/*    Address = ((Address<<1)&0x7E);
    Rc522_SpiSendByte(Address);
    Rc522_SpiSendByte(Value);
    */
    wbuf[0] = ((Address<<1)&0x7E);
    wbuf[1] = Value;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi,wbuf, 2, NULL, 0));
    RC522_SetCsHigh();
}
 
//******************************************************************/
//功    能：读RC522寄存器
//参数说明：Address[IN]:寄存器地址
//返    回：读出的值
//******************************************************************/
unsigned char ReadRawRC(unsigned char Address)
{

    unsigned char temp;
    RC522_SetCsLow();
    Address = ((Address<<1)&0x7E)|0x80;
    Rc522_SpiSendByte(Address);
    temp = Rc522_SpiReadByte();
    RC522_SetCsHigh();
    return(temp);
}

//******************************************************************/
//功    能：置RC522寄存器位
//参数说明：reg[IN]:寄存器地址
//          mask[IN]:置位值
//******************************************************************/
void SetBitMask(unsigned char reg,unsigned char mask)  
{
    char tmp = 0x0;
    tmp = ReadRawRC(reg)| mask;
    WriteRawRC(reg,tmp | mask);  // set bit mask
}

//******************************************************************/
//功    能：清RC522寄存器位
//参数说明：reg[IN]:寄存器地址
//          mask[IN]:清位值
//******************************************************************/
void ClearBitMask(unsigned char reg, unsigned char mask)  
{
    uint8 tmp = 0x0;
    tmp = ReadRawRC(reg);
    tmp &= (~mask);
    WriteRawRC(reg, tmp);  // clear bit mask 
} 
/////////////////////////////////////////////////////////////////////
//开启天线  
//每次启动或关闭天险发射之间应至少有1ms的间隔
/////////////////////////////////////////////////////////////////////
void PcdAntennaOn(void)
{
    unsigned char i;
    i = ReadRawRC(TxControlReg);
    if (!(i & 0x03))
    {
        SetBitMask(TxControlReg, 0x03);
    }
}

/////////////////////////////////////////////////////////////////////
//关闭天线
/////////////////////////////////////////////////////////////////////
void PcdAntennaOff(void)
{
    ClearBitMask(TxControlReg, 0x03);
}
/****************************************************************************************************
**Function:
   uint8 Rc522_PowerOn(uint8 on)
**Author: Rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Rc522_PowerOn(uint8 on)
{
    uint8 temp;
    if(on == STD_TRUE)
    {
        RC522_SetRstHigh();
        Sys_GenFunDelayUs(5);
        temp = ReadRawRC(CommandReg);//88    15
        temp &= ~0x30;
        WriteRawRC ( CommandReg, temp );            //向CommandReg 写入 0x0f    作用是使RC522复位
        PcdAntennaOff();
    }
    else 
    {
        temp = ReadRawRC(CommandReg);//88    15
        temp |= 0x30;
        WriteRawRC ( CommandReg, temp );            //向CommandReg 写入 0x0f    作用是使RC522复位
        WriteRawRC ( CommandReg, 0x0f );            //向CommandReg 写入 0x0f    作用是使RC522复位
        PcdAntennaOff();
        RC522_SetRstLow();
    }
}

/////////////////////////////////////////////////////////////////////
//功    能：复位RC522
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
//#define     TxAutoReg             0x15
/**************************************************************************************
* FunctionName   : RC_PcdISOType()
* Description    : 设置工作协议模式
* EntryParameter : type - 工作模式(ISO14443_A, ISO14443_B)
* ReturnValue    : None
**************************************************************************************/
void RC_PcdISOType(unsigned char type)
{
    Rc522_PowerOn(STD_TRUE);
    switch (type)
    {
        case  RC_ISO14443_A:
        {
            uint8 Temp;
            uint8 i = 20;
            
            WriteRawRC(ControlReg,0x10);//18 10
            WriteRawRC(ModeReg,0x3F);//22 3f
            WriteRawRC(RFU23,0x00);//46 00
            WriteRawRC(RFU25,0x80);//4a 80
            WriteRawRC(AutoTestReg,0x40);//6c 40 
            WriteRawRC(TxASKReg,0x40);//2a 40
            ReadRawRC(TxASKReg);//aa 40
            SetBitMask(TxControlReg,0x03);//a8 80 28 83
            WriteRawRC(TPrescalerReg,0x3D);//56 3d
            WriteRawRC(TModeReg,0x0D);//54 0d
            WriteRawRC(TReloadRegL,0x0A);//5a 0a
            WriteRawRC(TReloadRegH,0x00);//58 00
            WriteRawRC(ComIrqReg,0x01);//08 01
            WriteRawRC(RFCfgReg,0x73);
            SetBitMask(ControlReg,0x40);//98 10 18 50
            do
            {
                Temp = ReadRawRC(ComIrqReg);//88    15
                Sys_GenFunDelayUs(10);
                i--;
            }while((!(Temp&0x01))&&(i != 0));
            WriteRawRC(ComIrqReg,0x01);//08 01
            WriteRawRC(CommandReg,0x00);//02 00
    //        write dat  add 2017 03 08        
            WriteRawRC(ModeReg,0x3D);            
            WriteRawRC(TReloadRegL,30);              
            WriteRawRC(TReloadRegH,0);            
            WriteRawRC(TModeReg,0x8D);
            WriteRawRC(TPrescalerReg,0x3E);
            WriteRawRC(TxASKReg,0x40);
            ClearBitMask(TestPinEnReg, 0x80);//off MX and DTRQ out
            WriteRawRC(TxASKReg,0x40);

            break;
        }
        case  RC_ISO14443_B:
        {

            printf("RC_ISO14443_B\n\r");
            ClearBitMask(Status2Reg, 0x08);
            WriteRawRC(ModeReg, 0x3F);                                         // For 0xFFFF crc
            WriteRawRC(TReloadRegL, 30);
            WriteRawRC(TReloadRegH, 0);
            WriteRawRC(TModeReg, 0x8D);
            WriteRawRC(TPrescalerReg, 0x3E); 
            WriteRawRC(TxASKReg, 0);                                         // Force 100ASK = 0

            // ------------------------- 发送部分设置 ------------------------------
//            WriteRawRC(ModGsCfgReg, testreg++);   
//            WriteRawRC(GsNReg, testreg++);                                     // TX输出电导设置
//            WriteRawRC(ModGsCfgReg, testreg++);                                // 调制指数设置RegModGsp,, TYPEB ModConductance 0x1A
//            WriteRawRC(CWGsCfgReg, testreg++);                               // 调制指数设置RegModGsp,, TYPEB ModConductance 0x1A

//            WriteRawRC(CWGsCfgReg,10);
//            WriteRawRC(ModGsCfgReg, 5);   
//            WriteRawRC(CWGsCfgReg, testreg++);
            WriteRawRC(GsNReg, RC523_GSNCFG);                                         // TX输出电导设置
            WriteRawRC(CWGsCfgReg, RC523_CWGSCFG);
            WriteRawRC(ModGsCfgReg, RC523_MODGSCFG);                                   // 调制指数设置RegModGsp,, TYPEB ModConductance 0x1A
            WriteRawRC(TxModeReg, 0x03);                                     // 编码器设置,106kbps,14443B
            WriteRawRC(BitFramingReg, 0x00);                                 // 调制脉宽,0x13->2.95us RegTypeBFraming ,,TYPEB
            WriteRawRC(AutoTestReg, 0x00);

            // ------------------------- 接收部分设置 ------------------------------
            // 低二位为接收增益，
            // 00,10,20,30,40,50,60,70
            // 18,23,18,23,33,38,43,48dB
            WriteRawRC(RFCfgReg, 0x73);                                      // 0x59 RegRxControl1//73,
            WriteRawRC(RxModeReg, 0x03);                                     // 解码器设置,,106kbps,14443B
            WriteRawRC(RxThresholdReg, 0xf7);        //    0x75                // 高四位->最小信号强度，低三位->冲突最小信号强度,最大0xF7

            // ------------------------- TYPEB特征参数设定 -------------------------
            ClearBitMask(RxSelReg,0x3F);                                     // TR0
            SetBitMask(RxSelReg, 0x08);
            ClearBitMask(TxModeReg, 0x80);                                     // 无CRC,无奇偶校验
            ClearBitMask(RxModeReg, 0x80);
            ClearBitMask(Status2Reg, 0x08);                                  // MFCrypto1On =0
            PcdAntennaOn();
            Sys_GenFunDelayMs(5);
            break;
        }
        default: break;
    }
}


unsigned char PcdReset(void)
{
    RC522_SetRstLow();
    Sys_GenFunDelayMs(1);
    RC522_SetRstHigh();    
    return MI_OK;
}
/**************************************************************************************
* FunctionName   : RCRequestTypeB()
* Description    : 寻卡
* EntryParameter : None
* ReturnValue    : 成功返回MI_OK
**************************************************************************************/
unsigned char RCRequestTypeB(void)
{
    unsigned int len;
    unsigned char buf[RC_MAX_RLEN] = {0};

    buf[0] = 0x05;
    buf[1] = 0x00;
    buf[2] = 0x00;

    buf[3] = 0x71;                                                              // crc
    buf[4] = 0xFF;
    PcdComMF522(PCD_TRANSCEIVE, buf, 5, buf, &len,150);                       // 判断回应的数据buf是否为"50,00,...."    
    if(((len>>3) >= 2)&&(buf[0] == 0x50))
    {
        return MI_OK;
    }
    return MI_ERR;
}
/**************************************************************************************
* FunctionName   : RCATTRIBTypeB()
* Description    : 向TypeB卡发送ATTRIB指令
* EntryParameter : None
* ReturnValue    : 成功返回MI_OK
**************************************************************************************/
unsigned char RCATTRIBTypeB(void)
{
    unsigned int len;
    unsigned char buf[RC_MAX_RLEN] = {0};

    buf[ 0] = 0x1D;                                                             // 1d 00 00 00 00 00 08 01 08
    buf[ 1] = 0x00;                                                             // PUPI
    buf[ 2] = 0x00;
    buf[ 3] = 0x00;
    buf[ 4] = 0x00;

    buf[ 5] = 0x00;
    buf[ 6] = 0x08;
    buf[ 7] = 0x01;
    buf[ 8] = 0x08;

    buf[ 9] = 0xF3;                                                             // crc
    buf[10] = 0x10;
    PcdComMF522(PCD_TRANSCEIVE, buf, 11, buf, &len,500);                     // 判断回应的数据buf是否为"08"
    if((len >>3)!= 0)
    {
        return MI_OK;
    }
    return MI_ERR;
}

/**************************************************************************************
* FunctionName   : RCGetUIDTypeB()
* Description    : 获取UID
* EntryParameter : pUID - 返回UID
* ReturnValue    : 成功返回MI_OK
**************************************************************************************/
unsigned char RCGetUIDTypeB(unsigned char *pUID)
{
    unsigned int len;
    unsigned char buf[RC_MAX_RLEN] = {0};
    buf[0] = 0x00;
    buf[1] = 0x36;
    buf[2] = 0x00;
    buf[3] = 0x00;
    buf[4] = 0x08;

    buf[5] = 0x57;                                                              // crc
    buf[6] = 0x44;
    PcdComMF522(PCD_TRANSCEIVE, buf, 7, buf, &len,500);                      // 判断回应的数据是否为UID "....9000"
    if(((len>>3) >= 12)&&(buf[8] == 0x90))
    {
        memcpy(pUID, buf, 8);
        return MI_OK;
    }
    return MI_ERR;
}


/////////////////////////////////////////////////////////////////////
//功    能：通过RC522和ISO14443卡通讯
//参数说明：Command[IN]:RC522命令字
//          pInData[IN]:通过RC522发送到卡片的数据
//          InLenByte[IN]:发送数据的字节长度
//          pOutData[OUT]:接收到的卡片返回数据
//          *pOutLenBit[OUT]:返回数据的位长度
/////////////////////////////////////////////////////////////////////
unsigned char PcdComMF522(unsigned char Command, 
                 unsigned char *pInData, 
                 unsigned char InLenByte,
                 unsigned char *pOutData, 
                 unsigned int  *pOutLenBit,
                 unsigned int dlnum)
{
    uint8 status  = MI_OK;
    uint8 irqEn   = 0x00;
    uint8 waitFor = 0x00;
    uint8 lastBits;
    uint8 n;
    uint16 i;

    switch (Command)
    {
         case PCD_AUTHENT   : 
             irqEn = 0x12; waitFor = 0x10; 
         break;
         case PCD_TRANSCEIVE: 
             irqEn = 0x77; waitFor = 0x30; 
         break;
         default: 
         break;
    }
   
    RC_WriteRawRC(ComIEnReg, irqEn|0x80);
    ClearBitMask(ComIrqReg,0x80);
    RC_WriteRawRC(CommandReg, PCD_IDLE);
    SetBitMask(FIFOLevelReg,0x80);
    for (i=0; i<InLenByte; i++)
    {   
        RC_WriteRawRC(FIFODataReg, pInData[i]);    
    }
    memset(pInData,0,sizeof(pInData));

    RC_WriteRawRC(CommandReg, Command);
    if (Command == PCD_TRANSCEIVE)
    {    
        SetBitMask(BitFramingReg,0x80);  
    }
    i = dlnum;                                                                     // 根据时钟频率调整，操作M1卡最大等待时间25ms
    do 
    {
        n = RC_ReadRawRC(ComIrqReg);
        i--;
        
    } while ((i!=0) && !(n&0x01) && !(n&waitFor));
        ClearBitMask(BitFramingReg,0x80);
    if (i!=0)
    {    
        if (!(RC_ReadRawRC(ErrorReg) & 0x1B))
        {
            status = MI_OK;
            if (n & irqEn & 0x01)
            {   
                status = MI_NOTAGERR;  
            }
            
            if (Command == PCD_TRANSCEIVE)
            {
                n = RC_ReadRawRC(FIFOLevelReg);
                  lastBits = RC_ReadRawRC(ControlReg) & 0x07;
                *pOutLenBit = (lastBits) ? ((n-1)*8 + lastBits) : (n*8);
                
                if (n == 0)
                {
                    n = 1;    
                }
                if (n > RC_MAX_RLEN)
                {   
                    n = RC_MAX_RLEN;   
                }
                
                for (i=0; i<n; i++)
                {
                    pOutData[i] = RC_ReadRawRC(FIFODataReg);    
                }
            }
        }
        else
        {
            status = MI_ERR;
        }
    }
    SetBitMask(ControlReg, 0x80);                                            // stop timer now
    RC_WriteRawRC(CommandReg, PCD_IDLE); 
     return status;
 }

//******************************************************************/
//功    能：寻卡                                                    /
//参数说明: req_code[IN]:寻卡方式                                   /
//                0x52 = 寻感应区内所有符合14443A标准的卡           /
//                0x26 = 寻未进入休眠状态的卡                       /
//          pTagType[OUT]：卡片类型代码                             /
//                0x4400 = Mifare_UltraLight                        /
//                0x0400 = Mifare_One(S50)                          /
//                0x0200 = Mifare_One(S70)                          /
//                0x0800 = Mifare_Pro(X)                            /
//                0x4403 = Mifare_DESFire                           /
//返    回: 成功返回MI_OK                                           /
//******************************************************************/
uint8 PcdRequestA(uint8 req_code, uint8 *pTagType)
{
 
     char status;
     unsigned int  unLen;
     unsigned char ucComMF522Buf[MAXRLEN];
     char i=0;
     do
     {
         status = ReadRawRC(Status2Reg); //90 00(01,05)
         WriteRawRC(Status2Reg,status&0xf7); //10 00
         WriteRawRC(CollReg,0x80);         //1c 80
         ClearBitMask(TxModeReg,0x80);     //a4 00 24 00
         ClearBitMask(RxModeReg,0x80);     //a6 00 26 00
         WriteRawRC(BitFramingReg,0x07); //1a 07
         SetBitMask(TxControlReg,0x03);  //a8 83 28 83
         ucComMF522Buf[0] = req_code;
         i++;
         status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,1,ucComMF522Buf,&unLen,20);  //250
             break;
     }while(status ==MI_ERR);
    if ((status == MI_OK) && (unLen == 0x10))
    {     
        *pTagType      = ucComMF522Buf[0];
        *(pTagType+1) = ucComMF522Buf[1];
    }
    else
    {    
        status = MI_ERR;    
}
    return status;
}


/////////////////////////////////////////////////////////////////////
//功    能：防冲撞
//参数说明: pSnr[OUT]:卡片序列号，4字节
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////  
unsigned char PcdAnticoll(unsigned char *pSnr)
{
    unsigned char status;
    static  unsigned int    unLen;
    unsigned char  ucComMF522Buf[64];
    unsigned char  i;
    unsigned char  snr_check=0;    

    ClearBitMask(TxModeReg,0x80);//a4 00 24 00
    ClearBitMask(RxModeReg,0x80);//a6 00 26 00
    WriteRawRC(CollReg,0x00);    //1c 00
    WriteRawRC(BitFramingReg,0x00);//1a 00
    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x20;
    
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,2,ucComMF522Buf,&unLen,250);
    WriteRawRC(BitFramingReg,0x00);//1a 00
    WriteRawRC(CollReg,0x80);//1c 80 
    if (status == MI_OK)
    {
        for (i=0; i<4; i++)
        {
            *(pSnr+i)    = ucComMF522Buf[i];
            snr_check ^= ucComMF522Buf[i];
        }
        if (snr_check != ucComMF522Buf[i])
        {
            status = MI_ERR;   
        }
    }
    return status;
}
uint8 kk[4];

/****************************************************************************************************
**Function:
   Std_ReturnType Rc522_DetTypeA(Card_ParmType *pIndat)
**Author: Rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
Std_ReturnType Rc522_DetTypeA(Card_ParmType *pIndat)
{
    uint8 cardgettype[2];
    uint8 err=E_NOT_OK;
    RC_PcdISOType(RC_ISO14443_A);
     if(PcdRequestA(PICC_REQALL,cardgettype)==MI_OK)
    {
        if(PcdAnticoll(pIndat->pid)==MI_OK)
        {   
            //memcpy(kk,pIndat->pid,4 );
            pIndat->IdType = KEY_TYPE_CARD;
#if (RC522_RDDAT == STD_TRUE)
            if(MI_OK == PcdSelect(pIndat->pid))
            {
                if(MI_OK == PcdAuthState(PICC_AUTHENT1A,RC522_DAT_BLOCK,(uint8 *)Card_Pswd,pIndat->pid))
                {
                    uint8 *pdat= pIndat->pdat;
                    if(MI_OK == PcdRead(RC522_DAT_BLOCK+1,pdat))
                            {
                                return E_OK;
                            }
                        }
            }
            return E_OK;
#else             
            err = E_OK;
#endif
        }
    }
    return err;
}
/****************************************************************************************************
**Function:
   Std_ReturnType Rc522_DetTypeA(Card_ParmType *pIndat)
**Author: Rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
Std_ReturnType Rc522_DetTypeB(Card_ParmType *pIndat)
{
    uint8 err;

    err=E_NOT_OK;
    RC_PcdISOType(RC_ISO14443_B);
    if(RCRequestTypeB() == MI_OK)
    {
        if(RCATTRIBTypeB() == MI_OK)
        {
            if(RCGetUIDTypeB(pIndat->pid)==MI_OK)
            {
                pIndat->IdType = KEY_TYPE_IDENTITY_CARD;
                                err = E_OK;
            }
        }
    }
    //Rc522_PowerOn(STD_FALSE);
    return err;
}
/////////////////////////////////////////////////////////////////////
//用MF522计算CRC16函数
/////////////////////////////////////////////////////////////////////
void CalulateCRC(unsigned char *pIndata,unsigned char len,unsigned char *pOutData)
{
    unsigned char i,n;
    ClearBitMask(DivIrqReg,0x04);
    WriteRawRC(CommandReg,PCD_IDLE);
    SetBitMask(FIFOLevelReg,0x80);
    for (i=0; i<len; i++)
    {   WriteRawRC(FIFODataReg, *(pIndata+i));   }
    WriteRawRC(CommandReg, PCD_CALCCRC);
    i = 0xFF;
    do 
    {
        n = ReadRawRC(DivIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x04));
    pOutData[0] = ReadRawRC(CRCResultRegL);
    pOutData[1] = ReadRawRC(CRCResultRegM);
}
/////////////////////////////////////////////////////////////////////
//功    能：选定卡片
//参数说明: pSnr[IN]:卡片序列号，4字节
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdSelect(unsigned char *pSnr)
{
    char status;
    unsigned char i;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 
    
    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x70;
    ucComMF522Buf[6] = 0;
    for (i=0; i<4; i++)
    {
        ucComMF522Buf[i+2] = *(pSnr+i);
        ucComMF522Buf[6]  ^= *(pSnr+i);
    }
    CalulateCRC(ucComMF522Buf,7,&ucComMF522Buf[7]);
  
    ClearBitMask(Status2Reg,0x08);

    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,9,ucComMF522Buf,&unLen,1000);
    
    if ((status == MI_OK) && (unLen == 0x18))
    {   status = MI_OK;  }
    else
    {   status = MI_ERR;    }

    return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：验证卡片密码
//参数说明: auth_mode[IN]: 密码验证模式
//                 0x60 = 验证A密钥
//                 0x61 = 验证B密钥 
//          addr[IN]：块地址
//          pKey[IN]：密码
//          pSnr[IN]：卡片序列号，4字节
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////                   
char PcdAuthState(unsigned char auth_mode,unsigned char addr,unsigned char *pKey,unsigned char *pSnr)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = auth_mode;
    ucComMF522Buf[1] = addr;
    for (i=0; i<6; i++)
    {    
        ucComMF522Buf[i+2] = *(pKey+i);   
    }
    
    for (i=0; i<6; i++)
    {    
        ucComMF522Buf[i+8] = *(pSnr+i);   
    }
    
    status = PcdComMF522(PCD_AUTHENT,ucComMF522Buf,12,ucComMF522Buf,&unLen,1000);
    if ((status != MI_OK) || (!(ReadRawRC(Status2Reg) & 0x08)))
    {   status = MI_ERR;   }
    
    return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：读取M1卡一块数据
//参数说明: addr[IN]：块地址
//          p [OUT]：读出的数据，16字节
//返    回: 成功返回MI_OK
///////////////////////////////////////////////////////////////////// 
char PcdRead(unsigned char addr,unsigned char *pData)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_READ;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
   
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen,1000);
    if ((status == MI_OK) && (unLen == 0x90))
    {
        for (i=0; i<16; i++)
        {    *(pData+i) = ucComMF522Buf[i];   }
    }
    else
    {   status = MI_ERR;   }
    
    return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：写数据到M1卡一块
//参数说明: addr[IN]：块地址
//          p [IN]：写入的数据，16字节
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////                 
char PcdWrite(unsigned char addr,unsigned char *pData)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 
    
    ucComMF522Buf[0] = PICC_WRITE;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen,1000);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }
        
    if (status == MI_OK)
    {
        for (i=0; i<16; i++)
        {    ucComMF522Buf[i] = *(pData+i);   }
        CalulateCRC(ucComMF522Buf,16,&ucComMF522Buf[16]);
        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,18,ucComMF522Buf,&unLen,1000);
        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {   status = MI_ERR;   }
    }
    
    return status;
}
/////////////////////////////////////////////////////////////////////
//功    能：扣款和充值
//参数说明: dd_mode[IN]：命令字
//               0xC0 = 扣款
//               0xC1 = 充值
//          addr[IN]：钱包地址
//          pValue[IN]：4字节增(减)值，低位在前
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////                
char PcdValue(unsigned char dd_mode,unsigned char addr,unsigned char *pValue)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 
    
    ucComMF522Buf[0] = dd_mode;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen,1000);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }
        
    if (status == MI_OK)
    {
        for (i=0; i<16; i++)
        {    ucComMF522Buf[i] = *(pValue+i);   }
        CalulateCRC(ucComMF522Buf,4,&ucComMF522Buf[4]);
        unLen = 0;
        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,6,ucComMF522Buf,&unLen,1000);
        if (status != MI_ERR)
        {    status = MI_OK;    }
    }
    
    if (status == MI_OK)
    {
        ucComMF522Buf[0] = PICC_TRANSFER;
        ucComMF522Buf[1] = addr;
        CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]); 
   
        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen,1000);

        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {   status = MI_ERR;   }
    }
    return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：备份钱包
//参数说明: sourceaddr[IN]：源地址
//          goaladdr[IN]：目标地址
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdBakValue(unsigned char sourceaddr, unsigned char goaladdr)
{
    char status;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_RESTORE;
    ucComMF522Buf[1] = sourceaddr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen,1000);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }
    
    if (status == MI_OK)
    {
        ucComMF522Buf[0] = 0;
        ucComMF522Buf[1] = 0;
        ucComMF522Buf[2] = 0;
        ucComMF522Buf[3] = 0;
        CalulateCRC(ucComMF522Buf,4,&ucComMF522Buf[4]);
 
        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,6,ucComMF522Buf,&unLen,1000);
        if (status != MI_ERR)
        {    status = MI_OK;    }
    }
    
    if (status != MI_OK)
    {    return MI_ERR;   }
    
    ucComMF522Buf[0] = PICC_TRANSFER;
    ucComMF522Buf[1] = goaladdr;

    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen,1000);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }

    return status;
}




/****************************************************************************************************
**Function:
   Std_ReturnType Rc522_Open(void *pData)
**Author: Rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
 void Rc522_Open(void *pData)
{ 
#if 0
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

//    GPIO_InitStructure.GPIO_Pin = SPI_CLK_PIN|SPI_MISO_PIN|SPI_MOSI_PIN ; 
//    GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(SPI_CLK_GPIO,&GPIO_InitStructure); 

    GPIO_InitStructure.GPIO_Pin = RC522_CS_PIN;             
    GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_PP;
    GPIO_Init(RC522_CS_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = RC522_RST_PIN;             
    GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_PP;
    GPIO_Init(RC522_RST_GPIO, &GPIO_InitStructure);

/*    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStructure);
`    SPI_Cmd(SPI1, ENABLE);*/
#else 
//    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG(SPI_INSTANCE);

/*    nrf_gpio_cfg_output(RC522_CS_PIN);
    nrf_gpio_cfg_output(RC522_RST_PIN);
    nrf_gpio_cfg_output(RC522_MSCK_PIN);
    nrf_gpio_cfg_output(RC522_MOSI_PIN);
    nrf_gpio_cfg_output(FLASH_CS_PIN);    
    nrf_gpio_cfg_input(RC522_MISO_PIN,GPIO_PIN_CNF_PULL_Disabled);*/
/*
    spi_config.miso_pin = RC522_MISO_PIN;
    spi_config.mosi_pin = RC522_MOSI_PIN;
    spi_config.sck_pin = RC522_MSCK_PIN;
    nrf_drv_spi_init(&spi, &spi_config, spi_event_handler);
*/

    nrf_gpio_cfg_output(RC522_CS_PIN);
    nrf_gpio_cfg_output(RC522_RST_PIN);
    
    RC522_SetCsHigh();
    RC522_SetRstLow();
#endif
}

/****************************************************************************************************
**Function:
   int8 Wifi_Read(void *pData)
**Author: Rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
Std_ReturnType Rc522_Read(void *pData)
{
    uint8 err;
    err = E_NOT_OK;
    if(Rc522_DetTypeA(pData) == E_OK)
    {
        err= E_OK;
    }
#if (RC523_TYPEB== STD_TRUE)
    else if(Rc522_DetTypeB(pData) == E_OK)
    {
        err= E_OK;
    }
#endif
    Rc522_PowerOn(STD_FALSE);
    return err;
}
/****************************************************************************************************
**Function:
   int8 Wifi_Read(void *pData)
**Author: Rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/

Std_ReturnType Rc522_Ioctl(uint8 Cmd,void *pData)
{
    switch(Cmd)
    {
        case CARD_RD_DAT:        
        {
            RC_PcdISOType(RC_ISO14443_A);
            uint8 cardgettype[2];
            if(PcdRequestA(PICC_REQALL,cardgettype)==MI_OK)
            {
                uint8 id[4];
                if(PcdAnticoll(id)==MI_OK)
                { 
                    if(MI_OK == PcdSelect(id))
                    {
                        if(MI_OK == PcdAuthState(PICC_AUTHENT1A,RC522_DAT_BLOCK,(uint8 *)Card_Pswd,id))
                        {
                            uint8 *pdat= pData;
                            if(MI_OK == PcdRead(RC522_DAT_BLOCK,pdat))
                            {
                                if(MI_OK == PcdRead(RC522_DAT_BLOCK+1,&pdat[16]))
                                {
                                    if(MI_OK == PcdRead(RC522_DAT_BLOCK+2,&pdat[32]))
                                    {
                                        return E_OK;
                                    }
                                }
                            }
                        }                        
                        return E_NOT_OK;
                    }
                }
            }            
        }break;

        case CARD_PSW_INIT:
        {
            uint8 *pdat= pData;
            if(pdat[0]> 8 )
            {
                return E_NOT_OK;
            }
            Card_Page= pdat[0];
            memcpy(Card_Pswd,&pdat[1],4);
            Card_Pswd[4]=0xff;
            Card_Pswd[5]=0xff;
            return E_OK;
        }
        default:
            break;
    }
    return E_NOT_OK;
}
 
 
const Dr_OpertonType Rc522Drive =
{
    Rc522_Open,NULL, Rc522_Read, NULL, Rc522_Ioctl
};


#if (RC522_CHECK == STD_TRUE)

/*******************************************************************************
* Function Name  : 
void Rc522_Check(void)
* Description    : 
*                  reset values.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Rc522_Check(void)
{
    uint8 CardId[8] = {0};
    Card_ParmType pcard;
    pcard.pid = CardId;
    
    Rc522Drive.Open(NULL);
    
    static uint8 dat[48];
    for(;;)
    {    
        Sys_GenFunDelayMs(500);
        #if 0
        if(Rc522Drive.Read(&pcard) == E_OK)
        {
            Card_PswType Card_Psw;            
            Card_Psw.id[0] = CardId[0];
            Card_Psw.id[1] = CardId[1];
            Card_Psw.id[2] = CardId[2];
            Card_Psw.id[3] = CardId[3];
            if(MI_OK == PcdSelect(CardId))
            {
                Card_Psw.add = 1;
                memset(Card_Psw.Pswd,0xff,6);
                Card_Psw.KeyType = 1;
                if(MI_OK == PcdAuthState(PICC_AUTHENT1A,4, Card_Psw.Pswd,Card_Psw.id))
                {
                    static uint8 datbuf[16];
                    memset(datbuf,0x55,16);
                    if(MI_OK == PcdWrite(4,datbuf))
                    {
                        Sys_GenFunDelayMs(10);
                    }                    
                    if(MI_OK == PcdWrite(5,datbuf))
                    {
                        Sys_GenFunDelayMs(10);
                    }
                    memset(datbuf,0,16);
                    if(MI_OK == PcdRead(4,datbuf))
                    {
                        Sys_GenFunDelayMs(10);
                    }                    
                    memset(datbuf,0,16);
                    if(MI_OK == PcdRead(5,datbuf))
                    {
                        Sys_GenFunDelayMs(10);
                    }
                }
            }
        }
        #else 
        memset(dat,0x33,48);
        Rc522_Ioctl(CARD_WR_DAT,dat);
        memset(dat,0,48);
        Rc522_Ioctl(CARD_RD_DAT,dat);
        
        memcpy(dat,Card_Pswd,6);
        Rc522_Ioctl(CARD_PSW_SET,dat);
        #endif
    }
}
#endif



/************************ (C) COPYRIGHT DahaoMicroelectronics *****END OF FILE****/


