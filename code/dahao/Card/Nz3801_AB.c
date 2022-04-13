#include "Nz3801_AB.h"
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_drv_spi.h"
#include "cdemo.h"
#include "app_error.h"
#include <stdbool.h>
#include "stdio.h"
#include "nz3801.h"
#include "cdemo.h"
#include "SYSTEM\sys.h"
#include "Access\LockKey.h"

#define SPI_INSTANCE  0 

const unsigned char  FSCTab[8]={16,24,32,40,48,64,96,128}; 
const unsigned char  UartSpeedTab[12]= {0xFA,0xEB,0xDA,0xCB,0xAB,0x9A,0x7A,0x74,0x5A,0x3A,0x1C,0x15};

const unsigned char TestProduction_FobTab1[4]={0x3E, 0x4A, 0xF3, 0x22};
const unsigned char TestProduction_FobTab2[4]={0xCB, 0xDA, 0x1A, 0xAC};
const unsigned char TestProduction_FobTab3[4]={0xCB, 0x82, 0x30, 0xAC};

unsigned int  Nz3801AdcVoltageTab[Max_Len_NZ3801]={0}; 
unsigned char Nz3801TabLen=0;
unsigned int  AdcVoltageThreshold=0;
unsigned int  AdcVoltageSummation=0;

unsigned char  CType;		 		 // 0 TypeA, 1 TypeB
unsigned char  PCB;          // 块头域PCB字段
unsigned char  FWI = 4;      // 等待时间
unsigned short FSC;		 		 	 // PICC最大字节数
unsigned short FSD;          // PCD最大字节数
unsigned char  CID;          
unsigned char  NAD;          
unsigned char  BlockNum = 0;  // 当前块号

unsigned char  reg18h_RXTRESHOLD = 0x9b;
unsigned char  reg24h_MODWIDTH   = 0x26;
unsigned char  reg26h_RFCFG      = 0x68;
unsigned char  reg27h_GSN        = 0xff;  
unsigned char  reg28h_CWGSP      = 0x3f;
unsigned char  reg29h_MODGSP     = 0x2a;
unsigned char  reg16h_TXSEL      = 0x10;
unsigned char  reg38h_ANALOGTEST = 0x00;

unsigned char  CPUSN[4]={0};
unsigned char  CPUKeyTab[24]={0};
const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);

uint8_t Rc522_SpiSendByte(uint8_t byte);
uint8_t Rc522_SpiReadByte(void);

void Sys_SpiOpen(void);
void Sys_SpiClose(void);

extern unsigned char LowVoltage_ShutdownFlag;
extern unsigned char LogsPswTemp[4];

 uint8  Access_IllegalFobCount;
 uint8  Access_IllegalFobTimeout;

extern void Access_DetCardProc(void);
extern void Shutdown_BuzzerAction(void);

extern  CPUFob_DataType  CPUFobKey_List;


unsigned char FobFlag=0;


uint8_t Rc522_SpiSendByte(uint8_t byte)
{
 	 APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &byte, 1, NULL, 0));
	 return (0xff);
}


uint8_t Rc522_SpiReadByte(void)
{
	uint8_t data;
	(nrf_drv_spi_transfer(&spi, NULL, 0, &data, 1));
	return data;
}

unsigned char ReadRawRC(unsigned char Address)
{
	unsigned char temp=0;

	 SPI_CS_Enable();
	 Address = (((Address<<1)&0x7E)|0x80);
	 Rc522_SpiSendByte(Address);
	 temp = Rc522_SpiReadByte();
	 SPI_CS_Disable();
	 return(temp);
}


void WriteRawRC(unsigned char  Address,   unsigned char  value)
{
    unsigned char   ucAddr;
	  unsigned char   wbuf[2];
	 
    SPI_CS_Enable();
    ucAddr = ((Address<<1)&0x7E);

	  wbuf[0]=ucAddr;
	  wbuf[1]=value;
	
	  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi,wbuf, 2, NULL, 0));
	
    SPI_CS_Disable();
}


void KCT_DelayMs(unsigned short ms)
{
    nrf_delay_ms(ms);
}


void KCT_DelayUs(unsigned short us)
{
   nrf_delay_us(us);
}


void RF_RST_Enable(void)		
{
	 Nz3801RST_En();
}


void RF_RST_Disable(void)		
{
	  Nz3801RST_Dis();
}


static void KCT_WriteReg(unsigned char address, unsigned char value)
{
 	unsigned char  ucAddr=0;
	unsigned char  wbuf[2]={0};
	
    SPI_CS_Enable();
	
	   ucAddr=( (address<<1)&0x7E );

	   wbuf[0]=ucAddr;
	   wbuf[1]=value;
     APP_ERROR_CHECK( nrf_drv_spi_transfer(&spi, wbuf, 2, NULL, 0) );

     SPI_CS_Disable();

}

static unsigned char KCT_ReadReg(unsigned char address)
{
    unsigned char rx_dat=0xFF;
	
	  rx_dat=ReadRawRC(address);
	
    return(rx_dat);
}



unsigned int power(unsigned char n)
{
    unsigned char i=0;
    unsigned int  t=0;
	
    t=1;
    for(i=0; i<n; i++)
    {
       t *= 2;
    }
    return(t);
}

void *mem_copy(void * dest,const void *src, unsigned short count)
{
    char *tmp = (char *)dest;
    char *s   = (char *)src;
	
    if(dest <= src)
    {
        while (count--)
        {
            *tmp++ = *s++;
        }
    }
    else
    {
        tmp = (char *) dest + count;		//从后往前拷贝，防止数据被覆盖掉
        s   = (char *) src + count;
        while (count--)
				{
            *--tmp = *--s;
				}
    }
    return(dest);
}


void KCT_SetBitMask(unsigned char reg,unsigned char mask)
{
    unsigned char tmp=0;
    tmp = KCT_ReadReg(reg);
    KCT_WriteReg(reg,(tmp|mask));
}


void KCT_ClearBitMask(unsigned char reg,unsigned char mask)
{
    unsigned char tmp;
    tmp = KCT_ReadReg(reg);
    KCT_WriteReg(reg,  (tmp&(~mask)));
}


void KCT_SetRegExt(unsigned char extRegAddr,unsigned char extRegData)
{
    unsigned char addr,regdata;

    addr = BFL_JREG_EXT_REG_ENTRANCE;
    regdata = BFL_JBIT_EXT_REG_WR_ADDR + extRegAddr;
    KCT_WriteReg(addr, regdata);

    addr = BFL_JREG_EXT_REG_ENTRANCE;
    regdata = BFL_JBIT_EXT_REG_WR_DATA + extRegData;
    KCT_WriteReg(addr, regdata);
}



void Debug_ClearFifo(void)
{
	 	unsigned char value=0xFF;
	
	  value=KCT_ReadReg(FIFOLEVEL&0x7F);
	  //nrf_delay_ms(1);
    while(value)
    {
        KCT_SetBitMask(FIFOLEVEL,  0x00|BFL_JBIT_FLUSHBUFFER);
  			//nrf_delay_ms(1);
			  value=KCT_ReadReg(FIFOLEVEL&0x7F);
    }
}

void KCT_ClearFifo(void)
{
	  while( (KCT_ReadReg(FIFOLEVEL&0x7f) !=0) )
    {
        KCT_SetBitMask(FIFOLEVEL,  0x00|BFL_JBIT_FLUSHBUFFER);
    }
}


void KCT_ClearFlag(void)
{
    KCT_WriteReg(COMMIRQ, 0x7f);
    KCT_WriteReg(DIVIRQ,  0x7f);
}


void KCT_StartCmd(unsigned char cmd)
{
    KCT_WriteReg(COMMAND, cmd);
}


void KCT_StopCmd(void)
{
    KCT_WriteReg(COMMAND, 0x00);
}


void KCT_SetCRC(unsigned char bEN)
{
    if(bEN)
    {
        KCT_SetBitMask(TXMODE, BFL_JBIT_CRCEN);
        KCT_SetBitMask(RXMODE, BFL_JBIT_CRCEN);
    }
    else
    {
        KCT_ClearBitMask(TXMODE, BFL_JBIT_CRCEN);
        KCT_ClearBitMask(RXMODE, BFL_JBIT_CRCEN);
    }
}


void KCT_SetPARITY(unsigned char bEN)
{
    ;
}


unsigned char KCT_FlagOK(void)
{
    if( (KCT_ReadReg(REGERROR)&(BFL_JBIT_CRCERR|BFL_JBIT_PROTERR/*|BFL_JBIT_COLLERR|BFL_JBIT_PARITYERR*/))==0)
    {
        return NZ3801_TRUE;
    }
    return NZ3801_FALSE;
}

unsigned char KCT_CrcOK(void)
{
    if((KCT_ReadReg(REGERROR)&(BFL_JBIT_CRCERR))==0)
    {
        return NZ3801_TRUE;
    }
    return NZ3801_FALSE;
}


void Nz3801_SetTimer(unsigned int fc)
{
    unsigned int prescale = 0;
    unsigned int t=0;

    t = fc;
    while(fc>65535)
    {
        prescale++;
        fc = t/(2*prescale+1);
        if(fc*(2*prescale+1) != t)
				{
           fc++;
				}
    }
    if(prescale>=4096)
    {
        fc = 65535;
        prescale = 4095;
    }
    KCT_WriteReg(TMODE, 0x90|((prescale>>8)&0xf));
    KCT_WriteReg(TPRESCALER, prescale&0xff);
    KCT_WriteReg(TRELOADHI, (fc>>8)&0xff);
    KCT_WriteReg(TRELOADLO, fc&0xff);
}


unsigned char Nz3801_HwReset(void)
{
    RF_RST_Enable();			//
    //KCT_DelayUs(1);				//低电平持续1us(要求 >= 100ns)，触发复位
    KCT_DelayMs(10);	
    RF_RST_Disable();			//
    //KCT_DelayMs(1);				//nRST高电平工作等待 1ms再进入工作
    KCT_DelayMs(5);
    return(SUCCEED_NZ3801);
}


unsigned char Nz3801_SoftReset(void)
{
    KCT_WriteReg(COMMAND,COMMAND_SOFTRESET);
    KCT_DelayMs(1);
    return SUCCEED_NZ3801;
}


unsigned char Nz3801_SoftPwrDown(unsigned char bEnable)
{
    if(bEnable)
    {
        KCT_SetBitMask(COMMAND,  BFL_JBIT_POWERDOWN);
    }
    else
    {
        KCT_ClearBitMask(COMMAND, BFL_JBIT_POWERDOWN);
    }
    return SUCCEED_NZ3801;
}

/*******************************************************************************
---函数：unsigned char Nz3801_ActivateField(unsigned char activateField)
---功能：控制开关场强
---输入：activateField = NZ3801_TRUE，开启场强；
											 = NZ3801_FALSE，关闭场强；
---输出：
---备注：
*************************************************/
unsigned char Nz3801_ActivateField(unsigned char activateField)
{
    unsigned char reg_val;
#if FIELD_ONOFF_RETRY_EN
    unsigned char retry = 5;
    if(activateField)
    {
        while(retry--)
        {
            KCT_SetBitMask(TXCONTROL,0x03);
            reg_val = KCT_ReadReg(TXCONTROL);
            if((reg_val&0x03)==0x03)
            {
                break;
            }
        }
        if(retry==0)
        {
            return ERR_ONOFFFIELD;
        }
    }
    else
    {
        while(retry--)
        {
            KCT_ClearBitMask(TXCONTROL,0x03);
            reg_val = KCT_ReadReg(TXCONTROL);
            if((reg_val&0x03)==0x00)
            {
                break;
            }
        }
        if(retry==0)
        {
            return ERR_ONOFFFIELD;
        }
    }
#else
    if(activateField)
    {
        KCT_SetBitMask(TXCONTROL,0x03);
        reg_val = KCT_ReadReg(TXCONTROL);
        if((reg_val&0x03)!=0x03)
        {
           
            return ERR_ONOFFFIELD;
        }
    }
    else
    {
        KCT_ClearBitMask(TXCONTROL,0x03);
        reg_val = KCT_ReadReg(TXCONTROL);
        if((reg_val&0x03)!=0x00)
        {
            
            return ERR_ONOFFFIELD;
        }
    }
#endif
    return SUCCEED_NZ3801;
}

/*******************************************************************************
---函数：unsigned char Nz3801_Init(teCardType card)
---功能：根据输入的协议类型初始化Nz3801_寄存器
---输入：
---输出：
---备注：
*************************************************/



unsigned char Nz3801_Init_CPU(teCardType card)
{
	 // unsigned char temp=0;


	  #if NZ3802_AB
	    KCT_WriteReg(0x1C, 0x72);
    #endif
	
    KCT_WriteReg(COMMAND,COMMAND_SOFTRESET);
    KCT_WriteReg(COMMIEN,0x00);
    KCT_WriteReg(DIVIEN,0x00);//禁止所有中断
    KCT_WriteReg(COMMIRQ,0x3f);
    KCT_WriteReg(DIVIRQ,0x3f);//清除中断位标示
    KCT_ClearBitMask(STATUS2,8);//非m1加密模式

    KCT_SetRegExt(0x01,0x21);
    KCT_SetRegExt(0x03,0x04);
	 
    KCT_WriteReg(RXSEL,0x88);// 8个bit位后接收所有后续数据, 8*128fc，速度不为106时需调整
    if(card==CT_B)// TypeB
    {
        KCT_WriteReg(TXMODE,0x00|BFL_JBIT_CRCEN|BFL_JBIT_106KBPS|BFL_JBIT_TYPEB);
        //crc on /106k/ no inverted/TxMix off/RxFraming 14443b
        KCT_WriteReg(RXMODE,0x00|BFL_JBIT_CRCEN|BFL_JBIT_106KBPS|BFL_JBIT_TYPEB );
        //crc on /106k/ not valid received/RxMultiple inable/RxFraming 14443b
        KCT_WriteReg(TXAUTO,0x00); //no Force100ASK
        //no AutoRFOFF/no Force100ASK/ AutoWakeUp 0/CAOn/ InitialRFOn/Tx2RFAutoEn off/Tx1RFAutoEn off
        KCT_WriteReg(RXTRESHOLD,0x68 );	//0x43(0x64)
        //not sure 0  use osc  //  8f
        KCT_WriteReg(GSN, 0xfa);// 低四位改变调制深度，数值越小，调制深度越大；
        //改变调制深度，数值越小，调制深度越大
        //CWGsNOn/ModGsNOn
        KCT_WriteReg(MODGSP, reg29h_MODGSP);//#2(12.8%/)   // 0x2a

        KCT_WriteReg(TYPEBREG, 0xc0);
        //Initiator
        KCT_WriteReg(RFCFG, reg26h_RFCFG);		 //0x64
			
        KCT_WriteReg(DEMOD, 0X5D);  //19 0x10
        KCT_WriteReg(BITFRAMING, 0);
        CType = CT_B;
    }
    else // TypeA
    {
        KCT_WriteReg(TXMODE,0x00|BFL_JBIT_106KBPS|BFL_JBIT_TYPEA);
        //crc off /106k/ no inverted./TxMix off/RxFraming 14443A
        KCT_WriteReg(RXMODE,0x00|BFL_JBIT_106KBPS|BFL_JBIT_TYPEA);
        //crc off /106k/  valid received/RxMultiple inable/RxFraming 14443A
        KCT_WriteReg(TXAUTO,0X00|BFL_JBIT_FORCE100ASK);
        //no AutoRFOFF/ Force100ASK/ AutoWakeUp 0 /CAOn/InitialRFOn/ Tx2RFAutoEn off/ Tx1RFAutoEn off
        KCT_WriteReg(MODWIDTH, 0x26);
        //miler
        KCT_WriteReg(RXTRESHOLD, reg18h_RXTRESHOLD);// 0x8b 0x9b 0x7b(0x66)

        //not sure 0  use osc    // 8f
        KCT_WriteReg(GSN, reg27h_GSN);    // 0x8f  0xff
        //Initiator
        KCT_WriteReg(RFCFG, reg26h_RFCFG);	 // 0x78
        KCT_WriteReg(BITFRAMING, 0);
        KCT_WriteReg(TYPEBREG, 0x00);

        CType = CT_A;
    }
    KCT_WriteReg(CONTROL, 0x40);    // stop timer
    KCT_WriteReg(CWGSP, reg28h_CWGSP);      // 10  1f  3f
    KCT_SetBitMask(TXCONTROL, 0x83);//KCT_ClearBitMask(TXCONTROL,0x03);

    KCT_SetBitMask(TXSEL, reg16h_TXSEL);  //MFOUT Test  14 15
    KCT_SetBitMask(ANALOGTEST, reg38h_ANALOGTEST);// AUX Test 24 56 cd
   
		//temp=KCT_ReadReg(SERIALSPEED);
	
	 #if NZ3802_AB
	    KCT_WriteReg(0x1C, 0x72);
    #endif
		
    return(SUCCEED_NZ3801);
}


unsigned char Nz3801_Xfer(eCmd command,
                               const unsigned char *request, unsigned char requestLength, unsigned char txalign,
                               unsigned char *response, unsigned char *responseLength, unsigned char rxalign)
{
    unsigned long  t=0, tmp=0;
    unsigned char  i=0;
    unsigned char  status=0;
    unsigned char  rec_times=0;
    unsigned char  noise=0, timerout=0, ov=0; //标志


    if( requestLength==0 || !request || !response || !responseLength )
		{
        return ERR_PARA;
		}
		
    if(CType==CT_B && command==TA_IBLOCK)
    {
        command=TB_IBLOCK;
    }

    KCT_StopCmd();
	  KCT_ClearFifo(); 
    KCT_ClearFlag();

    if(CType==CT_A)
    {
        KCT_SetPARITY(NZ3801_TRUE);
    }
    else
    {
        KCT_SetPARITY(NZ3801_FALSE);
    }
    if(command==TA_REQA || command==TA_WUPA || command==TA_ANT)
    {
        KCT_SetCRC(NZ3801_FALSE);
    }
    else
    {
        KCT_SetCRC(NZ3801_TRUE);
    }
    // 设置发送/接收位对齐方式
    if(txalign>=8 || rxalign>=8)
    {
        return ERR_PARA;
    }
    if(txalign!=0 || rxalign!=0)
    {
        KCT_WriteReg(BITFRAMING, (rxalign<<4)|txalign);
    }
    else
    {
        KCT_WriteReg(BITFRAMING,0);
    }
    KCT_DelayMs(1);
    // 设置timeout时间
    if(command==TA_REQA||command==TA_WUPA||command==TA_ANT||command==TA_SELECT||command==TA_HLTA)
    {
        t = 9*128+20;
    }
    else if(command==TA_RATS || command==TA_PPS)
    {
        t = 559*128+20;
    }
    else if(command==TB_WUPB)
    {
        t = 7680;
    }
    else    // I BLOCK
    {
        if((request[0]&0xf0)==0xf0 && command==TA_RSBLOCK) // wtx
        {
            t = (request[1]&0x3f)*power(FWI);
        }
        else
        {
            t = power(FWI);
        }
        t = t * (4096) + 49152;        // 2.5,20160714

        tmp = (4096)*power(14)+49152; // 2.5,20160714 两边文档不一致，理论上不应该 +384
        if(t>tmp)
        {
            t = tmp;
        }
    }
    Nz3801_SetTimer(t);

    // 写入所有数据，64为本地fifo大小，len的大小由PICC和外部分帧决定
    if((TA_IBLOCK==command)||(command==TB_IBLOCK))
    {
        KCT_WriteReg(FIFODATA, PCB);
    }
    for(i=0; i<requestLength&&i<64-3; i++)
    {
        KCT_WriteReg(FIFODATA, request[i]);
    }
    KCT_StartCmd(COMMAND_TRANSCEIVE);

    KCT_SetBitMask(BITFRAMING,BFL_JBIT_STARTSEND);// 启动发送和接收

    while(i<requestLength) // 写入后续数据
    {
        if(KCT_ReadReg(FIFOLEVEL) < (64-2))
        {
            KCT_WriteReg(FIFODATA, request[i++]);
        }
    }
    while((KCT_ReadReg(COMMIRQ)&BFL_JBIT_TXI)==0);  // 数据发送完成  xu.kai 20170328
    KCT_WriteReg(COMMIRQ, 0x01);

    rec_times = 0;
REC:
    while(1)
    {
        if((KCT_ReadReg(STATUS2)&0x07) <= 0x01)// 停止接受数据
        {
            break;
        }
        if(KCT_ReadReg(FIFOLEVEL) != 0)
        {
            break;
        }
        if((KCT_ReadReg(COMMIRQ) & BFL_JBIT_TIMERI))// 超时
        {
            if(timerout==0)// timerout后再延时400us继续收后续数据
            {
                timerout = 1;
                KCT_DelayUs(500);
                continue;
            }
            KCT_SetBitMask(CONTROL, BFL_JBIT_TSTOPNOW);
            return ERR_TIMEOUT;
        }
    }
    *responseLength = 0;
    while((i=(KCT_ReadReg(STATUS2))&0x07)>0x01) // 在接收数据
    {
        while(KCT_ReadReg(FIFOLEVEL)!=0)
        {
            response[*responseLength] = KCT_ReadReg(FIFODATA);
            if(*responseLength >= 256-2)      // 2字节CRC，只有I-BLOCK才会存在这种情况
                ov = 1;
            else
                *responseLength += 1;
        }
    }
    while(KCT_ReadReg(FIFOLEVEL)!=0) // 接收后续数据
    {
        response[*responseLength] = KCT_ReadReg(FIFODATA);
        if(*responseLength >= 256-2)      // 2字节CRC，只有I-BLOCK才会存在这种情况
            ov = 1;
        else
            *responseLength += 1;
    }
    if(ov)
    {
        KCT_SetBitMask(CONTROL, BFL_JBIT_TSTOPNOW);
        return ERR_OVERLOAD;
    }
    if(command==TA_REQA||command==TA_WUPA||command==TA_HLTA||command==TA_ANT||command==TA_SELECT||command==TB_WUPB)
    {
        KCT_SetBitMask(CONTROL, BFL_JBIT_TSTOPNOW);
        return SUCCEED_NZ3801;
    }
    // 判断是否为noise，后续noise判断只有RATS和I-BLOCK
    noise = 0;
    status = KCT_ReadReg(REGERROR);
    KCT_WriteReg(RXSEL, 0x82);
    KCT_StartCmd(COMMAND_RECEIVE);//need?
    KCT_SetBitMask(CONTROL,BFL_JBIT_TSTARTNOW);

    // CRC错
    if((status&BFL_JBIT_CRCERR)!=0)
    {
        if((command==TB_ATTRIB) && (*responseLength==0) && ((status&BFL_JBIT_PROTERR)==0))// Protocol error，无CRC的情况
        {
            noise = 1;// 2.5,2060714,return ERR_PROTOCOL;
        }
        if(*responseLength<4)
        {
            noise = 1;
        }
    }
    if(*responseLength<2)// +CRC，小于4字节
    {
        if((status&(BFL_JBIT_CRCERR|BFL_JBIT_PARITYERR))!=0)// CRC或者奇偶错
            noise = 1;
    }
    if((status&BFL_JBIT_COLLERR)!=0)// 位冲突
    {
        noise = 1;
    }
    if(*responseLength==0)// 没有收到有效数据
    {
        noise = 1;
    }
    if(((status&BFL_JBIT_PROTERR)!=0) && (*responseLength<2))// 接收开始时间 <FDT(picc,min)
    {
        noise = 1;
    }
    if(noise)
    {
        KCT_WriteReg(COMMIRQ, 0x7e);
        KCT_WriteReg(DIVIRQ, 0x7f);
        KCT_ClearFifo();
        if(++rec_times<5)
            goto REC;
        else
            return ERR_TIMEOUT;
    }
    else
    {
        if ((status&(BFL_JBIT_COLLERR|BFL_JBIT_PROTERR|BFL_JBIT_PARITYERR|BFL_JBIT_CRCERR))!=0)
        {
            KCT_WriteReg(COMMAND, 0);
            KCT_SetBitMask(CONTROL, BFL_JBIT_TSTOPNOW);
            return ERR_PARA;
        }
    }
    KCT_WriteReg(COMMAND, 0);
    KCT_SetBitMask(CONTROL, BFL_JBIT_TSTOPNOW);

    return SUCCEED_NZ3801;
}

//--------------------Start of ISO14443A -------------
/*******************************************************************************
---函数：static unsigned char iso14443AVerifyBcc(const unsigned char* uid, unsigned char length, unsigned char bcc)
---功能：
---输入：
---输出：
---备注：没有在外申明
*************************************************/
static unsigned char iso14443AVerifyBcc(const unsigned char* uid, unsigned char length, unsigned char bcc)
{
    unsigned char actbcc = 0;

    do {
        length--;
        actbcc ^= uid[length];
    } while (length);

    if (actbcc != bcc)
    {
        return ERR_CRC;
    }
    return SUCCEED_NZ3801;
}

/*******************************************************************************
---函数：static unsigned char iso14443ADoAntiCollisionLoop(unsigned char UIDLen,iso14443AProximityCard_t *card)
---功能：
---输入：
---输出：
---备注：没有在外申明
*************************************************/
static unsigned char iso14443ADoAntiCollisionLoop(unsigned char UIDLen,iso14443AProximityCard_t *card)
{
    unsigned char cscs[ISO14443A_MAX_CASCADE_LEVELS][ISO14443A_CASCADE_LENGTH];
    unsigned char cl = ISO14443A_CMD_SELECT_CL1;
    unsigned char regcoll;
    unsigned char regerr;
    unsigned char bytes = 0;
    unsigned char bits = 0;
    unsigned char txlength = 2;
    unsigned char rxlength;
    unsigned char saklength;
    unsigned char savecoll = 0;
    unsigned char status;
    unsigned char i;
    unsigned char* buf;

    KCT_ClearBitMask(STATUS2, 0x08);
    KCT_WriteReg(BITFRAMING,  0x00);
    KCT_ClearBitMask(COLL,    0x80);

    card->cascadeLevels = 0;
    card->collision     = NZ3801_FALSE;
    card->actlength     = 0;
    buf    = cscs[card->cascadeLevels];
    buf[1] = 0x20;

    do
    {
        buf[0] = cl;
        status = Nz3801_Xfer(TA_ANT,buf,txlength,bits,buf+2,&rxlength,bits);
        if(status!=SUCCEED_NZ3801) goto out;

        /* now check for collision */
        regcoll = KCT_ReadReg(COLL);/* read out collision register */
        regerr = KCT_ReadReg(REGERROR);
        if( (!(regcoll&0x20)) || (regerr&BFL_JBIT_COLLERR) )
        {
            card->collision = NZ3801_TRUE;
            /* save the colision byte */
            savecoll = buf[2+(regcoll&0x1F)/8];
            bits = (regcoll&0x1F) % 8;
            bytes = (regcoll&0x1F) / 8;
            /* FIXME handle c_pb collision in parity bit */
            /* update NVB. Add 2 bytes for SELECT and NVB itself */
            buf[1] = 0x20 + (bytes<<4) + (bits&0xF);
            if(bits!=0)
            {
                txlength = bytes + 2 + 1;
            }
            else
            {
                txlength = bytes + 2;
            }
        }
        else
        {
            /* got a frame w/o collision - store the uid and check for CT */
            /* answer with complete uid and check for SAK. */
            buf[1] = 0x70;
            buf[0] = cl;
            buf[2+bytes] |= savecoll;
            txlength = rxlength + 2;
            status = Nz3801_Xfer(TA_SELECT, buf,txlength,0,&card->sak[card->cascadeLevels],&saklength,0);
            if(status!=SUCCEED_NZ3801) goto out;
					
            if(!KCT_CrcOK())
            {
                status = ERR_DATA;
                goto out;
            }
            if(saklength!=1)
            {
                status = ERR_DATA;
                goto out;
            }

            if (card->sak[card->cascadeLevels] & 0x4)
            {
                /* reset variables for next cascading level */
                txlength = 2;
                bytes = 0;
                bits = 0;

                if (ISO14443A_CMD_SELECT_CL1 == cl)
                {
                    cl = ISO14443A_CMD_SELECT_CL2;
                }
                else if (ISO14443A_CMD_SELECT_CL2 == cl)
                {
                    cl = ISO14443A_CMD_SELECT_CL3;
                }
                else
                {
                    /* more than 3 cascading levels are not possible ! */
                    status = ERR_COLL;
                    goto out;
                }
            }
            else
            {
                card->cascadeLevels++;
                break;
            }
            card->cascadeLevels++;
            buf = cscs[card->cascadeLevels];
            buf[0] = cl;
            buf[1] = 0x20;
        } /* no collision detected */
    } while (card->cascadeLevels <= ISO14443A_MAX_CASCADE_LEVELS);

    /* do final checks... */
    for (i = 0; i< card->cascadeLevels; i++)
    {
        status = iso14443AVerifyBcc(&cscs[i][2], 4, cscs[i][6]);
        if(status!=SUCCEED_NZ3801) goto out;
    }
    /* extract pure uid */
    switch (card->cascadeLevels)
    {
    case 3:
        mem_copy(card->uid+6, cscs[2]+2, 4);
        mem_copy(card->uid+3, cscs[1]+3, 3);
        mem_copy(card->uid+0, cscs[0]+3, 3);
        card->actlength = 10;
        break;
    case 2:
        mem_copy(card->uid+3, cscs[1]+2, 4);
        mem_copy(card->uid+0, cscs[0]+3, 3);
        card->actlength = 7;
        break;
    case 1:
        mem_copy(card->uid+0, cscs[0]+2, 4);
        card->actlength = 4;
        break;
    default:
        status = ERR_COLL;
        goto out;
    }
out:
    KCT_SetBitMask(COLL, 0x80);
    return status;
}

/*******************************************************************************
---函数：unsigned char iso14443ASendHlta(void)
---功能：命令已激活的卡片进入休眠状态，停卡
---输入：none
---输出：result
---备注：
*************************************************/
unsigned char iso14443ASendHlta(void)
{
    unsigned char status;
    unsigned char buf[2];
    unsigned char rec[256];
    unsigned char len;

    /* send HLTA command */
    buf[0] = ISO14443A_CMD_HLTA;
    buf[1] = 0;
    status = Nz3801_Xfer(TA_HLTA, buf, 2, 0, rec, &len, 0);
    return status;
}

/*******************************************************************************
---函数：unsigned char iso14443ASelect(iso14443ACommand_t cmd, iso14443AProximityCard_t* card)
---功能：A卡选卡函数，执行完初始化之后就可以操作选卡
---输入： cmd = 0x26[ISO14443A_CMD_REQA] - 寻未进入休眠状态的卡
							= 0x52[ISO14443A_CMD_WUPA] - 寻所有状态的卡
---输出：iso14443AProximityCard_t* card，寻卡成功时配置如下三个参数:
				card->atqa  : 卡类型
				card->uid	:		卡序列号
				card->sak :		卡SAK
---备注：在一个感应区域内的同一时刻，只可能有一张TypaA卡处于被选中状态
*************************************************/
unsigned char iso14443ASelect(iso14443ACommand_t cmd, iso14443AProximityCard_t* card)
{
    unsigned char status = SUCCEED_NZ3801;
    unsigned char buf[2];
    unsigned char atqalen;
    unsigned char UIDLen;

    if( (cmd!=ISO14443A_CMD_REQA) && (cmd!=ISO14443A_CMD_WUPA) )
        return ERR_PARA;

    buf[0] = cmd;
		
    status = Nz3801_Xfer(TA_WUPA, buf, 1, 7, card->atqa, &atqalen, 0);
		
    if(status==ERR_TIMEOUT)
    {
        if( KCT_FlagOK() )
        {
            return ERR_TIMEOUT;
        }
    }
    if(status!=SUCCEED_NZ3801)
    {
        return ERR_PARA;
    }
    if(atqalen!=2) // 16 bits ?
    {
        return ERR_DATA;
    }

    if(!KCT_FlagOK()||!KCT_CrcOK())
    {
        return ERR_DATA;
    }

    if(card->atqa[1]==0xf0)// 2.5,20160714
    {
        return ERR_DATA;
    }
    UIDLen = ((card->atqa[0])>>6)&0x3;
    status = iso14443ADoAntiCollisionLoop(UIDLen,card);

    return status;
}

/*******************************************************************************
---函数：unsigned char iso14443AEnterProtocolMode(iso14443AProximityCard_t* card, unsigned char* answer, unsigned char* length)
---功能：请求RATS，激活A卡，进入ISO14443-4应用层
---输入：card ->fsdi -PCD帧的最大长度
				 card ->CID = 0
---输出：card -> fsci - PICC帧的最大长度
				 card -> sfgi - PICC应答ATS后下一帧的保护时间
				 card -> TA -收发位速率
				 answer:卡应答ATS数据
				 length: 返回的ATS长度
---备注：
*************************************************/
unsigned char iso14443AEnterProtocolMode(iso14443AProximityCard_t* card, unsigned char* answer, unsigned char* length)
{
    unsigned char status;
    unsigned char buf[2];
    unsigned char TA, TB, TC;
    unsigned int SFGT = 0;

    if(!answer || !length)
        return ERR_PARA;

    /* send RATS command */
    buf[0] = ISO14443A_CMD_RATS;
    buf[1] = (card->fsdi<<4) | 0x0; // = 0x80 : FSD=256,cid=0

    BlockNum = 0;
    FWI = 4;
    FSD = 256;
    FSC = 32;
    CID = 0;
    NAD = 0;
    status = Nz3801_Xfer(TA_RATS, buf, 2, 0, answer, length, 0);
		
    if(status==ERR_TIMEOUT) // 直接超时
        return ERR_TIMEOUT;
    if(status!=SUCCEED_NZ3801)
        return ERR_PARA;
    if(!KCT_FlagOK())
        return ERR_PARA;
    if(!KCT_CrcOK())
        return ERR_DATA;

    if(answer[0]==0 || answer[0]!=*length)
        return ERR_PROTOCOL;
    if(answer[0]==1)  // 无T0，无后续数据
        return SUCCEED_NZ3801;

    TA = TB = TC = 0;
    if((answer[1]&0x10)!=0) TA = 1;
    if((answer[1]&0x20)!=0) TB = 1;
    if((answer[1]&0x40)!=0) TC = 1;
    if( answer[0]<(TA+TB+TC+2))
        return ERR_PROTOCOL;

    card->fsci = answer[1]&0x0f;
    if(card->fsci>=8)
        FSC = 256;
    else
        FSC = FSCTab[card->fsci];

    // TA中DR/DS可同时为0，所以该字节任何数据都是正确的
    if(TA)
    {
       card->TA = answer[2];//详细见-4协议
    }
    // TB值
    if(TB)
    {
        card->fwi = (answer[2+TA]>>4)&0xf;
        card->sfgi = answer[2+TA]&0xf;
        SFGT = card->sfgi;
        if(SFGT==15)
        {
            card->sfgi = 0;
            SFGT = 0;
        }
        if(card->fwi==15)
        {
            card->fwi = 4;
        }
        FWI = card->fwi;
        if(card->fwi>15 || SFGT>15)
        {
            return ERR_PROTOCOL;
        }
    }
    // TC值没有限制，所有值合法
    if(TC)
    {
        NAD = answer[2+TA+TB]&0x01;
        CID = (answer[2+TA+TB]>>1)&0x01;
    }
    if(SFGT)// 延时SFGT
    {
        KCT_ClearFlag();
        Nz3801_SetTimer((256*16+384)*power(SFGT));
        KCT_SetBitMask(CONTROL, BFL_JBIT_TSTARTNOW);  // 手动启动
        while((KCT_ReadReg(COMMIRQ)&BFL_JBIT_TIMERI) == 0);
    }
    return(SUCCEED_NZ3801);
}

/*******************************************************************************
---函数：unsigned char iso14443ASendProtocolAndParameterSelection(unsigned char pps1)
---功能：A 协议和参数选择，如PPS，在-4层协议需要用到
---输入：PPS1 - 协商传输位速率
					PPS1 = 0x0, 选择默认106KBd			PPS1 = 0x05, 选择212KBd
					PPS1 = 0x0A, 选择424KBd					PPS1 = 0x0F，选择848KBd
---输出：result
---备注：
*************************************************/
unsigned char iso14443ASendProtocolAndParameterSelection(unsigned char pps1)
{
    unsigned char  status;
    unsigned char  buf[3],rec[256];
    unsigned char  len;

   /* send PPS command */
    buf[0] = ISO14443A_CMD_PPSS | 0x0;
    buf[1] = 0x11;
    buf[2] = pps1;
    status = Nz3801_Xfer(TA_PPS, buf, 3, 0, rec, &len, 0);
    if(status==ERR_TIMEOUT) // 直接超时
        return ERR_TIMEOUT;
    if(status!=SUCCEED_NZ3801)
        return ERR_PARA;
    if(!KCT_FlagOK())
        return ERR_PARA;
    if(!KCT_CrcOK())
        return ERR_DATA;

    if(rec[0]!=buf[0] || len!=1)
        return ERR_PROTOCOL;

    return(SUCCEED_NZ3801);
}



/*******************************************************************************
---函数：unsigned char iso14443BSendHltb(iso14443BProximityCard_t* card)
---功能：命令激活的卡片进入休眠状态，停卡
---输入：
---输出：
---备注：
********************************************************************************/


unsigned char iso14443BSendHltb(iso14443BProximityCard_t* card)
{
    unsigned char status;
    unsigned char actlength;
    unsigned char buf[5];

    buf[0] = ISO14443B_CMD_HLTB;
    mem_copy(&buf[1], card->pupi, 4);

    status = Nz3801_Xfer(TB_HLTB, buf, 5, 0, buf, &actlength, 0);
    if ((ERR_TIMEOUT == status) || (buf[0] != 0x0))
    {
        status = ERR_DATA;
    }
    return status;
}

/*******************************************************************************
---函数：
---功能：TypeB 寻卡，防碰撞选卡操作
---输入：cmd =  0x00[ISO14443B_CMD_REQB] - 寻未进入休眠状态的卡
						 =  0x08[ISO14443B_CMD_WUPB] - 寻所有状态的卡
				 afi ： Type B应用族识别符，具体见ISO14443协议
---输出：card -> pupi -Type B伪唯一PICC卡标识符
				 card -> applicationData - 应用数据，用于通知PCD在PICC当前安装了哪些应用
				 card -> protocolInfo - 指示了卡所支持的参数，如帧最大长度/帧等待时间/位速率
---备注：在一个感应区域内的同一时刻，只可有一张B卡处于选中状态
*********************************************************************************/

unsigned char iso14443BSelect(iso14443BCommand_t cmd,
                              iso14443BProximityCard_t* card,
                              unsigned char afi)
{
    unsigned char status;
    unsigned char buf[3];
    unsigned char actlength;
    unsigned char fsci;

    if( (cmd!=ISO14443B_CMD_REQB) && (cmd!=ISO14443B_CMD_WUPB) )
        return ERR_PARA;

    buf[0] = ISO14443B_PARAM_APF;//"\x05\x00\x08"
    buf[1] = afi;
    buf[2] = cmd;
    status = Nz3801_Xfer(TB_WUPB, buf, 3, 0, (unsigned char*)card, &actlength, 0);
    if(status==ERR_TIMEOUT)
    {
        if(KCT_FlagOK())
        {
            return ERR_TIMEOUT;
        }
    }

    if((KCT_ReadReg(REGERROR)&(BFL_JBIT_COLLERR|BFL_JBIT_PROTERR))!=0)// 传输/帧错误
        return ERR_DATA;

    if(!((KCT_ReadReg(REGERROR)&(BFL_JBIT_CRCERR))==0))
        return ERR_DATA;

    if( SUCCEED_NZ3801==status && actlength!=0xc )
    {
        return ERR_DATA;
    }

    if( SUCCEED_NZ3801!=status )
        return ERR_DATA;

    if(card->atqb!=0x50)
        return ERR_DATA;

    fsci = (card->protocolInfo[1]>>4) & 0x0f;
    if(fsci>=8)
        FSC = 256;
    else
        FSC = FSCTab[fsci];

    FWI = (card->protocolInfo[2]>>4) & 0x0f;
    if(FWI==0x0f) // 2.5,20160714
        FWI = 4;

    return status;
}

/*******************************************************************************
---函数：iso14443BEnterProtocolMode
---功能：Attrib命令，激活B卡，进入ISO14443-4应用层
---输入：	card -> pupi, TypeB伪唯一 PICC卡标识符，通过iso14443BSelect()选卡获取
					param: baok 4个参数，具体见协议
---输出：	answer: AttriB应答数据，包含MBLI和CID
---备注：
*******************************************************************************/

unsigned char iso14443BEnterProtocolMode(iso14443BProximityCard_t* card,
        iso14443BAttribParameter_t* param,
        iso14443BAttribAnswer_t* answer)
{
    unsigned char status;
    unsigned char actlength;
    unsigned char buf[9];

    if(!answer)
        return ERR_PARA;

    buf[0] = ISO14443B_CMD_ATTRIB;
    mem_copy(&buf[1], card->pupi, 4);
    mem_copy(&buf[5], (unsigned char*)param, sizeof(iso14443BAttribParameter_t));//"\x00\x08\x01\x00"
    status = Nz3801_Xfer(TB_ATTRIB, buf, 9, 0, buf, &actlength, 0);
    if(status==ERR_TIMEOUT)
        return ERR_TIMEOUT;
    if(status!=SUCCEED_NZ3801)
        return ERR_PARA;
    if (actlength!=1)
        status = ERR_DATA;
    if((buf[0]&0x0f)!=0)
        return ERR_DATA;

    answer->mbli = (buf[0] >> 4) & 0xf;
    answer->cid = buf[0] & 0xf;
    BlockNum = 0;

    return status;
}


unsigned char xbuf[512]={0};
unsigned int  xLen=0;

//extern void PasswordERR_display(void);
extern unsigned int TouchTimer64ms;
extern void SysBasetimer_Start(unsigned int);
unsigned char SN[4];
extern unsigned char Find_FobKey(void);


unsigned char Find_FobKey(void)
{
	 unsigned char i=0,j=0, k=0, value=0;

	 for(j=0; j<CPUFOB_PAGEMAX; j++ )
	 {
		 CpuFobList_Readflash(j);
		 
		 for(i=0; (i<CPUFOB_MAX)&&(k==0); i++)
		 {
			   if  ( CPUFobKey_List.CpuFobList[i].Fob_SN[0]==SN[3] && 
					 CPUFobKey_List.CpuFobList[i].Fob_SN[1]==SN[2] &&
					 CPUFobKey_List.CpuFobList[i].Fob_SN[2]==SN[1] &&
				     CPUFobKey_List.CpuFobList[i].Fob_SN[3]==SN[0]
				   )
				 {
				   for(k=0; k<16; k++)
				   {
						   ExteralKey2[k]= CPUFobKey_List.CpuFobList[i].Fob_KeyArray[k];
				   }
					 
					 for(k=0; k<8; k++)
				    {
						   ExteralKey2[16+k]= CPUFobKey_List.CpuFobList[i].Fob_KeyArray[k];
					}
					 
				  value=1;
				 }
		 }
	 }
	 if(value==1)
	 {
		  return(1);
	 }
	 
	 return(0);
}

unsigned char ActionStateSuccFail;
extern unsigned char PT_SN[4];
extern void AddLogsStore(unsigned char,  unsigned char);

extern void HC595_LedWriteData(unsigned int);

uint8 FobType;
uint8 atqatemp[2];

unsigned char Iso14443ADemonstrate(void)
{
    iso14443AProximityCard_t gvCardA;
	
    unsigned char status=0, len=0;
    unsigned char value=0;
	//uint8 atqatemp[2];
	uint8 ret;
	
    Nz3801_HwReset();
    Nz3801_Init_CPU(CT_A);	
    status = Nz3801_ActivateField(NZ3801_TRUE);	//激活场强
	 
    if(status != SUCCEED_NZ3801)
    {
        return(status);
    }
    KCT_DelayMs(2);		//针对银行卡等CPU卡，此处需要增加延时操作给卡片充电，测试时间不能低于2ms，具体时间根据卡片功耗情况增加；

		status = iso14443ASelect(ISO14443A_CMD_WUPA,  &gvCardA);
		Sys_wdt_feed();
		
		memcpy( atqatemp ,gvCardA.atqa ,2 );

		if(status == SUCCEED_NZ3801)
		{
			gvCardA.fsdi = 8;
			status = iso14443AEnterProtocolMode(&gvCardA,  xbuf,  &len);

			memcpy( SN, gvCardA.uid,sizeof(SN));

			if ( 0x08 == gvCardA.atqa[0] )         //  cpufob
			{
			    FobType = KEY_TYPE_CPUCARD;
				if ( 1 == Find_FobKey())
				{
					if ( 0 == SelectMainFile())
					{
					  return CARD_CPU_APDU_ERR;
					}
					
					value = ExternalAuthenticate2();
					
					if ( value == 1 )
				 	{
				 		ret = CARD_TYPE_READ_OK;
				 	}
					else if ( value == CARD_CPU_LOCK_ERR )
					{
					    ret = CARD_CPU_LOCK_ERR;
					}
					else
					{
						ret = CARD_CPU_AUTH_ERR;
					}
				}
				else
				{
					ret = CARD_CPU_FINDKEY_ERR;
				}
			}
			else if ( 0x04 == gvCardA.atqa[0] )    //  M1fob
			{
			    FobType = KEY_TYPE_CARD;
				ret = CARD_TYPE_READ_OK; 
			}
		}
		else
		{
			ret = CARD_READ_NZ3801_ERR;
		}

		Nz3801_ActivateField(NZ3801_FALSE);
		
    return(ret);
}

/*******************************************************************************
---函数：unsigned char Iso14443BDemonstrate(void)
---功能：B卡寻卡交互测试代码
---输入：
---输出：
---备注：
*************************************************/
unsigned char Iso14443BDemonstrate(void)
{
    static iso14443BProximityCard_t    gvCardB;
    static iso14443BAttribParameter_t  gvParam;
    static iso14443BAttribAnswer_t     gvAnswer;
	
    const unsigned char iAPDU_GetID[5] = {0x00,0x36,0x00,0x00,0x08}; 	//获取2代身份证ID    {0x00,0x36,0x00,0x00,0x08};

    unsigned char status;
    unsigned char RspLen;
    unsigned char RspBuf[256];
		
    Nz3801_HwReset();
    Nz3801_Init_CPU(CT_B);
    status = Nz3801_ActivateField(NZ3801_TRUE);
		
    if(status!=SUCCEED_NZ3801)
    {
        return status;
    }
//    KCT_DelayMs(2);		//根据卡片情况延长场强时间，测试Awen身份证不需要延时即可寻到ID

    status = iso14443BSelect(ISO14443B_CMD_WUPB, &gvCardB, 0x00);
		
    if(status==SUCCEED_NZ3801)
    {
        gvParam.param1 = 0x00;
        gvParam.param2 = 0x08;
        gvParam.param3 = 0x01;
        gvParam.param4 = 0x00;
        status = iso14443BEnterProtocolMode(&gvCardB,&gvParam,&gvAnswer);
    }
    if(status==SUCCEED_NZ3801)
    {
       //二代身份证-非标APDU
       status = Nz3801_Xfer(TB_xBLOCK,(unsigned char*)iAPDU_GetID,sizeof(iAPDU_GetID),0,RspBuf,&RspLen,0);
    }
    status = Nz3801_ActivateField(NZ3801_FALSE);
    if(status!=SUCCEED_NZ3801)
    {		
        return status;
    }
    return status;
}



unsigned char nz3801Transceive(eCmd command, 
           const unsigned char *request,  unsigned char requestLength,    unsigned char txalign, 
						     unsigned char *response, unsigned char *responseLength,  unsigned char rxalign)
{
	unsigned char status=0;
	
  status=Nz3801_Xfer(command,  
	                   request,  requestLength,  txalign,
                   	 response, responseLength, rxalign);
	return(status);
}

unsigned char nz3801IBLOCK(const unsigned char *inf, unsigned int infLength, unsigned char *response, unsigned int *responseLength)
{
	unsigned char rec[256];
	unsigned int sp, l;  // sp 已发送长度,也可用来表示发送pcb的位置;
	unsigned char rlen;
	unsigned char r;
	unsigned char RB[5];
	unsigned char timer, re;
	unsigned char tmp;
	unsigned char brec;	// 是否在接收数据状态
	unsigned char Rerr;

	#define ACK		0
	#define NAK		0x10
	#define SendRB(type)	if(++timer>=3+Rerr) return ERR_TIMEOUT; RB[0]=0xa2|type|BlockNum; r = nz3801Transceive(TA_RSBLOCK, RB, 1, 0, rec, &rlen, 0);
	#define SendFinish()	((PCB&BIT4)==0)

	/*mem_copy(send+1, (u8*)"\x00\xa4\x04\x00\x0e", 5);
	slen = 5;
	sp = strlen("2PAY.SYS.DDF01")+1;
	mem_copy(send+1+5, "2PAY.SYS.DDF01", sp);
	slen += sp;*/

	*responseLength = 0;
	sp = 0;
	while(1)
	{
		Rerr = 0;
		// 发送过程
		if(infLength-sp+1+2>FSC)	// 剩余数据长度+PCB+CRC
		{
			l = FSC-2;			// 最大值-CRC
			PCB = 0x12;
		}
		else
		{
			l = infLength-sp+1;		// PCB
			PCB = 0x02;
		}
		PCB |= BlockNum;
		timer = re = 0;

SEND:
		r = nz3801Transceive(TA_IBLOCK, inf+sp, l-1, 0, rec, &rlen, 0);
		brec = 0;
        
CHECK:
		if(r==ERR_TIMEOUT)
		{
			if(brec)
			{
				SendRB(ACK);
			}
			else
			{
				SendRB(NAK);
			}

			goto CHECK;
		}
		if(r==ERR_OVERLOAD)		// 接收数据超长
		{
			return ERR_PROTOCOL;
		}
		if(r==ERR_PARA)
		{
			if(brec)
			{
				SendRB(ACK);
			}
			else
			{
				SendRB(NAK);
			}

			goto CHECK;
		}

		if(rlen==0) // xu.kai added 20170331
		{
				return ERR_PROTOCOL;
		}
        
		// 协议错误
		tmp = (rec[0]>>6)&0x3;
		if(tmp==0x00)	// I BLOCK
		{
			if((rec[0]&BIT5)!=0)	
                return ERR_PROTOCOL;// b6=1,2.5,20160714
			if((rec[0]&BIT1)==0)	
                return ERR_PROTOCOL;// b2=0
			if((rec[0]&(BIT2|BIT3))!=0)	
                return ERR_PROTOCOL;// 有CID, NAD
			if((rec[0]&0x01)!=BlockNum)	
                return ERR_PROTOCOL;// 块号不对
			if(!SendFinish())	
                return ERR_PROTOCOL;// 有后续数据要发送，不应该回I BLOCK
			Rerr = 0;
			timer = 0;
			BlockNum = 1 - BlockNum;
			if(brec==0)	
      {
         infLength = 0;	// slen为接收数据位置
			}
      brec = 1;
			mem_copy(response+*responseLength, rec+1, rlen-1);
			*responseLength += (rlen-1);
			if((rec[0]&BIT4)==0)	// 无链接帧
			{
				return ERR_NONE;
			}
			else
			{
				SendRB(ACK);
				timer = 0;	// 正常ACK
				goto CHECK;
			}

		}
		else if(tmp==0x02)	// R BLOCK
		{
			Rerr = 1;
			if((rec[0]&(BIT2|BIT3))!=0)
				return ERR_PROTOCOL;
			if((rec[0]&BIT5)==0)
				return ERR_PROTOCOL;
			if((rec[0]&(BIT0|BIT1))==1)// 2.5,20160714
				return ERR_PROTOCOL;
			if((rec[0]&BIT4)!=0)		// NAK
				return ERR_PROTOCOL;
			else
			{
				if(brec)	
                   return ERR_PROTOCOL;	// 接收数据过程中, PICC是不应该发送ACK的
				if((rec[0]&0x01)!=BlockNum)
				{
					if(++re>=3) 
              return ERR_TIMEOUT;
					goto SEND;
				}
				if(SendFinish())	
                    return ERR_PROTOCOL;	// 此时应该收到数据
				BlockNum = 1-BlockNum;
				sp += (l-1);
				continue;	// 继续发送后续数据
			}
		}
		else if(tmp==0x03)	// S BLOCK
		{
			unsigned char t;
			unsigned char sb[2];
            
			t = (rec[0]>>4)&0x03;
			
			if(t==0)	
          return ERR_PROTOCOL;// DESELECT
			else if(t!=3)	
          return ERR_PROTOCOL; // 非WTX
			
			t = rec[0]&0x02;
			
			if(t==0)	
          return ERR_PROTOCOL;
			if(rlen!=2 || rec[1]==0||rec[1]>59)	
          return ERR_PROTOCOL;	// WTX=0
			
			sb[0] = 0xf2;
			sb[1] = rec[1];
			r = nz3801Transceive(TA_RSBLOCK, sb, 2, 0, rec, &rlen, 0);	// 发送wtx
			goto CHECK;
		}
		else
    {
			 return ERR_PROTOCOL;
		}
	}
}


unsigned char ApduTransceive(unsigned char  *inf,        unsigned int infLength, 
	                           unsigned char  *response,   unsigned int *responseLength)
{
    if(!inf || !response || !responseLength)
		{
        return ERR_DATA;
		}
    return nz3801IBLOCK(inf,  infLength,  response,  responseLength);    
}

void ClearBitMask(unsigned char reg,   unsigned char mask)
{
    char   tmp = 0x0;
    tmp = ReadRawRC(reg);
    WriteRawRC(reg, tmp & ~mask);  
}

void PcdAntennaOn(void)
{
    unsigned char   i;
    i = ReadRawRC(TxControlReg);
    if( !(i & 0x03) )
    {
        KCT_SetBitMask(TxControlReg, 0x03);
    }
}

void PcdAntennaOff(void)
{
    ClearBitMask(TxControlReg, 0x03);
}


uint8_t CertificationCPUFob(void)
{
	uint8_t Value=0;

	Nz3801_SoftPwrDown(0);  
	Value = Iso14443ADemonstrate();  
	Nz3801_SoftPwrDown(1); 

	return(Value);
}

void NZ3801_Open( void )
{ 
	nrf_gpio_cfg_output(NZ3801_CS);
	nrf_gpio_cfg_output(NZ3801_RST);

	SPI_CS_Disable();    //  High
	Nz3801RST_En();      //  low
}

void nz3801_Init(void)
{
	Nz3801RST_En();
    nrf_delay_ms(1);
	
    Nz3801RST_Dis();
    nrf_delay_ms(1);

#if NZ3802_AB     
     WriteRawRC(0x1C,0x72);    
#endif	

	WriteRawRC(CommandReg, PCD_RESETPHASE);   //0x01, 0x0F

    nrf_delay_us(800); //
    
    WriteRawRC(ModeReg,     0x3D);            
    WriteRawRC(TReloadRegL, 30);     
    WriteRawRC(TReloadRegH, 0);      
    WriteRawRC(TModeReg,      0x8D);       
    WriteRawRC(TPrescalerReg, 0x3E);  
    WriteRawRC(TxAutoReg,     0x40);	
    
   	#if NZ3802_AB
	     WriteRawRC(0x1C,0x72);
    #endif

   PcdAntennaOn();	
}

