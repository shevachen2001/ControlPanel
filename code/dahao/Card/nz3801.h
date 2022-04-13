
#ifndef __OPS_NZ3801_H_
#define __OPS_NZ3801_H_

//#include "Platform_Types.h"
//#include <stdio.h>	//Uart打印Debug信息需要用到
//#define SPI_CS_EN()    nrf_gpio_pin_clear(RC522_CS_PIN)     		//SPI使能
//#define SPI_CS_Dis()   nrf_gpio_pin_set(RC522_CS_PIN)         	//SPI关闭
//#define Nz3801RST_Dis()  nrf_gpio_pin_set(RC522_RST_PIN)	   //芯片工作
//#define Nz3801RST_En()   nrf_gpio_pin_clear(RC522_RST_PIN)    //芯片复位，低电平保持至少10ns

#define K_DELAY_MS(a)		HwSleepMs(a)				   //延时函数，毫秒级

/*================================================
---	Nz3801命令字
-----------------------------------------------*/
#define PCD_IDLE              0x00               //取消当前命令
#define PCD_AUTHENT           0x0E               //验证密钥
#define PCD_RECEIVE           0x08               //接收数据
#define PCD_TRANSMIT          0x04               //发送数据
#define PCD_TRANSCEIVE        0x0C               //发送并接收数据
#define PCD_RESETPHASE        0x0F               //复位
#define PCD_CALCCRC           0x03               //CRC计算

/*==================================================
---	M1卡命令字
-----------------------------------------------*/
#define PICC_REQIDL           0x26               //寻天线区内未进入休眠状态
#define PICC_REQALL           0x52               //寻天线区内全部卡
#define PICC_ANTICOLL1        0x93               //防冲撞
#define PICC_ANTICOLL2        0x95               //防冲撞

#define PICC_AUTHENT1A        0x60               //验证A密钥
#define PICC_AUTHENT1B        0x61               //验证B密钥
#define PICC_READ             0x30               //读块
#define PICC_WRITE            0xA0               //写块
#define PICC_DECREMENT        0xC0               //扣款
#define PICC_INCREMENT        0xC1               //充值
#define PICC_RESTORE          0xC2               //调块数据到缓冲区
#define PICC_TRANSFER         0xB0               //保存缓冲区中数据
#define PHAL_MFC_CMD_PERSOUID 0x40  					   //MF个性化ID指令
#define PICC_HALT             0x50               //休眠


/*==============================================
---	Nz3801 FIFO长度
-----------------------------------------------*/
#define DEF_FIFO_LENGTH       64    //FIFO size=64byte
#define MAXRLEN               32		//18

/*=============================================
---	Nz3801寄存器定义
-----------------------------------------------*/
//--- PAGE 0
#define     RFU00                 0x00
#define     CommandReg            0x01
#define     ComIEnReg             0x02
#define     DivlEnReg             0x03
#define     ComIrqReg             0x04
#define     DivIrqReg             0x05
#define     ErrorReg              0x06
#define     Status1Reg            0x07
#define     Status2Reg            0x08
#define     FIFODataReg           0x09
#define     FIFOLevelReg          0x0A
#define     WaterLevelReg         0x0B
#define     ControlReg            0x0C
#define     BitFramingReg         0x0D
#define     CollReg               0x0E
#define     RFU0F                 0x0F
//--- PAGE 1
#define     RFU10                 0x10
#define     ModeReg               0x11
#define     TxModeReg             0x12
#define     RxModeReg             0x13
#define     TxControlReg          0x14
#define     TxAutoReg             0x15
#define     TxSelReg              0x16
#define     RxSelReg              0x17
#define     RxThresholdReg        0x18
#define     DemodReg              0x19
#define     RFU1A                 0x1A
#define     RFU1B                 0x1B
#define     MifareReg             0x1C
#define     RFU1D                 0x1D
#define     RFU1E                 0x1E
#define     SerialSpeedReg        0x1F
//--- PAGE 2
#define     RFU20                 0x20
#define     CRCResultRegM         0x21
#define     CRCResultRegL         0x22
#define     RFU23                 0x23
#define     ModWidthReg           0x24
#define     RFU25                 0x25
#define     RFCfgReg              0x26
#define     GsNReg                0x27
#define     CWGsCfgReg            0x28
#define     ModGsCfgReg           0x29
#define     TModeReg              0x2A
#define     TPrescalerReg         0x2B
#define     TReloadRegH           0x2C
#define     TReloadRegL           0x2D
#define     TCounterValueRegH     0x2E
#define     TCounterValueRegL     0x2F
//--- PAGE 3
#define     RFU30                 0x30
#define     TestSel1Reg           0x31
#define     TestSel2Reg           0x32
#define     TestPinEnReg          0x33
#define     TestPinValueReg       0x34
#define     TestBusReg            0x35
#define     AutoTestReg           0x36
#define     VersionReg            0x37
#define     AnalogTestReg         0x38
#define     TestDAC1Reg           0x39
#define     TestDAC2Reg           0x3A
#define     TestADCReg            0x3B
#define     RFU3C                 0x3C
#define     RFU3D                 0x3D
#define     RFU3E                 0x3E
#define     RFU3F		  					  0x3F

/*================================================
---	卡类型代码定义
-----------------------------------------------*/
enum CartType
{
    CTYPE_A = 0,				// TypeA模式
    CTYPE_B							// TypeB模式
};



//#define 	RF_OK                 0
//#define 	RF_NOTAG_ERR          1
//#define 	RF_ERR                2
//#define Max_Len_NZ3801  10

enum
{
  ADC_NULL_FLAG = 0x00, 
  ADC_BAT_FLAG  = 0x01,
	ADC_FOB_FLAG  = 0x02,
};

//extern unsigned int  Nz3801AdcVoltageTab[Max_Len_NZ3801];
//extern unsigned char Nz3801TabLen;
//extern unsigned char IsCard(unsigned int *pVoltage);

//extern unsigned char SN[4]; 					//卡号
//unsigned char ReadRawRC_1(unsigned char   Address);

//void WriteRawRC_1(unsigned char   Address, unsigned char   value);
//void SetBitMask_1(unsigned char   reg,unsigned char   mask);
//void ClearBitMask_1(unsigned char   reg,unsigned char   mask);
//void PcdAntennaOn_1(void);
//void PcdAntennaOff_1(void);
//char PcdReset_1(void);
//char InitCardType(unsigned char   type);
//void Nz3801_Reset_1(void);
//void Nz3801_Init(void);
//void CalulateCRC_1(unsigned char *pIn ,unsigned char   len,unsigned char *pOut );
//char Nz3801_sCom_1(unsigned char   Command,
//                 unsigned char *pIn ,
//                 unsigned char   InLenByte,
//                 unsigned char *pOut ,
//                 unsigned char *pOutLenBit);
//char PcdRequest_1(unsigned char   req_code,unsigned char *pTagType);
//char PcdAnticoll_1(unsigned char *pSnr);
//char PcdSelect(unsigned char *pSnr);
//char PcdHalt_1(void);
//char PcdAuthState(unsigned char auth_mode,		//0x60-KeyA, 0x61-KeyB
//									unsigned char addr,					//块绝对地址
//									unsigned char *pKey,				//Key
//									unsigned char *pSnr);				//卡ID
//char PcdRead(unsigned char  addr,  unsigned char *p );
//char PcdWrite(unsigned char   addr,unsigned char *p );
//char PcdValue_1(unsigned char dd_mode,			// 0xC0-扣款，0xC1-充值
//							unsigned char addr,					// 钱包的块地址
//							unsigned char *pValue);			// 操作金额，低位在前

//char PcdBakValue_1(unsigned char sourceaddr, unsigned char goaladdr);
//unsigned char  eWallet_Test(void);
//unsigned char TestReadCard(void);


#endif
