/*****************************************************************************
|  File Name: sys.c
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

#include "SYSTEM\sys.h"
#include "Protocol\factInfo.h"
#include "KEY\key.h"
#include "SYSTEM\Sys_GenFun.h"
#include "Access\Access.h"
#include "Beep\Beep.h"
#include "Card\Rc522.h"
#include "Flash\Flash.h"
#include "Flash\Flash.h"
#include "Flash\Flash_Nrf.h"
#include "protocol\Proto_Analy.h"
#include "ble_dahao.h"
#include "Light\light.h"
#include "CustomCode\CustomCode.h"
#include "adc\adc_Nrf.h"
#include "SYSTEM\Sys_TimeBase.h"
#include "Access\LockKey.h"
#include "crc32.h"
#include "nrf_drv_common.h"
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "Card\Nz3801_AB.h"

extern void Wiegand_Init(void);
extern void uart_init(void);

Sys_StateType Sys_State;
uint16 Sys_TimeOutX16mS;
uint8  Sys_CheckFlag;

Sys_PataCfgType    Sys_PataCfg;

uint8 Sys_CheckUpFlag = 0;
uint8  Sys_McuRestFlag = 0;
uint8  Sys_StoreFlag = 0;
uint16 Sys_RstTimsoutx64ms = 0;
uint16 Sys_StoreTimsoutx64ms = 0;
uint16 Sys_SleepErrTimsoutx64ms = 0;
uint32 Sys_RestTimex64ms;


uint32 Sys_WkTick = 0;
uint32 NwxtSys_Tick = 0;
uint8  Sys_WkupType;
uint8  Sys_ResetDevice;
uint8  Sys_frstPoweron;
uint8  Sys_SpiOnFlag;

#define RELOAD_COUNT (32768*60-1)     //    60    

extern const uint8 Aes_KeyTable[16];

/****************************************************************************************************
**Function:
    void Sys_EnterWaitWakeUp(void)
**Author: lxw
**Description:
**Input:
**Output:
****************************************************************************************************/
void Sys_EnterWaitWakeUp(void)
{ 

}
/****************************************************************************************************
**Function:
    void Sys_IoEnterSleep(void)
**Author: lxw
**Description:
**Input:
**Output:
****************************************************************************************************/
void Sys_IoEnterSleep(void)
{

}
/****************************************************************************************************
**Function:
    void Sys_IoExitSleep(void)
**Author: lxw
**Description:
**Input:
**Output:
****************************************************************************************************/
void Sys_IoExitSleep(void)
{



}
/****************************************************************************************************
**Function:
    void Sys_wdt_init(void)  
**Author: lxw
**Description:
**Input:
**Output:
****************************************************************************************************/
void Sys_wdt_init(void)  
{  
    NRF_WDT->TASKS_START = 0;  
    NRF_WDT->CRV = RELOAD_COUNT;  
    NRF_WDT->CONFIG =  (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos |  \
    WDT_CONFIG_SLEEP_Pause << WDT_CONFIG_SLEEP_Pos);  
    NRF_WDT->RREN = (WDT_RREN_RR0_Enabled << WDT_RREN_RR0_Pos);  
}  
/****************************************************************************************************
**Function:
    void Sys_wdt_start(void)  
**Author: lxw
**Description:
**Input:
**Output:
****************************************************************************************************/ 
void Sys_wdt_start(void)  
{  
    NRF_WDT->TASKS_START = 1;  
}  
/****************************************************************************************************
**Function:
  void Sys_wdt_feed(void)  
**Author: lxw
**Description:
**Input:
**Output:
****************************************************************************************************/
void Sys_wdt_feed(void)  
{  
    if(NRF_WDT->RUNSTATUS & WDT_RUNSTATUS_RUNSTATUS_Msk)  
    {
        NRF_WDT->RR[0] = WDT_RR_RR_Reload;  
    }
}  
/****************************************************************************************************
**Function:
    void Sys_wdt_stop(void)  
**Author: lxw
**Description:
**Input:
**Output:
****************************************************************************************************/
void Sys_wdt_stop(void)  
{  
    NRF_WDT->TASKS_START = 0;  
}  
/****************************************************************************************************
**Function:
    void Sys_IoExitSleep(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
Std_ReturnType Sys_DetWakeUp(void)
{
/*    static Adc_Stm8OutDataType addata;
    Sys_InitClock();
#if (ADC_EN == STD_TRUE)
        AdcStm8Drive.open(NULL);
#endif
    for(times= 0; times< 3;times++)
    {
        AdcStm8Drive.ioctl(ADC_STM8_CMD_SCAN,NULL);
        Sys_GenFunDelayUs(200);
    }
    addata.Channel = ADC_STM8_CHANNE1; 
    if(E_OK == AdcStm8Drive.read(&addata))
    {
        addata.Data = (addata.Data<<2);
        addata.Data += 300;
        if(addata.Data > 9300)
        {
            return E_OK;
        }
        else 
        {
            return E_NOT_OK;
        }
    }
    else 
    {
        return E_NOT_OK;
    }*/
    return E_OK;
}

/****************************************************************************************************
**Function:
    void Ekey_Parainit(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Sys_ParainitFirst(void)
{
    memset(&Sys_PataCfg,0,sizeof(Sys_PataCfg));
    
    Sys_PataCfg.Mac = BUILD_UINT32(m_mac_add[5], m_mac_add[4], m_mac_add[3], m_mac_add[2]);
    Sys_PataCfg.Pswd = 0x12345678;
    Sys_PataCfg.SysId = CUSTOM_ID;
    Sys_PataCfg.CustomId = 0x6868;
    Sys_PataCfg.OpenLockTime = 6000;
    Sys_PataCfg.HeartTime = 30;
    Sys_PataCfg.CfgFlag = (SYSBIGFLG);
    Sys_PataCfg.BuildId = 0xff;
    Sys_PataCfg.PropertyId = 0;
    Sys_PataCfg.HotelPswd = 0;
    Sys_PataCfg.bleCfg.advInt = 1000;
    Sys_PataCfg.bleCfg.connectOff = 5000;
    Sys_PataCfg.bleDSTFlag = false;
    Sys_PataCfg.lockDSTFlag = false;
    Sys_PataCfg.resetButtonEnable = false;

    Sys_PataCfg.bleCfg.txPower = 4;
    
    Sys_PataCfg.usedpage = 0x03;
    /*Sys_PataCfg.EncpyPsd[0]= 0x12;
    Sys_PataCfg.EncpyPsd[1]= 0x34;
    Sys_PataCfg.EncpyPsd[2]= 0x56;
    Sys_PataCfg.EncpyPsd[3]= 0x78;*/

    Sys_PataCfg.StartTime = 0;
    Sys_PataCfg.touchSensitive = 4;
//    Sys_PataCfg.SyncIndex = 0;
    Sys_PataCfg.Headadd = 0;
    Sys_PataCfg.HeadMac = 0;
    Sys_PataCfg.rfPara.rfreq= 2;
    Sys_PataCfg.rfPara.Power = 20;
    Sys_PataCfg.rfPara.Speed=1;
    Sys_PataCfg.UploadRecord=1;

    Sys_PataCfg.SyncIndex = 1;

   Sys_PataCfg.configMode = 0;

    memcpy(&Sys_PataCfg.Aeskey, Aes_KeyTable, 16);
    Access_EraseAllKey(KEY_TYPE_ALL);
    Access_EraseRecordData();
    memset(&LockKeyStatistics,0,sizeof(LockKeyStatistics));
    Sys_PataCfg.FactoryTimeout = 0;
    Sys_PataCfg.State =WORK_NORMAL;
	Sys_PataCfg.settimeBak = 1632855680;
    //Access_DefaultPaswd();
}


/****************************************************************************************************
**Function:
    void Ekey_Parainit(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Sys_Parainit(void)
{
    Sys_PataCfg.Pswd = 0x12345678;
    Sys_PataCfg.OpenLockTime = 5000;
    Sys_PataCfg.HeartTime = 30;
    // Enable AUTOLOCK feature by default
    //
    Sys_PataCfg.CfgFlag = (SYSBIGFLG | AUTOLOCK);
    Sys_PataCfg.BuildId = 0xff;
    Sys_PataCfg.PropertyId = 0;
    Sys_PataCfg.HotelPswd = 0;
    Sys_PataCfg.bleCfg.advInt = 1000;
    Sys_PataCfg.bleCfg.connectOff = 5000;
    Sys_PataCfg.bleDSTFlag = false;
    Sys_PataCfg.lockDSTFlag = false;
    Sys_PataCfg.resetButtonEnable = false;

    Sys_PataCfg.bleCfg.txPower = 4;
    Sys_PataCfg.FactoryTimeout = 0;
    Sys_PataCfg.usedpage = 0x03;
    Sys_PataCfg.StartTime = 0;
    Sys_PataCfg.touchSensitive = 4;
    Sys_PataCfg.Headadd = 0;
    Sys_PataCfg.HeadMac = 0;
    Sys_PataCfg.rfPara.rfreq= 2;
    Sys_PataCfg.rfPara.Power = 20;
    Sys_PataCfg.rfPara.Speed=1;
    Sys_PataCfg.UploadRecord=1;
    Sys_PataCfg.configMode = 0;
    memcpy(&Sys_PataCfg.Aeskey, Aes_KeyTable, SIZE_AES_KEY);
    Access_EraseAllKey(KEY_TYPE_ALL);   
    Access_EraseRecordData();         
    memset(&LockKeyStatistics,0,sizeof(LockKeyStatistics));
    Sys_PataCfg.State =WORK_NORMAL;
	Sys_PataCfg.settimeBak = 1632855680;

}

/****************************************************************************************************
**Function:
    void Ekey_StorePara(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Sys_StorePara(void)
{
#if ((defined USE_FLASH_NRF) && (USE_FLASH_NRF == STD_TRUE))
    NrfFlash_PstorageType Message;

    if(NrfFlash_Store(NRF_FLASH_SYS_PRAR) == E_OK)
    {
        Message.Sector = NRF_FLASH_SYS_PRARBK;
    }
    else
    {
        Message.Sector = NRF_FLASH_SYS_PRAR;
    }
    Message.pData= NULL;
    Message.Add = 0;
    Message.Lenth = 0;
    NrfFlashDrive.ioctl(NRFFLASH_STORE, &Message);
#else
    FlashDrive.open(NULL); 

    Flash_ComType eepdata;

    Sys_PataCfg.KeyWord = 0x12345678;
    eepdata.pData = (uint8_t *)&Sys_PataCfg;
    eepdata.Lenth = sizeof(Sys_PataCfg);
    eepdata.Add = ACCESS_SYS_PARA_OFFSET;
    FlashDrive.ioctl(FLASH_ERASE_SECT,&eepdata.Add);
    FlashDrive.write(&eepdata);
#endif
}

/****************************************************************************************************
**Function:
    void Sys_LoadPara(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Sys_LoadPara(void)
{
    Flash_ComType eepdata;
    uint8 i;
    
    FlashDrive.open(NULL); 
    eepdata.pData = (uint8_t *)&Sys_PataCfg;
    eepdata.Lenth = sizeof(Sys_PataCfg);
    eepdata.Add = ACCESS_SYS_PARA_OFFSET;
    
    FlashDrive.read(&eepdata);

    for(i=0; i<3; i++)
    {
        FlashDrive.read(&eepdata);
        if(Sys_PataCfg.KeyWord!= 0x12345678)
        {
            Sys_GenFunDelayMs(500);
            continue;
        }
        else
        {
            break;
        }
    }
    if(i>=3)
    {
        Sys_ParainitFirst();
        Sys_StorePara();
        Sys_McuRestFlag=STD_TRUE;
    }
    
    Sys_PataCfg.SysId = CUSTOM_ID;
    Sys_PataCfg.CfgFlag |= SYSBIGFLG;
    Sys_PataCfg.rfPara.Power = 20;
    Sys_PataCfg.rfPara.Speed = 1;
}

uint32 Rtcpowercnt;
/****************************************************************************************************
**Function:
    void Sys_IoStartUp(void)
**Author: lxw
**Description:
**Input:
**Output:
****************************************************************************************************/
void Sys_IoStartUp(void)
{
    nrf_gpio_cfg_output(RTC_POWER);
    nrf_gpio_cfg_output(TSM_POWER);
    nrf_gpio_cfg_output(Infrared_POWER);
    nrf_gpio_cfg_output(A485_POWER);
    nrf_gpio_cfg_output(WG_POWER);
    
    
    Infrared_PowerOn();
    RTC_PowerOn();
    Rtcpowercnt = 0;
    TSM_PowerOn();
    A485_PowerOn();
    WG_PowerOn();
}
/****************************************************************************************************
**Function:
    void Sys_GetMcuID(void)
**Author: lxw
**Description:
**Input:
**Output:
****************************************************************************************************/
void Sys_InitClock(void)
{  

}

/****************************************************************************************************
**Function:
void Sys_RestProc(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Sys_RestProc(void)
{
    if ( ++Rtcpowercnt >= 150000)   //  2Сʱ����
      {
        RTC_PowerOff();
      }
    if(Sys_StoreTimsoutx64ms)
    {
        Sys_StoreTimsoutx64ms--;
        if(Sys_StoreTimsoutx64ms == 0)
        {
            Sys_StorePara();
        }
    }
    if(Sys_RstTimsoutx64ms)
    {
        Sys_RstTimsoutx64ms--;
        if(Sys_RstTimsoutx64ms == 0)
        {
            NVIC_SystemReset();
        }
    }
    if(Sys_StoreFlag != STD_FALSE)
    {
        Sys_StoreFlag=STD_FALSE;
        Sys_StoreTimsoutx64ms = 1200/64;
    }
    if(Sys_McuRestFlag != STD_FALSE)
    {
        Sys_McuRestFlag=STD_FALSE;
        Sys_RstTimsoutx64ms = 4000/64;
    }
}

/****************************************************************************************************
**Function:
    void Sys_StartUpCheck(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
#define SYS_AUTO_IDENCRY STD_TRUE

uint16 longAccRcord;
uint8  pcbtestflag;
/****************************************************************************************************
**Function:
void Sys_Init(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Sys_Init(void)
{
    RecordAddr_Readflash();
    Sys_IoStartUp();
    Wiegand_Init();
    LightDrive.open(NULL);

   if (0 == IntestGet()) 
   {
     pcbtestflag = 1;
	 uart_init();
   }
	
#ifdef ENABLE_CONSOLE_LOG
    uart_init();
#endif // ENABLE_CONSOLE_LOG
    Sys_GenFunDelayMs(1000);
    Sys_SpiOpen();
    BeepDrive.open(NULL);
    FlashDrive.open(NULL); 
	  NZ3801_Open();
    // Enable AUTOLOCK feature by default
    //
    if(Sys_CheckDisableAutoLock())
    {
        Sys_PataCfg.CfgFlag |= AUTOLOCK;
        // Set Auto-lock time as 5 second
        //
        Sys_PataCfg.OpenLockTime = 5000;
    }
    
    //HC595_LedWriteData(0x0200); 
    //Sys_ParainitFirst();
    //Sys_StorePara();
    
    Sys_LoadPara();


    KeyDrive.open(NULL);
    Access_Init();
    ProtoAnaly_Init();
    Sys_CheckUpFlag = STD_TRUE;
#if (defined WATCHDOG_EN) && (WATCHDOG_EN == STD_TRUE)
    Sys_wdt_init();
#endif
    Event_Init();
    Sys_ResetDevice = 0;
    Sys_State = SYS_NORMAL;
    RtcDrive.open(NULL);
    Sys_RestTimex64ms = 24*60*60*1000/64;
    
    Access_BatteryData= 100;       //

    longAccRcord = sizeof(LockKeyServeKeyList);

    // To disable Voice mode for already installed panels, this tweak is added
    // Currently panel supports only 2 modes
    //   1. Voice mode => bit 0
    //   2. Passage mode => bit 1
    // If any bit other than these 2 bits is set, then it Config mode has a garbage value that
    // needs to be cleared.
    //
    if(Sys_PataCfg.configMode > 0x03) Sys_PataCfg.configMode = 0;
    //Access_BeepStart(BEEP_VERY_FAST,5);
    //HC595_LedWriteData( 0x1000|0x0040|0x0002|0x8000 );   
}

#if (defined DEBUG_ENABLE) && (DEBUG_ENABLE == STD_TRUE)
uint8 Sys_deugFlag = 0;
void Sys_Debug(uint8 mode)
{
    if(Sys_deugFlag == 0)
    {
        return;
    }
    if(mode == STD_OFF)
    {
        DI();
    }
    else
    {
        EI();
    }
}
#endif
extern const nrf_drv_spi_t spi;
/****************************************************************************************************
**Function:
RC522_SetCsLow();
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Sys_SpiOpen(void)
{
    if(Sys_SpiOnFlag == STD_TRUE)
    {
        return;
    }
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG(SPI_INSTANCE);
    spi_config.miso_pin = RC522_MISO_PIN;
    spi_config.mosi_pin = RC522_MOSI_PIN;
    spi_config.sck_pin = RC522_MSCK_PIN;
    nrf_drv_spi_init(&spi, &spi_config, NULL);
    RC522_SetCsHigh();
    Sys_SpiOnFlag = STD_TRUE;
}
/****************************************************************************************************
**Function:
void Sys_SpiClose(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Sys_SpiClose(void)
{
    if(Sys_SpiOnFlag == STD_FALSE)
    {
        return;
    }
    nrf_drv_spi_uninit(&spi);
    nrf_gpio_cfg_output(RC522_MSCK_PIN);
//    nrf_gpio_cfg_output(RC522_MOSI_PIN);
    nrf_gpio_cfg_output(RC522_MISO_PIN);    
    RC522_ClkLow();
    RC522_MosiLow();
//    RC522_MisoLow();
    RC522_SetCsLow();
    Sys_SpiOnFlag = STD_FALSE;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

