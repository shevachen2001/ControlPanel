/*****************************************************************************
|  File Name: Data.c
|
|  Description: 
|
|-----------------------------------------------------------------------------
|               C O P Y R I G H T
|-----------------------------------------------------------------------------
| Copyright (c) 2002-2012 by DaHao TECHNOLOGY Co., Ltd. All rights reserved.
|
|               This software is copyright protected and proprietary 
|               to DaHao TECHNOLOGY Co., Ltd. DaHao TECHNOLOGY Co., Ltd 
|               grants to you only those rights as set out in the 
|               license conditions. All other rights remain with 
|               DaHao TECHNOLOGY Co., Ltd.
|-----------------------------------------------------------------------------
|               A U T H O R   I D E N T I T Y
|-----------------------------------------------------------------------------
| Initials     Name                      Company
| --------     ---------------------     -------------------------------------
| LXW          LXW               DaHao TECHNOLOGY Co., Ltd
|-----------------------------------------------------------------------------
|               R E V I S I O N   H I S T O R Y
|-----------------------------------------------------------------------------
| Date        Ver     Author  Description
| ---------   ----    ------  ------------------------------------------------
| 2013-12 1.00    Rory     Initial version 
|****************************************************************************/

/**************************************************************************************************/ 
/* Include file                                                                                   */                                                 
/**************************************************************************************************/
#include "SYSTEM\sys.h"
#include "KEY\key.h"
#include "SYSTEM\Sys_TimeBase.h"
#include "Protocol\factInfo.h"
#include "Access\Access.h"
#include "Pwm\Pwm.h"
#include "Card\Rc522.h"
#include "protocol\Proto_Analy.h"
#include "RTC\Rtc.h"
#include "Flash\Flash.h"
#include "KEY\Touch.h"
#include "Access\LockKey.h"
#include "Commu\Lock_Commu.h"
#include "Light\light.h"
#include "adc\adc_Nrf.h"
#include "ble_dahao.h"
#include "Encpt\Encpt.h"
#include "CustomCode\CustomCode.h"
#include "FACTORY\Factory.h"
#include "Protocol\Proto_CommPack.h"
#include "HC595\HC595.h"
#include "Card\Nz3801_AB.h"

#define IS_INITIALIZED !(Sys_PataCfg.Pswd == 0x12345678)

extern void LogsCounter(ReadIndexDataType *readIndexPtr);
#ifdef ENABLE_CONSOLE_LOG
extern void printRTCTime(uint32_t rawTime);
#endif // ENABLE_CONSOLE_LOG

extern uint8  Cardetflag;
extern uint8  Lora_Timeout;
extern uint8  ProtoAnaly_LoraBrgSize;
extern uint8 SysSleepFlag;
extern Rtc_Type ProtoAnaly_Rtcinfo;
extern uint8 BattdetTimer;
extern uint8_t pKey[];
extern uint8_t debugPasscodeFail;
extern LockKey_ServeKeyType keyRecord;
extern uint16 keyIndex;
extern uint8 pageId;
extern uint8_t CommandId;
extern unsigned char SN[4];
extern uint8 FobType;
extern CPUFob_DataType CPUFobKey_List;
extern uint8 TouchKeyIndex;
extern _Bool checkKeyForExpiredKeys;
uint16 checkKeyForExpiredCnt;
extern uint8_t uartBuff[32];  
extern uint8_t uartLenx; 
extern uint8  pcbtestflag;

extern void Tsm_Init(void);
extern void Touch_Init( void );
extern uint16 SPI_FLASH_ReadID(void);
extern Std_ReturnType Access_ComTimeLoop(uint32 startTime, uint32 endTime);
extern void Send_Weigand34(unsigned char *str);
extern void SendUart_data(uint8_t * p_data, uint16_t length);
extern void Carddet_timer_start(uint16 time);
extern _Bool isKeyExpired(uint32 time);
extern _Bool isPassageModeActive(void);
extern void nz3801_Init(void);

extern void Tsmxx_Read_Nyetes(uint8 *poutdat,uint8 subadd,uint8 num);
extern void get_mac_addr(uint8_t *p_mac_addr);


Std_ReturnType ValidateFlashWriteOperation(uint8 flashOffset,
                                           uint16 index,
                                           uint32_t passCode,
                                           uint8_t IdType,
                                           uint32_t endTime,
                                           uint32_t startTime,
                                           uint8_t isCyclic);
Access_ParaType Access_Globle;
uint16 Access_LockTimer64 = 0;
uint16 Access_DefaultTimer64 = 0;
uint16 Access_CardLearnTimer64 = 0;
uint16 Access_ResetDeviceTimer64 = 0;
uint8  Access_AddId_NewFlag= 0;
uint8  Access_FlashProcFlag = 0;
uint32 Access_UserId_Temp = 0;
uint32 Access_ScanInex_Temp = 0;
uint8 Access_AttTIme_Temp[9] = {0};
uint16 Access_OpenRedIndex = 0;
uint16 Access_OpenWriteaIndex = 0;
uint16 Access_Store_Recordx64ms = 0;
uint8 Access_KeyDatTimex64ms=0 ;
uint16 Access_BatteryData;
uint16 Access_LockDeviceTimer64ms=0;  /* 三次错误锁住*/
uint16 Access_LockDeviceTimer64msfob=0;  /* 三次错误锁住*/

uint8 Access_LockStatus=0; /* 门锁状态*/
uint8     RelayOpenFlag;
uint16_t  RelayOpenDelayTime = 0;;
uint8     AgeingFlag;   // Ageing
uint32    Ageingcnt;
uint8     HardDbgKeyFlag;
AccRcordParaType AccRcord;
AccRcordParaindex PAccRcord;

uint32_t InfraredTimer64ms = 0;
uint16 Access_CardOpenTimer64 = 0;
uint8 Access_Record_Overflow = 0;

uint8_t fob_sn[4]={0x00};
void Access_CardProcess(uint8 idtpye, uint8* pUid,Access_CardDatType *CardDat);
Std_ReturnType Access_ComTimeLoop(uint32 startTime, uint32 endTime);

/****************************************************************************************************
**Function:
    void Access_Init(void)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Access_Init(void)
{
    Access_LockTimer64 = 0;
    Access_DefaultTimer64 = 0;
    Access_CardLearnTimer64 = 0;
    Access_ResetDeviceTimer64 = 0;

    if(Sys_PataCfg.State  == WORK_LEARN_CARD)
    {
        Sys_PataCfg.State = WORK_NORMAL;
    }
    Access_Globle.DefaultTimes = 0;
    LockCommu_Init();
    BlackList_LoadPara();
	RecordIndex_LoadPara();
	
    Access_FlashArrang();
#if (defined TOUCH_ENABLE) && (TOUCH_ENABLE == STD_TRUE)    
    TouchRstTimes = 1;
#endif
    Factory_State = COMD_FATORY_IDLE;
}


uint8 TestFob_SN[4] = {0x49,0x4f,0x7d,0x62};
uint8 FobOkFlag;
/****************************************************************************************************
**Function:
    void Access_DetCardProc(void)
**Author: rory
**Description:  64ms  once
**Input: 
**Output: 
****************************************************************************************************/
void Access_DetCardProc(void)
{
    uint8 CardId[8] = {0};
    uint8 Carddat[16];
    Card_ParmType pcard;
    Access_CardDatType Access_CardDat;
	uint8 tbuf[4]={0};
    uint8 ret;

    pcard.pid = CardId;
    pcard.pdat = Carddat;
    memset(Carddat,0,sizeof(Carddat));
    nz3801_Init();
	ret = CertificationCPUFob();

	if ( FactoryAppTest.testFlag.FT_FobFlag )
	{
	  	memcpy(fob_sn,SN,sizeof(fob_sn));
	}

	if ( pcbtestflag&& (0 == memcmp(TestFob_SN ,SN ,4 )))
	{
	  FobOkFlag = 1;
	  return;
	}
	
	if( CARD_TYPE_READ_OK == ret )
    { 
		pcard.IdType = FobType;
        ProtoAnaly_UpdateTime();
		
        {
		    memset((uint8*)&Access_CardDat, 0xFF, sizeof(Access_CardDat));
            goto CARDIDPROC;
        }
    }

	else if( ret == CARD_CPU_FINDKEY_ERR )
	{
		Access_Globle.ErrorTimesfob++;  
		Access_OpenError();
		if(Access_Globle.ErrorTimesfob >= 5)    
		{        
			Access_LockDeviceTimer64msfob = 180000/64; 
			return;
		}
        tbuf[0]= SN[3];
        tbuf[1]= SN[2];
        tbuf[2]= SN[1];
        tbuf[3]= SN[0];
	    Access_WriteRecordFlash(tbuf, ProtoAnaly_RtcLocalTime, FobType, false);	
	}
	else if ( ret == CARD_CPU_LOCK_ERR )
	{
		Access_OpenError();
        tbuf[0]= SN[3];
        tbuf[1]= SN[2];
        tbuf[2]= SN[1];
        tbuf[3]= SN[0];
	    Access_WriteRecordFlash(tbuf, ProtoAnaly_RtcLocalTime, CPUFOB_INVALID, false);
	}
	 
    return;
    CARDIDPROC:

	if((pcard.IdType == KEY_TYPE_CARD) || (pcard.IdType == KEY_TYPE_CPUCARD))
    {
        tbuf[0]= SN[3];
        tbuf[1]= SN[2];
        tbuf[2]= SN[1];
        tbuf[3]= SN[0];
    }
    Access_CardProcess(pcard.IdType,tbuf,&Access_CardDat);
    Access_KeyDatTimex64ms = 100/64;
    Carddet_timer_start(500);
#if (defined DEBUG_ENABLE) && (DEBUG_ENABLE == STD_TRUE)
    Sys_Debug(STD_ON);
#endif
}
/****************************************************************************************************
**Function:
    void Access_Proc(void)
**Author: rory
**Description:  64ms
**Input: 
**Output: 
****************************************************************************************************/
void Access_TimerProc(void)
{    
    if(Access_ResetDeviceTimer64 != 0)
    {
        Access_ResetDeviceTimer64--;
        if(Access_ResetDeviceTimer64 == 0)
        {
            NVIC_SystemReset();
        }
    }
    if(Access_LockDeviceTimer64ms != 0)
    {
        Access_LockDeviceTimer64ms--;
        if(Access_LockDeviceTimer64ms <= 1)
        {
            Access_Globle.ErrorTimes =0;
            Access_LockDeviceTimer64ms = 0;
        }
    }

    if(Access_LockDeviceTimer64msfob!= 0)
    {
        Access_LockDeviceTimer64msfob--;
        if(Access_LockDeviceTimer64msfob<= 1)
        {
            Access_Globle.ErrorTimesfob=0;
            Access_LockDeviceTimer64msfob= 0;
        }
    }
	
    
    if(Access_CardLearnTimer64 != 0)
    {
        --Access_CardLearnTimer64;
        if(Access_CardLearnTimer64 == 0)
        {
            Sys_PataCfg.State = WORK_NORMAL;
            Access_BeepStart(BEEP_NORMAL,1);
        }
    }

	if(Access_CardOpenTimer64 != 0)
	{
	  --Access_CardOpenTimer64;
	  if(Access_CardOpenTimer64 == 0)
	  {
	      Access_Unlock(); 
	  }
	}  

	if(checkKeyForExpiredCnt != 0)
	{
	  --checkKeyForExpiredCnt;
	  if(checkKeyForExpiredCnt == 0)
	  {
		checkKeyForExpiredKeys = true;
	  }
	} 

    BeepDrive.ioctl(BEEP_CMD_SCAN, NULL);
    LightDrive.ioctl(LIGHT_CMD_SCAN,NULL);
}
/****************************************************************************************************
**Function: void Vdd_PowerOff(void)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Vdd_PowerOff(void)
{
    if(BattdetTimer == STD_FALSE)
    {
        nrf_gpio_pin_clear(VDD_POWER);
    }
}

void ReLay_Keyscan(void)                         //  debug
{
    static uint8 cnt;
    nrf_gpio_cfg_input(RELAY_SW_PIN,NRF_GPIO_PIN_PULLUP);
    if ( RELAYSWGetData() == 0 && 0 == RelayOpenFlag )    // RELAYSWGetData 
      {
#ifdef ENABLE_CONSOLE_LOG
          printf("Exit: %d: Count: %d\n\r", RELAYSWGetData(), cnt);
#endif // ENABLE_CONSOLE_LOG
        if ( ++cnt >= 2 )
          {
            cnt = 0;
            Access_Unlock();
#ifdef ENABLE_CONSOLE_LOG
            printf("Log: Manual unlock\n\r");
#endif // ENABLE_CONSOLE_LOG
            // Update time
            //
            ProtoAnaly_UpdateTime();
            // Log activity for manual unlock
            //
            Access_WriteRecordFlash(pKey, ProtoAnaly_RtcLocalTime, 28, true);
          }
      }
}

void Infrared_Keyscan(void)
{
    _Bool beepStart = false;

    nrf_gpio_cfg_input(Infrared_PIN, NRF_GPIO_PIN_PULLUP);

    if((1 == InfraredGetData()) && IS_INITIALIZED)
    {
        beepStart = true;
        if(0 == InfraredTimer64ms)
        {
            // Load down counter with 30 sec
            //
            InfraredTimer64ms = 30000 / 64;
        }
    }

    // Start and continue beep sounds untill backplate is open OR
    // timer expires
    //
    if(true == beepStart || InfraredTimer64ms != 0)
    {
        Access_BeepStart(BEEP_OPEN_ERR, 1);
#ifdef ENABLE_CONSOLE_LOG
        printf("K8\n\r");
#endif // ENABLE_CONSOLE_LOG
        Access_LightStart(LIGHT_NG,LIGHT_SLOW, 1, 0);
    }
    // Check if counter is loaded
    //
    if(false == beepStart && InfraredTimer64ms)
    {
        // Decrement counter untill cleared
        //
        InfraredTimer64ms--;
    }
}

uint8 errorstatus;
uint32 RtcLocalTime;
uint32 RtcLocalTimebak;

extern _Bool RtcReadStatus;
static uint32 mon_yday[2][12] =
  {
    {0,31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334},
    {0,31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335},
  };

int isleap(int year)
  {
    return (year) % 4 == 0 && ((year) % 100 != 0 || (year) % 400 == 0);
  }


uint32 mktime( Rtc_Timestamp dt )
  {
	  uint32 result;
	  uint16 i =0;

	  // Seconds in normal years
	  result =( dt.Year - 1970 )* 365 * 24 * 3600 +
	  (mon_yday[isleap(dt.Year)][dt.Month-1] + dt.DayOfMonth - 1) * 24 * 3600 +
	  dt.Hour * 3600 + dt.Minute * 60 + dt.Second;

	  // Add the number of seconds in leap years
	  for(i = 1970; i < dt.Year; i++ )
	 	{
	 	  if (isleap(i))
	 	 	{
	 	 	   result += 24 * 3600;
	 	 	}
	 	}
	  return(result);
  }

Rtc_Type RTC_CheckData;

void UpdateTimestamp(void)
{
	Rtc_Timestamp	Rtc_Tsdata;

	Rtc_Tsdata.Year = RTC_CheckData.Year + 2000;
	Rtc_Tsdata.Month= RTC_CheckData.Month;
	Rtc_Tsdata.DayOfMonth= RTC_CheckData.DayOfMonth;
	Rtc_Tsdata.Hour= RTC_CheckData.Hour;
	Rtc_Tsdata.Minute= RTC_CheckData.Minute;
	Rtc_Tsdata.Second= RTC_CheckData.Second;		
	RtcLocalTime = mktime(Rtc_Tsdata);
}

const char Days[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

void localtime(uint32 time, Rtc_Type *t)
{
	unsigned int Pass4year;
	int hours_per_year;
	uint16 tyear;
	if(time < 0)
	{
	  time = 0;
	}
	t->Second=(int)(time % 60);
	time /= 60;
	t->Minute=(int)(time % 60);
	time /= 60;
	Pass4year=((unsigned int)time / (1461L * 24L));
	//t->Year=(Pass4year << 2) + 1970;
	tyear = (Pass4year << 2) + 1970;
	time %= 1461L * 24L;
	for (;;)
		{
		hours_per_year = 365 * 24;
		if ((tyear & 3) == 0)
		//if ((t->Year & 3) == 0)
			{
				hours_per_year += 24;
			}
		if (time < hours_per_year)
			{
				break;
			}
		//t->Year++;
		tyear++;
		time -= hours_per_year;
		}
	t->Hour=(int)(time % 24);
	time /= 24;
	time++;
	if((tyear & 3) == 0)
	//if((t->Year & 3) == 0)
		{
		if (time > 60)
			{
				time--;
			}
		else
			{
			if (time == 60)
				{
					t->Month = 1;
					t->DayOfMonth = 29;
					return ;
				}
			}
		}
	for (t->Month = 0; Days[t->Month] < time;t->Month++)
		{
			time -= Days[t->Month];
		}
	t->DayOfMonth = (int)(time);
	tyear =  tyear - 2000;
	t->Year = (uint8)tyear;
	return;
  }

uint32 Checkcnt;


void CheckRtc(void)
{
    Rtc_Type Rtc_data;
    static uint8 cnt;

	Checkcnt ++;

	if ( ++cnt >= 5 )
	{
	    cnt = 0;
		if ((RTC_CheckData.Year  < 21)   || (RTC_CheckData.Year>=64) ||   
    		(RTC_CheckData.Month  > 12)  || (RTC_CheckData.Month==0) ||   
    		(RTC_CheckData.DayOfMonth   > 31)   || (RTC_CheckData.DayOfMonth==0)  ||   
    		(RTC_CheckData.Hour   >=24)   ||     
    		(RTC_CheckData.Minute >=60)   ||   
    		(RTC_CheckData.Second >=60)      
		   )
		{
			localtime(Sys_PataCfg.settimeBak + Checkcnt, &Rtc_data );
			Rtc_data.Month = Rtc_data.Month + 1;
			Rtc_Ioctl(RTC_CLOCKSET,&Rtc_data);		
			return;
		}
	}
}



void LEDerrordisplay( void )
{
   switch ( errorstatus )
   {
   	 case 1:
	    HC595_LedWriteData(0x0040);    // 7
	  break; 
   	 case 2:
	    HC595_LedWriteData(0x0100);    // 3
	  break; 	 
   	 case 3:
        HC595_LedWriteData(0x0020);    // 4
	  break; 
   	 case 4:
		HC595_LedWriteData(0x0004);    // 5
	  break; 	 
   	 case 5:
	    HC595_LedWriteData(0x1000);    // 6
        default:
	  break; 	
   }  
}


void Ageing_Timepro(void) 
{
    static uint8 cnt ,cnt2s,cnt10s;
	static uint32 Delte,RtcLocalTime2sbak,RtcLocalTime10sbak;
	uint8 keydat;
    if ( 0 == AgeingFlag )  return;
	Ageingcnt ++;
    if ( 43200 < Ageingcnt  )
    {
      AgeingFlag = 0;
	  Ageingcnt = 0;
    }
	
    if ( 0!= errorstatus ) 
    {
      LEDerrordisplay();
	  return;
    }
	cnt++;
	
	if ( cnt == 2 )
	{
	    errorstatus = 0;
	 	HC595_LedWriteData(0xffff);
	}
	else if ( cnt == 4 )
	{
		cnt = 0;
		HC595_LedWriteData(0);
		if ( 0xEF15 != SPI_FLASH_ReadID())
		{
		    errorstatus = 1;
			HC595_LedWriteData(0x0040);    // 7
			return;
		}
		
        Tsmxx_Read_Nyetes(&keydat,0x08,1);
		if ( 0x82 != keydat )
		{
			errorstatus = 2;
			HC595_LedWriteData(0x0100);    // 3
			return;
		}
	}

	
	if( ++cnt2s >= 2 )
	{     
	      cnt2s = 0;
		  UpdateTimestamp();
		  
		  if ( 0 == RtcLocalTime2sbak)
		  {
		  	RtcLocalTime2sbak = RtcLocalTime;
		  }
		  else
		  {
		    if ( RtcLocalTime2sbak == RtcLocalTime )
		    {
		      errorstatus = 3;
			  HC595_LedWriteData(0x0020);    // 4
			  return;
		    }
			else
			{
		  		RtcLocalTime2sbak = ProtoAnaly_RtcLocalTime;
			}
		  }

		  if ( ++ cnt10s >= 5 )
		  {
		    cnt10s = 0;
			if ( 3600 > Ageingcnt || 39600 < Ageingcnt )  
			{
		      Access_Unlock();
			} 
            if ( 0 == RtcLocalTime10sbak) 
            {
              RtcLocalTime10sbak = RtcLocalTime;
            }  
			else
			{
			  Delte =( RtcLocalTime - RtcLocalTime10sbak );
			  if ( 8 > Delte ) 
			  {
		        errorstatus = 4;
			    HC595_LedWriteData(0x0004);    // 5
			    return;
			  }
			  else if ( 12 < Delte ) 
			  {
			 	errorstatus = 5;
			    HC595_LedWriteData(0x1000);    // 6
			    return;
			  }
			  else
			  {
			    RtcLocalTime10sbak = RtcLocalTime;
			  }
			}
		  }
	}
}

#define C32HEX2BYTESLSB1(pBuf, data)          \
        *(pBuf + 0) = (uint8)((data) >> 0);  \
        *(pBuf + 1) = (uint8)((data) >> 8);  \
        *(pBuf + 2) = (uint8)((data) >> 16); \
        *(pBuf + 3) = (uint8)((data) >> 24)


#define C32HEX2BYTESMSB1(pBuf, data)          \
        *(pBuf + 0) = (uint8)((data) >> 24); \
        *(pBuf + 1) = (uint8)((data) >> 16); \
        *(pBuf + 2) = (uint8)((data) >> 8);  \
        *(pBuf + 3) = (uint8)((data) >> 0)

void Rtc_SetTimeIn(void)
{
    Rtc_Type Rtc_data;

    Rtc_data.Year = 21;
    Rtc_data.Month= 1;
    Rtc_data.DayOfMonth= 1;
    Rtc_data.Hour= 1;
    Rtc_data.Minute=1;
    Rtc_data.Second= 1;
    Rtc_Ioctl(RTC_CLOCKSET,&Rtc_data);

	RtcLocalTimebak = 0;
}

extern void uart_init(void);


void PcbaTest_pro(void) 
{
	uint8_t m_mac_add[6];
	uint8_t rtc_buf[4];
	uint8_t temp;

	if ( 0 == pcbtestflag )  return;

	if ( strstr((const char*)uartBuff,(const char*)"Unlock")) 
	{
		Rtc_SetTimeIn();
        Access_Unlock();
	}
	
	if ( strstr((const char*)uartBuff,(const char*)"mac")) 
	{
	    get_mac_addr(m_mac_add);
        temp = 0xA5;
	    SendUart_data(&temp,1);		
		SendUart_data(m_mac_add,6);
	}

	if ( strstr((const char*)uartBuff,(const char*)"Fang_Test")) 
	{
	    if ( 0 == InfraredGetData() )
	    {
		   SendUart_data("Fang_OK",7);
	    }	
		else
		{
		   SendUart_data("Fang_NG",7);
		}
	}	
	
	if ( strstr((const char*)uartBuff,(const char*)"RTC_Test")) 
	{
       UpdateTimestamp();
       temp = 0x5A;
	   SendUart_data(&temp,1);
	   C32HEX2BYTESLSB1(rtc_buf, RtcLocalTime );
	   SendUart_data(rtc_buf,4);
	}	

	if ( strstr((const char*)uartBuff,(const char*)"Get_Time")) 
	{
       UpdateTimestamp();
       temp = 0xA5;
	   SendUart_data(&temp,1);
	   C32HEX2BYTESLSB1(rtc_buf, RtcLocalTime );
	   SendUart_data(rtc_buf,4);
	}

	if ( strstr((const char*)uartBuff,(const char*)"Fob_Test")) 
	{
	    if ( 1 == FobOkFlag )
	    {
		   SendUart_data("Fob_OK",6);
	    }	
		else
		{
		   SendUart_data("Fob_NG",6);
		}	
	}	
	
	memset(uartBuff,0,sizeof(uartBuff ));
	uartLenx = 0;	
}



uint16 RelaytestCnt;
void ReLay_Timepro(void) 
{
    ReLay_Keyscan();
#ifdef ENABLE_INFRARED_SENSOR
    Infrared_Keyscan();
#endif // ENABLE_INFRARED_SENSOR

    if(RelayOpenFlag)
    {
        // Check if passage mode is ON
        //
        if(true == isPassageModeActive())
        {
#ifdef ENABLE_CONSOLE_LOG
            printf("Autolock not done: Passage Mode is ON\n\r");
#endif // ENABLE_CONSOLE_LOG
            // Do not perform Autolock if passage mode is ON
            //
            return;
        }

        if(--RelayOpenDelayTime <= 0)
        {
            // Clear relay open flag
            ///
            RelayOpenFlag = 0;
            // Clear access card open flag
            //
            cardopenFlag = 0;
            // Drive relay pin to activate lock operation
            //
            ReLay_PIN_Low();
            // Start advertise to update lock status in the broadcast data
            //
            if(Sys_EnableBlueTooth())
            {
                ble_dahao_start_advert(1);
            }
#ifdef ENABLE_CONSOLE_LOG
            printf("Write Log: %d\n\r", KEY_TYPE_PASSWORD);
#endif // ENABLE_CONSOLE_LOG
            // Update time
            //
            ProtoAnaly_UpdateTime();
            // Log activity for auto-lock
            //
            Access_WriteRecordFlash(pKey, ProtoAnaly_RtcLocalTime, 12, true);
#ifdef ENABLE_CONSOLE_LOG
            printf("Auto lock done\n\r");
#endif // ENABLE_CONSOLE_LOG
        }
    }
}


/****************************************************************************************************
**Function:
    void Access_Unlock(void)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Access_Unlock(void)     //   open zhe door 
{
#ifdef ENABLE_CONSOLE_LOG
    printf("Unlocking...\n\r");
#endif // ENABLE_CONSOLE_LOG

    Vdd_PowerOn();
    Access_Globle.ErrorTimes =0;
	Access_Globle.ErrorTimesfob=0;

    Access_LockDeviceTimer64ms = 0;
	Access_LockDeviceTimer64msfob= 0;

    TouchKeyIndex = 0;

    if ( !AgeingFlag )
    {
      Access_LightStart(LIGHT_OK,LIGHT_SLOW,1,0);
    } 
	Access_BeepStart(BEEP_OPEN,1);
    ReLay_PIN_output();
    ReLay_PIN_High();

    RelayOpenFlag = 1;
    if(Sys_EnableBlueTooth())
    {
        ble_dahao_start_advert(1);
    }

    //  Check if autolock time is set in the config
    //
    if(Sys_PataCfg.OpenLockTime > 4999)
    {
        // Load relay open time with AUTOLOCK time configured
        //
        RelayOpenDelayTime = Sys_PataCfg.OpenLockTime / 64;
#ifdef ENABLE_CONSOLE_LOG
        printf("Autlock enabled...\n\r");
#endif // ENABLE_CONSOLE_LOG
    }
    else
    {
        // Load relay open time with default time of 5000ms
        //
        RelayOpenDelayTime = 5000 / 64;
#ifdef ENABLE_CONSOLE_LOG
        printf("Autlock disabled...\n\r");
#endif // ENABLE_CONSOLE_LOG
    }
#ifdef ENABLE_CONSOLE_LOG
    printf("Autolock in progress...%u msec\n\r", RelayOpenDelayTime * 64);
#endif // ENABLE_CONSOLE_LOG

#ifdef ENABLE_CONSOLE_LOG
    printf("Unlocked\n\r");
#endif // ENABLE_CONSOLE_LOG
}
/****************************************************************************************************
**Function:
    void Access_Lock(void)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Access_Lock(void)
{
    Access_Globle.ErrorTimes = 0;
    Access_LockDeviceTimer64ms = 0;
    RelayOpenDelayTime = 0;
    ReLay_PIN_Low();
    RelayOpenFlag = 0;
    cardopenFlag = 0;

    if(Sys_EnableBlueTooth())
    {
        ble_dahao_start_advert(1);
    }
#ifdef ENABLE_CONSOLE_LOG
    printf("Locked...\n\r");
#endif  // ENABLE_CONSOLE_LOG
}
/****************************************************************************************************
**Function:
    void Access_OpenError(void)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Access_OpenError(void)
{
    Access_BeepStart(BEEP_VERY_FAST,4);
#ifdef ENABLE_CONSOLE_LOG
    printf("K9\n\r");
#endif // ENABLE_CONSOLE_LOG
    Access_LightStart(LIGHT_NG,LIGHT_SLOW,1,0);
}

//#define ACCESS_FASTCMPMAX    1024
//#define ACCESS_FASTCMPMIN    512
//static uint8 Access_FastCmpBuf[ACCESS_FASTCMPMAX] __attribute__((at(0x20002D90)));

/****************************************************************************************************
**Function:
    void Access_EraseAllKey(void)
**Author: rory
**Description: 
**Input: 
**Output: 
****************************************************************************************************/
void Access_EraseAllKey(uint8 idtpye)
{
    memset(&LockKeyBlackList,0xff,sizeof(LockKeyBlackList));
    BlackList_StorePara();

    for (uint8 j=0;j<SERVE_KEY_PAGEMAX;j++ )
    {
        memset(&LockKeyServeKeyList,0xff,sizeof(LockKeyServeKeyList));
        ServeKeyList_Writeflash(j);
    }
	
    for (uint8 k=0;k<CPUFOB_PAGEMAX;k++ )    
	{
        memset(&CPUFobKey_List,0xff,sizeof(CPUFobKey_List));
        CpuFobList_Writeflash(k);
    }
}

/****************************************************************************************************
**Function:
    void Access_DefaultPaswd(void)
**Author: rory
**Description: 
**Input: 
**Output: 
****************************************************************************************************/

void Access_DefaultPaswd(void)
{
    LockKeyServeKeyList.KeyList[0].Type = KEY_TYPE_PASSWORD;
    LockKeyServeKeyList.KeyList[0].KeyId = 0xff888888;
    LockKeyServeKeyList.KeyList[0].EndTime = BUILD_TIME(30, 12, 30, 23, 59, 59);
    LockKeyServeKeyList.KeyList[0].StartTime = BUILD_TIME(18, 1, 1, 1, 1, 1);    
    LockKeyStatistics.ServKeyCount++;
    //ServeKeyList_StorePara1();
}

/****************************************************************************************************
**Function:
    void Access_LearnCardOk(uint8 newflag,uint8 idtpye,uint16 index,uint8 *time,uint8 *userid,uint8*cardid)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
Std_ReturnType Access_LearnCardOk(uint8 idtpye,uint8*cardid)
{
    uint8 index = 0xff;
    uint8 flashoffset;
    Std_ReturnType status = E_OK;
    uint32 cardId;
    uint8 modify = 0;
    uint32_t endTime = 0;
    uint32_t startTime = 0;

    if(idtpye == KEY_TYPE_IDENTITY_CARD)
    {
        cardId = BUILD_UINT32(cardid[7],cardid[6],cardid[5],cardid[4]);
    }
    else
    {
        cardId = BUILD_UINT32(cardid[3],cardid[2],cardid[1],cardid[0]);
    }

    for(uint8 i=0; i<BLACK_LIST_MAX; i++)
    {
        if((KEY_TYPE_CARD == LockKeyBlackList.BlackList[i].Type) || (KEY_TYPE_IDENTITY_CARD == LockKeyBlackList.BlackList[i].Type))
        {
            if(cardId == LockKeyBlackList.BlackList[i].KeyId)
            {
                modify = 1;
                memset(&LockKeyBlackList.BlackList[i], 0xff, sizeof(LockKey_BlackType));
                if(LockKeyStatistics.BlackListCount > 0)
                {
                    LockKeyStatistics.BlackListCount--;
                }
            }
        }
    }
    if(modify == 1)
    {
        BlackList_StorePara();
    }

    startTime = BUILD_UINT32(Access_AttTIme_Temp[3],Access_AttTIme_Temp[2],Access_AttTIme_Temp[1],Access_AttTIme_Temp[0]);
    endTime = BUILD_UINT32(Access_AttTIme_Temp[7],Access_AttTIme_Temp[6],Access_AttTIme_Temp[5],Access_AttTIme_Temp[4]);

    for (uint8 j=0;j<SERVE_KEY_PAGEMAX;j++ )
    {
        ServeKeyList_Readflash( j );
        for(uint8 i=0; i<SERVE_KEY_MAX; i++)
        {
            if(cardId == LockKeyServeKeyList.KeyList[i].KeyId)
            {//强制修改有效时间
                if(COMD_UPDATE_PASSKEY == CommandId)
                {
#ifdef ENABLE_CONSOLE_LOG
                    printf("Received update fob command\n\r");
#endif // ENABLE_CONSOLE_LOG
                    pageId = flashoffset = j;
                    keyIndex = index = i;
                }
                else if(isKeyExpired(LockKeyServeKeyList.KeyList[i].EndTime))
                {
#ifdef ENABLE_CONSOLE_LOG
                    printf("Found expired fob with same code\n\r");
#endif // ENABLE_CONSOLE_LOG
                    pageId = flashoffset = j;
                    keyIndex = index = i;
                }
                else
                {
                    return 0x0A;
                }
            }

            if(index == 0xff)
            {
                if(LockKeyServeKeyList.KeyList[i].KeyId == 0xffffffff)
                {
                    if(COMD_UPDATE_PASSKEY == CommandId)
                    {
                        return E_KEY_NOT_FOUND;
                    }
                    pageId = flashoffset = j;
                    keyIndex = index = i;
                }
            }
        }
    }
    if(index != 0xff)
    {//增新
        ServeKeyList_Readflash( flashoffset );

        pageId = flashoffset;
        keyIndex = index;
        memcpy(&keyRecord,
               (const LockKey_ServeKeyListType *)&LockKeyServeKeyList.KeyList[index],
               sizeof(keyRecord));

        LockKeyServeKeyList.KeyList[index].Type = idtpye;     // read 
        LockKeyServeKeyList.KeyList[index].KeyId = cardId;
        LockKeyServeKeyList.KeyList[index].StartTime = startTime;
        LockKeyServeKeyList.KeyList[index].EndTime = endTime;
        LockKeyServeKeyList.KeyList[index].Cycle = Access_AttTIme_Temp[8];
        LockKeyStatistics.ServKeyCount++;
        ServeKeyList_Writeflash( flashoffset );

        //ServeKeyList_StorePara1();
        return ValidateFlashWriteOperation(flashoffset, index, cardId, idtpye, endTime, startTime, Access_AttTIme_Temp[8]);
    }
    return status;
}
/****************************************************************************************************
**Function:
    Std_ReturnType Access_ComTime(uint32 startTime, uint32 endTime)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
Std_ReturnType Access_ComTime(uint32 startTime, uint32 endTime)
{
    uint32 LocalTime = ProtoAnaly_RtcLocalTime;//&0xffffffc0;

#ifdef ENABLE_CONSOLE_LOG
    printf("LocalTime: ");
    printRTCTime(LocalTime);
    printf("startTime: ");
    printRTCTime(startTime);
    printf("endTime: ");
    printRTCTime(endTime);
    printf("\n\r");
#endif // ENABLE_CONSOLE_LOG

    if((startTime == 0xffffffff) || (LocalTime < startTime))
    {
        debugPasscodeFail = 21;
#ifdef ENABLE_CONSOLE_LOG
        printf("LocalTime: 0x%lx::startTime: 0x%lx\n\r", LocalTime, startTime);
#endif // ENABLE_CONSOLE_LOG
        return E_END;
    }
    else if(endTime == 0xffffffff)
    {
        return E_OK;
    }
    if(startTime == 0)
    {
        if(endTime >= LocalTime)
        {
            return E_OK;
        }
    }
    else
    {
        if((startTime <= LocalTime)&&(LocalTime <= endTime))
        {
            return E_OK;
        }
        else if(LocalTime > endTime)
        {
            debugPasscodeFail = 22;
            checkKeyForExpiredCnt = 5000/64;
            return E_TIME_OUT;
        }
    }
    return E_NOT_OK;
}

/****************************************************************************************************
**Function:
    Std_ReturnType Access_ComTimeLoop(uint32 startTime, uint32 endTime)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
Std_ReturnType Access_ComTimeLoop(uint32 startTime, uint32 endTime)
{
    uint32 currentTime;
    Std_ReturnType status = E_NOT_OK;

    currentTime = BUILD_LOOP_TIME(ProtoAnaly_Rtcinfo.Hour,
                                  ProtoAnaly_Rtcinfo.Minute,
                                  ProtoAnaly_Rtcinfo.Second);

    // Extract only HHMMSS value from complete date 
    //
    startTime = startTime & 0x001ffffu;
    endTime   = endTime   & 0x001ffffu;
    
    // Check if current time falls between start and end time
    //
    if((startTime <= currentTime) &&
       (endTime   >= currentTime))
    {
        status = E_OK;
    }

    return status;
}

#if 1//(SUPPORT_RECORD_LOC_STORE==STD_TRUE)
/****************************************************************************************************
**Function:
    void Access_WriteRecordFlash(uint8* pKeyId,uint32 time ,uint8 type, uint8 action)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
uint8  Access_GetRecord(uint16 index,uint8 *poutdat)
{
    if ( 0xffffffff == AccRcord.RecordList[index].Id )
    {
      AccRcord.RecordList[index].Id = 0xAAAA5555;
    }
    poutdat[0] = BUILD_TYPE(AccRcord.RecordList[index].TypeResult);

    poutdat[1] = TWOHI_UINT32(AccRcord.RecordList[index].Id);
    poutdat[2] = TWOLO_UINT32(AccRcord.RecordList[index].Id);
    poutdat[3] = ONEHI_UINT32(AccRcord.RecordList[index].Id);
    poutdat[4] = ONELO_UINT32(AccRcord.RecordList[index].Id);
    poutdat[5] = 0xff;
    poutdat[6] = 0xff;
    poutdat[7] = 0xff;
    poutdat[8] = 0xff;
    poutdat[9] = BUILD_ACTION(AccRcord.RecordList[index].TypeResult);
    poutdat[10] = TWOHI_UINT32(AccRcord.RecordList[index].Time);
    poutdat[11] = TWOLO_UINT32(AccRcord.RecordList[index].Time);
    poutdat[12] = ONEHI_UINT32(AccRcord.RecordList[index].Time);
    poutdat[13] = ONELO_UINT32(AccRcord.RecordList[index].Time);
    return 1;
}
#endif

/****************************************************************************************************
**Function:
    void Access_WriteRecordFlash(uint8* pKeyId,uint32 time ,uint8 type, uint8 action)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Access_WriteRecordFlash(uint8* pKeyId,uint32 time ,uint8 type, uint8 action)
{
    uint32 idtemp;

	idtemp = BUILD_UINT32(pKeyId[3], pKeyId[2], pKeyId[1], pKeyId[0]);
	if( 0xffffffff == idtemp ) return;

	
    RecordIndex_LoadPara( );
    RecordList_LoadPara(PAccRcord.Wrecordindex.pageIndex);
    time = ProtoAnaly_RtcLocalTime;

    if(PAccRcord.recordnum >= 0xFFFF)
    {
        PAccRcord.recordnum = 1;
    }
    if(PAccRcord.Wrecordindex.sectorIndex >= ACCESS_RECORD_MAX )
    {
#ifdef ENABLE_CONSOLE_LOG
        printf("Max logs per page\n\r");
#endif
        PAccRcord.Wrecordindex.pageIndex++;
        if(PAccRcord.Wrecordindex.pageIndex >= ACCESS_PAGE_MAX)
        {
            PAccRcord.Wrecordindex.pageIndex = 0;
            Access_Record_Overflow = 1;
        }
        PAccRcord.Wrecordindex.sectorIndex = 0;
    }
    AccRcord.RecordList[PAccRcord.Wrecordindex.sectorIndex].Id = BUILD_UINT32(pKeyId[3], pKeyId[2], pKeyId[1], pKeyId[0]);
    AccRcord.RecordList[PAccRcord.Wrecordindex.sectorIndex].Time = time;
    AccRcord.RecordList[PAccRcord.Wrecordindex.sectorIndex].TypeResult = BUILD_ACTIONTYPE(type, action);

    PAccRcord.Wrecordindex.sectorIndex++;
    PAccRcord.recordnum++;
	
	RecordIndex_StorePara( );
	
    RecordList_StorePara(PAccRcord.Wrecordindex.pageIndex);

#ifdef ENABLE_CONSOLE_LOG
    printf("p: %u::s:%u\n\r", PAccRcord.Wrecordindex.pageIndex, PAccRcord.Wrecordindex.sectorIndex);
#endif // ENABLE_CONSOLE_LOG
    Access_Record_Null = 0;
}
/****************************************************************************************************
**Function:
    Std_ReturnType Access_GetSupportType(uint8 idtype)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
Std_ReturnType Access_GetSupportType(uint8 idtype)
{
    switch(idtype)
    {
#if(SUPPORT_APP_ID == STD_TRUE)
    case KEY_TYPE_APP: 
    {
        
    }break; 
#endif
#if(SUPPORT_M1_CARD == STD_TRUE)
    case KEY_TYPE_CARD: 
    case KEY_TYPE_CPUCARD:
    {
#ifdef ENABLE_CONSOLE_LOG
        printf("idtype: %d", idtype);
#endif // ENABLE_CONSOLE_LOG
    }break;     
#endif
#if(SUPPORT_QRCODE == STD_TRUE)
    case KEY_TYPE_QRCODE: 
    {
        
    }break;     
#endif
#if(SUPPORT_PWSD == STD_TRUE)
    case KEY_TYPE_PASSWORD:
    {
    }break;     
#endif
#if(SUPPORT_FINGERPRINT == STD_TRUE)
    case KEY_TYPE_FINGERPRINT:
    {
    }break; 
#endif
#if(SUPPORT_IDENTITY_CARD == STD_TRUE)
    case KEY_TYPE_IDENTITY_CARD:
    {
    }break;
#endif
    default:return E_NOT_OK;
    }
    return E_OK;
}
/****************************************************************************************************
**Function:
    void Access_EraseRecordData(void)
**Author: lxw
**Description:
**Input: 
**Output: 根据索引强制修改数据
****************************************************************************************************/
void Access_EraseRecordData(void)
{
    memset((uint8_t *)&AccRcord, 0xff, sizeof(AccRcordParaType));
	AccRcord.TimerBk = BUILD_TIME(18,1,1,1,1,1);
    RecordList_StorePara(0);
	memset((uint8_t *)&PAccRcord, 0xff, sizeof(AccRcordParaindex));
    PAccRcord.Wrecordindex.pageIndex = 0;
    PAccRcord.Wrecordindex.sectorIndex = 0;
    PAccRcord.Srecordindex.pageIndex = 0;
    PAccRcord.Srecordindex.sectorIndex = 0;
    PAccRcord.recordnum = 0;
	RecordIndex_StorePara( );
}

/****************************************************************************************************
**Function:
    void Access_FlashArrang(void)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Access_FlashArrang(void)
{
    uint8 i,j;

    memset(&LockKeyStatistics,0,sizeof(LockKeyStatistics));
    for(i=0; i<BLACK_LIST_MAX; i++)
    {
        if(LockKeyBlackList.BlackList[i].KeyId != 0xffffffff)
        {
            LockKeyStatistics.BlackListCount++;
        }    
    }

    
    for ( j=0;j<SERVE_KEY_PAGEMAX;j++ )
    {
        ServeKeyList_Readflash( j );
        for ( i=0; i<SERVE_KEY_MAX; i++)
          {
            if (LockKeyServeKeyList.KeyList[i].KeyId != 0xffffffff)
              {
                  LockKeyStatistics.ServKeyCount++;
              }
          }
    }
}
uint8 cardopenFlag;

/****************************************************************************************************
**Function:
    void Access_CardProcess(uint8 idtpye, uint8* pUid,Access_CardDatType *CardDat)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Access_CardProcess(uint8 idtpye, uint8* pUid,Access_CardDatType *CardDat)
{
    uint8 cardType;
    Std_ReturnType status = E_OK;
    
    if(Sys_PataCfg.State == WORK_TEST)
    {
        Sys_PataCfg.State = WORK_NORMAL;
    }
#if 0	
    else if(Sys_PataCfg.State == WORK_LEARN_CARD)    
    {
        Sys_PataCfg.State = WORK_NORMAL;
        Access_CardLearnTimer64 = 0;
        status = Access_LearnCardOk(idtpye,pUid);
        if(E_OK == status)
        {
            Access_BeepStart(BEEP_NORMAL, 1);
            Access_LightStart(LIGHT_OK,LIGHT_SLOW, 1, 0);
        }
        else
        {
            Access_BeepStart(BEEP_OPEN_ERR, 1);
#ifdef ENABLE_CONSOLE_LOG
            printf("K10\n\r");
#endif // ENABLE_CONSOLE_LOG
            Access_LightStart(LIGHT_NG,LIGHT_SLOW, 1, 0);
        }
        if(idtpye == KEY_TYPE_IDENTITY_CARD)
        {
            cardType = ACCESS_LEARN_IDENTITY_CARD_TPYE;
#ifdef ENABLE_CONSOLE_LOG
            printf("Write Log: %d\n\r", cardType);
#endif // ENABLE_CONSOLE_LOG
        #if (defined(SUPPORT_RECORD_LOC_STORE)&&(SUPPORT_RECORD_LOC_STORE == STD_TRUE))
            Access_WriteRecordFlash(&pUid[4], Access_UserId_Temp, cardType, status ? false : true);
        #endif
        }
        else
        {
            cardType = ACCESS_LEARN_CARD_TPYE;
#ifdef ENABLE_CONSOLE_LOG
            printf("Write Log: %d\n\r", cardType);
#endif // ENABLE_CONSOLE_LOG
#if (defined(SUPPORT_RECORD_LOC_STORE)&&(SUPPORT_RECORD_LOC_STORE == STD_TRUE))
            Access_WriteRecordFlash(pUid,Access_UserId_Temp, cardType, status ? false : true);
#endif
        }
#if (SUPPORT_RECORD_RTIME_SEND == STD_TRUE)
        ProtoAnaly_AddRecordCmd(pUid,Access_UserId_Temp, cardType, ACCESS_OPEN_LOCK_TPYE);
#endif
        if(IS_CONNECTED())
        {
            Proto_UpdateLearnRecord(pUid,Access_UserId_Temp, cardType, status);
        }
        return;
    }
#endif 	
    else if((Sys_PataCfg.State == WORK_NORMAL)
    #if (defined(CLASS_ROOM_ENABLE)&&(CLASS_ROOM_ENABLE == STD_TRUE))
        || (Sys_PataCfg.State == WORK_NORMALLY_OPEN)
    #endif
        )
    {
        uint8 ret = E_NOT_OK;
        
        ret = LockKey_Check_CardKey(idtpye,pUid);
        if(ret == E_NOT_OK)
        {
            goto ERR;
        }
        else if(ret == E_OK)
        {
            goto RIGHT;
        }
    }
    else if(Sys_PataCfg.State == WORK_NORMALLY_OPEN)
    {
        Sys_PataCfg.State = WORK_NORMAL;
        goto RIGHT;    
    }
    ERR:
	Access_Globle.ErrorTimesfob++;  
	Access_OpenError();
	if(Access_Globle.ErrorTimesfob >= 5)    
	{        
		Access_LockDeviceTimer64msfob = 180000/64; 
		return;
	}
    Access_WriteRecordFlash(pUid, ProtoAnaly_RtcLocalTime, idtpye, false);
    return ;
    RIGHT:
	if(Access_Globle.ErrorTimesfob >= 5)  
	{
	  Access_OpenError();
	  return;
	}
	Access_CardOpenTimer64 = 300/64;	
    cardopenFlag = 1;    
    uint8 action;

    action = ACCESS_OPEN_LOCK_TPYE;
#if (defined(SUPPORT_RECORD_LOC_STORE)&&(SUPPORT_RECORD_LOC_STORE == STD_TRUE))
    if(idtpye == KEY_TYPE_IDENTITY_CARD)
    {
        Access_WriteRecordFlash(&pUid[4],ProtoAnaly_RtcLocalTime, idtpye, action);
    }
    else
    {
#ifdef ENABLE_CONSOLE_LOG
        printf("Write Log: %d\n\r", idtpye);
#endif // ENABLE_CONSOLE_LOG
        Access_WriteRecordFlash(pUid,ProtoAnaly_RtcLocalTime,idtpye,action);
    }
#endif
#if (SUPPORT_RECORD_RTIME_SEND == STD_TRUE)
        ProtoAnaly_AddRecordCmd(pUid,ProtoAnaly_RtcLocalTime,idtpye,ACCESS_OPEN_LOCK_TPYE);
#endif
}

/****************************************************************************************************
**Function:
    void Access_BeepStart(uint8 mode, uint8 times)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Access_BeepStart(Beep_StateType mode, uint8 times)
{
    Beep_ComType Beep_Com;

#ifdef ENABLE_CONSOLE_LOG
    printf("Sys_PataCfg.configMode: 0x%x\n\r", Sys_PataCfg.configMode);
#endif // ENABLE_CONSOLE_LOG
    // Check if Voice mode is enabled
    //
    if(!IS_SET(Sys_PataCfg.configMode, VOICEMODE))
    {
        Beep_Com.state = mode;
        Beep_Com.wkTimes = times;
        BeepDrive.write(&Beep_Com);
    }
#ifdef ENABLE_CONSOLE_LOG
    printf("Beep\n\r");
#endif // ENABLE_CONSOLE_LOG
    //有 beep 时禁止读57卡
//    T5557Drive.release(NULL);        
}
/****************************************************************************************************
**Function:
    void Access_LightStart(uint8 index,uint8 mode, uint8 times,uint16 data)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Access_LightStart(Light_NameType index,Light_StateType mode, uint8 times,uint16 data)
{
    Light_ComType Lightst;

    //Hc595_PowerOn();
    Lightst.lightname=index;
    Lightst.wkTimes = times;
    Lightst.state = mode;
    Lightst.Hc595dat = data;
    LightDrive.write(&Lightst);
    LightDrive.ioctl(LIGHT_CMD_SCAN,NULL);
#ifdef ENABLE_CONSOLE_LOG
    printf("Light\n\r");
#endif // ENABLE_CONSOLE_LOG
}

/****************************************************************************************************
**Function:
    void Access_UpdateStatus(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Access_UpdateStatus(void)
{
}
