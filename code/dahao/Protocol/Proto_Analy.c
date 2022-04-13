/*****************************************************************************
|  File Name: ProtoAnaly.c
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
#include "protocol\Proto_Analy.h"
#include "Protocol\factInfo.h"
#include "Protocol\Proto_ComDef.h"
#include "SYSTEM\sys.h"
#include "SYSTEM\Sys_GenFun.h"
#include "Protocol\Proto_CommPack.h"
#include "RTC\Rtc.h"
#include "Access\Access.h"
#include "Protocol\Proto_NetComm.h"
#include "Beep\Beep.h"
#include "Light\light.h"
#include <cstdlib>
#include "ble_dahao.h"
#include "Access\LockKey.h"
#include "Encpt\Encpt.h"
#include "FACTORY\Factory.h"
#include "CustomCode\CustomCode.h"
#include "crc32.h"
#include "ble_hci.h"
#include "nrf_delay.h"
#ifdef BLE_DFU_APP_SUPPORT
#include "ble_dfu.h"
#include "dfu_app_handler.h"
#endif // BLE_DFU_APP_SUPPORT
#include "KEY\key.h"
#include "Encpt\Aes.h"
#include "KEY\Touch.h"

#define RESET_DISABLE  1
#define RESET_ENABLE   2

#define C32HEX2BYTESMSB(pBuf, data)          \
        *(pBuf + 0) = (uint8)((data) >> 24); \
        *(pBuf + 1) = (uint8)((data) >> 16); \
        *(pBuf + 2) = (uint8)((data) >> 8);  \
        *(pBuf + 3) = (uint8)((data) >> 0)


#define C32HEX2BYTESLSB(pBuf, data)          \
        *(pBuf + 0) = (uint8)((data) >> 0);  \
        *(pBuf + 1) = (uint8)((data) >> 8);  \
        *(pBuf + 2) = (uint8)((data) >> 16); \
        *(pBuf + 3) = (uint8)((data) >> 24)

extern  ble_dfu_t                         m_dfus;                                    /**< Structure used to identify the DFU service. */
extern dm_application_instance_t         m_app_handle;                              /**< Application identifier allocated by device manager */
extern Proto_NetcommType Proto_Netcomm;
extern uint8  Lora_Sync_Flag;
extern uint32 Sys_WkTick ;
extern _Bool ErrCodeBlackListFull;
extern _Bool DeleteOfflineFlag;
extern uint32 DeleteOfflineEndtime;
extern _Bool DFUServiceFlag;
extern uint8_t debugPasscodeFail;
extern CPUFob_DataType CPUFobKey_List;
extern uint8_t fob_sn[4];

extern void ProtoAnaly_LoraSlaveCallBackProc(void);
extern void advertising_init(void);
extern void Beep_once(void);
extern uint8 sys_CheckGetDynamicHotel_V8(uint32 DeviceId,
                                         uint32 DevicePassword,
                                         uint8* pTouchKey,
                                         uint8  length);

#ifdef ENABLE_CONSOLE_LOG
extern void printRTCTime(uint32_t rawTime);
#endif // ENABLE_CONSOLE_LOG

uint8  protoAnaly_netCommType;
uint32 ProtoAnaly_RtcLocalTime;
uint8  ProtoAnaly_GetAckTimes;
uint8  ProtoAnaly_EthTryTimes;
uint16 ProtoAnaly_HeartTimeOutx64ms;
uint8_t AesKeySetFlag;
uint8_t Aeskeytemp[SIZE_AES_KEY];
uint8_t SetParaFlag=0;

Proto_NetcommType Proto_NetcommBack;
Rtc_Type ProtoAnaly_Rtcinfo;

uint8  ProtoAnaly_LoraBrgBuf[30];
uint8  ProtoAnaly_LoraBrgSize;
uint8  ProtoAnalyRecordBuf[PROTOANALY_RECORD_CACHEMAX][16];
uint8  ProtoAnalyRecordIndex =0;
uint8  ProtoAnalyRecordSendErr =0;
uint32 ProtoAnalyHashKey;
uint8  ProtoAnaly_LoraSendTims;
uint8  protoAnaly_freqsetflag;
int16  ProtoAnaly_LoraSsi = 0;
_Bool daylightSavingTimeFlag = false;
_Bool setDstFlag = false;
uint32_t DFUServiceTime = 0;
uint8_t CommandId = 0;
uint16 keyIndex = 0xFF;
uint8 pageId = 0xFF;

uint16 CPUfobIndex ;      
CPUFobType CPUFobRecord;  

LockKey_ServeKeyType keyRecord = {0};
_Bool checkKeyForExpiredKeys = false;
_Bool RtcReadStatus = false;
/*
Sleep: 0:
Hread: 
*/
const uint32 PROTO_SLEEP_TIME_TABLE[]=
{
    PROTOANALY_REC_WINDOWTIMES*(PROTOANALY_CYCLETIMES_MAX -9),
    PROTOANALY_REC_WINDOWTIMES*(PROTOANALY_CYCLETIMES_MAX -(2*8+1)),
    PROTOANALY_REC_WINDOWTIMES*(PROTOANALY_CYCLETIMES_MAX -(4*8+1)),
};

const uint8 PROTO_BREAD_TIME_TABLE[]=
{
8,8,28,
};

Std_ReturnType ValidateKeyDeleteOperation(uint8 flashOffset,
                                           uint16 index,
                                           uint32_t passCode);

/****************************************************************************************************
**Function:
    void ProtoAnaly_EthAckOk(Proto_NetcommType CommType)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_EthAckOk(Proto_NetcommType CommType)
{
    protoAnaly_netCommType = CommType;
}
extern Rtc_Type RTC_CheckData;

/****************************************************************************************************
**Function:
    void ProtoAnaly_UpdateTime(void)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_UpdateTime(void)
{
    Rtc_Type Rtc_data;
    uint32 time = 0;

    RtcReadStatus = false;
    if(Rtc_Read(&Rtc_data)!= E_OK)
    {
        return;
    }
	memcpy(&RTC_CheckData,&Rtc_data,sizeof(RTC_CheckData));
    if((Rtc_data.Year > 99 )||(Rtc_data.Month > 12)||(Rtc_data.DayOfMonth> 31)
        ||(Rtc_data.Hour> 23)||(Rtc_data.Minute> 60)||(Rtc_data.Second> 60))
    {
        return;
    }
    memcpy(&ProtoAnaly_Rtcinfo,&Rtc_data,sizeof(ProtoAnaly_Rtcinfo));
    time = (((uint32)ProtoAnaly_Rtcinfo.Year << 26) & 0xfc000000) + (((uint32)ProtoAnaly_Rtcinfo.Month << 22) & 0x3c00000)
    + (((uint32)ProtoAnaly_Rtcinfo.DayOfMonth << 17) & 0x3e0000) + (((uint32)ProtoAnaly_Rtcinfo.Hour << 12) & 0x1f000)
    + (((uint32)ProtoAnaly_Rtcinfo.Minute << 6) & 0xfc0) + (ProtoAnaly_Rtcinfo.Second & 0x3f);

    ProtoAnaly_RtcLocalTime = time;
    AccRcord.TimerBk = time;

    ProtoAnaly_Rtcinfo.DayOfWeek = Main_GetWeekFromDay(ProtoAnaly_Rtcinfo.Year,ProtoAnaly_Rtcinfo.Month,ProtoAnaly_Rtcinfo.DayOfMonth);
    RtcReadStatus = true;
 }


 /****************************************************************************************************
 **Function:
     void ProtoAnaly_UpdateTime(void)
 **Author: rory
 **Description:
 **Input: 
 **Output: 
 ****************************************************************************************************/
 void ProtoAnaly_LoraSetTime(uint32 rtime)
 {
     Rtc_Type Rtc_data;

     Rtc_data.Year = ((rtime& 0xfc000000) >> 26) ;
     Rtc_data.Month = ((rtime& 0x3c00000) >> 22) ;
     Rtc_data.DayOfMonth = ((rtime& 0x3e0000) >> 17) ;
     Rtc_data.Hour = ((rtime& 0x1f000) >> 12) ;
     Rtc_data.Minute = ((rtime& 0xfc0) >> 6) ;
     Rtc_data.Second = (rtime& 0x3f) ;
     Rtc_Ioctl(RTC_CLOCKSET,&Rtc_data);
	 RtcLocalTimebak = 0;
  }

 /****************************************************************************************************
 **Function:
     void ProtoAnaly_HashKeyInit(void)
 **Author: lxw
 **Description:  
 **Input: 
 **Output: 
 ****************************************************************************************************/
 uint32 ProtoAnaly_HashKeyInit(uint32 paswd)
{
    uint8 hashbuf[8];
    srand(Sys_PataCfg.Mac);
    
    Sys_GenFun32To8(paswd,hashbuf);
    Sys_GenFun32To8(Sys_PataCfg.Mac,&hashbuf[4]);
//  ProtoAnalyHashKey= BKDRHash(hashbuf,8);
    return BKDRHash(hashbuf,8);
}

 /****************************************************************************************************
 **Function:
     void ProtoAnaly_Init(void)
 **Author: lxw
 **Description:  
 **Input: 
 **Output: 
 ****************************************************************************************************/
 void ProtoAnaly_Init(void)
{
    Proto_PackfunCallBack Proto_Pack;
#if (defined LORA_ENABLE) && (LORA_ENABLE == STD_TRUE)
    Sys_PataCfg.rfPara.ConnectCallBack = NULL;
    Sys_PataCfg.rfPara.ProcCallBack=Proto_CommHostUnPackRec;
    RadioDrive.open(&Sys_PataCfg.rfPara);
#endif


/*    Proto_Pack.Tc_TargeAdd    = NET_NULL;
    Proto_Pack.Tc_SourceAdd1 = NET_NULL;
    Proto_Pack.Tc_SourceAdd2 = NET_NULL;
*/
    Proto_Pack.AppProcCallBack = ProtoAnaly_CmdAppProc;
    Proto_Pack.MacProcCallBack = ProtoAnaly_CmdMacProc;
    Proto_Pack.ProcEndCallBack = ProtoAnaly_ProcEnd;
    Proto_Pack.ProcNetUpdateCallBack = ProtoAnaly_EthAckOk;
    Proto_Pack.PublicProcCallBack = ProtoAnaly_PublicProc;
    Proto_CommPrackInit(&Proto_Pack);
    ProtoAnalyHashKey = ProtoAnaly_HashKeyInit(Sys_PataCfg.Pswd);
    protoAnaly_freqsetflag = 0;
    Sys_SleepErrTimsoutx64ms = 0;
}
#if (defined LORA_ENABLE) && (LORA_ENABLE == STD_TRUE)
 /****************************************************************************************************
 **Function:
 void ProtoAnaly_SendHeartBeat(uint8 cmd,uint8 ch,uint8 state)
 **Author: lxw
 **Description:  
 **Input: 
 **Output: 
 ****************************************************************************************************/
 void ProtoAnaly_SendHeartBeat(uint8 cmd,uint8 ch,uint8 state)
 {
    uint8 datbuf[30]={0};
    datbuf[0] = 4;
    datbuf[1] = COMD_GET_HEARTBEAT;
    datbuf[8] = state;
    datbuf[9] = TYPE_LOCK_ACCESS_REMOTE;
    datbuf[10] = 0;
    Proto_CommSend(LORA,datbuf);
 }
#endif // #if (defined LORA_ENABLE) && (LORA_ENABLE == STD_TRUE)
/****************************************************************************************************
**Function:
    void ProtoAnaly_CleanErr(void)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
uint16    ProtoAnaly_GprsHeartTimeOutx16ms;
uint16     ProtoAnaly_GprsHeartErr;
uint8     ProtoAnaly_GprsRstHeartTimes= 0;

unsigned char IsOfflineCode(unsigned char *pTouchKey, unsigned char lenth)
{
     unsigned char ret=E_NOT_OK;

     DeleteOfflineFlag=1;

     ret = sys_CheckGetDynamicDate(Sys_PataCfg.Mac, Sys_PataCfg.Pswd, pTouchKey, lenth);
 
     if(ret == E_NOT_OK)
     {
             ret = sys_CheckGetDynamicFlat(Sys_PataCfg.Mac, Sys_PataCfg.Pswd, pTouchKey, lenth);//
     }

     if(ret == E_NOT_OK)
     {
             ret = sys_CheckGetDynamicHotel_V8(Sys_PataCfg.Mac, Sys_PataCfg.Pswd, pTouchKey, lenth);
     }

     DeleteOfflineFlag=0;

     return(ret);
}

void DeleteCPUID(uint32 keyid)
{
	uint8 i=0, j=0;
	uint8 idbuf[4];

    C32HEX2BYTESMSB(idbuf,keyid );
	
	for(j=0; j<CPUFOB_PAGEMAX; j++ )
	{
		CpuFobList_Readflash(j);
		for(i=0; i<CPUFOB_MAX; i++)
		{
			if( CPUFobKey_List.CpuFobList[i].Fob_SN[0]==idbuf[0] && 
				CPUFobKey_List.CpuFobList[i].Fob_SN[1]==idbuf[1] && 
				CPUFobKey_List.CpuFobList[i].Fob_SN[2]==idbuf[2] && 
				CPUFobKey_List.CpuFobList[i].Fob_SN[3]==idbuf[3] )
			{
				CPUfobIndex = i;
				memcpy(&CPUFobRecord,
				(const CPUFob_DataType *)&CPUFobKey_List.CpuFobList[i],
				sizeof(CPUFobRecord));	
				memset(&CPUFobKey_List.CpuFobList[i],0xff, 20 );
				CpuFobList_Writeflash(j);
				return;
			}
		}
	}
}


/****************************************************************************************************
**Function:
    Std_ReturnType ProtoAnaly_GetHeartbeatIn(uint32 id)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
Std_ReturnType ProtoAnaly_RemoveId(uint8 idtpye,uint8* id)
{    
    uint8 i = 0;
    uint8 j = 0;
    uint32 endTime = 0;
    uint8 ret = E_KEY_NOT_FOUND;
    uint32 keyId;

    if(idtpye == KEY_TYPE_IDENTITY_CARD)
    {
        keyId = BUILD_UINT32(id[7], id[6], id[5], id[4]);
    }
    else
    {
        keyId = BUILD_UINT32(id[3], id[2], id[1], id[0]);
    }
    
#ifdef ENABLE_CONSOLE_LOG
    printf("Type: %u::Key: 0x%lx\n\r", idtpye, keyId);
#endif // ENABLE_CONSOLE_LOG

#if (defined RTC_EN) && (RTC_EN == STD_TRUE)
#ifdef ENABLE_CONSOLE_LOG
    printf("Update time...\n\r");
#endif // ENABLE_CONSOLE_LOG
    ProtoAnaly_UpdateTime();
#endif

    if(keyId == 0xffffffff)
    {
        ret = E_INVALID_KEY_VALUE;
    }
    else
    {
        for(j = 0; j < SERVE_KEY_PAGEMAX; j++)
        {
            ServeKeyList_Readflash(j);
            for(i = 0; i < SERVE_KEY_MAX; i++)
            {
                if(keyId == LockKeyServeKeyList.KeyList[i].KeyId)
                {
#ifdef ENABLE_CONSOLE_LOG
                    printf("Found code at %u:%u\n\r", j, i);
#endif // ENABLE_CONSOLE_LOG
                    if((idtpye == KEY_TYPE_ONE_TIME)    ||
                       (idtpye == KEY_TYPE_FOREVER)     ||
                       (idtpye == KEY_TYPE_TIME_SLOT_7) ||
                       (idtpye == KEY_TYPE_CLEAR)       ||
                       (idtpye == KEY_TYPE_TIME_SLOT_8))
                    {
#ifdef ENABLE_CONSOLE_LOG
                        printf("Adding code to blacklist...\n\r");
#endif // ENABLE_CONSOLE_LOG

                        BlackList_Add(LockKeyServeKeyList.KeyList[i].Type, LockKeyServeKeyList.KeyList[i].KeyId,
                        BUILD_TIME(ProtoAnaly_Rtcinfo.Year,ProtoAnaly_Rtcinfo.Month,ProtoAnaly_Rtcinfo.DayOfMonth,
                        23,59,59));
                        BlackList_StorePara();
                    }
                    else
                    {
                        if(idtpye != LockKeyServeKeyList.KeyList[i].Type)
                        {
                            return E_INVALID_KEY_TYPE;
                        }
                    }

					if ( KEY_TYPE_CPUCARD == idtpye )
					{
					  DeleteCPUID( keyId );
					}
                    keyIndex = i;
                    pageId = j;
                    memcpy(&keyRecord,
                           (const LockKey_ServeKeyListType *)&LockKeyServeKeyList.KeyList[i],
                           sizeof(keyRecord));
                    memset(&LockKeyServeKeyList.KeyList[i], 0xff, sizeof(LockKey_ServeKeyType));
                    if(LockKeyStatistics.ServKeyCount > 0)
                    {
                        LockKeyStatistics.ServKeyCount--;
#ifdef ENABLE_CONSOLE_LOG
                        printf("Code Count: %u\n\r", LockKeyStatistics.ServKeyCount);
#endif // ENABLE_CONSOLE_LOG
                    }

#ifdef ENABLE_CONSOLE_LOG
                    printf("Writing to flash...\n\r");
#endif // ENABLE_CONSOLE_LOG
                    ServeKeyList_Writeflash(j);

                    return ValidateKeyDeleteOperation(pageId, keyIndex, keyId);
                }
            }
        }
    }

    if(true == ErrCodeBlackListFull)
    {
        ErrCodeBlackListFull = false;
        return E_MAX_BLACKLIST_COUNT;
    }

    if((idtpye == KEY_TYPE_ONE_TIME) || (idtpye == KEY_TYPE_FOREVER) ||
       (idtpye == KEY_TYPE_CLEAR)    || (idtpye == KEY_TYPE_TIME_SLOT_8))
    {
#ifdef ENABLE_CONSOLE_LOG
        printf("??? Adding code to BLACKLIST...\n\r");
#endif // ENABLE_CONSOLE_LOG
        if(E_OK == IsOfflineCode(&id[0], 8))
        {
            if(KEY_TYPE_TIME_SLOT_8 == idtpye ||
               KEY_TYPE_FOREVER     == idtpye ||
               KEY_TYPE_TIME_SLOT_7 == idtpye)
            {
               endTime = DeleteOfflineEndtime;
            }
            /* 加入黑名单*/
            BlackList_Add(idtpye, keyId, endTime);
            BlackList_StorePara();
            ret = E_OK;
        }
    }
    return ret;
}

Std_ReturnType ValidateKeyDeleteOperation(uint8 flashOffset,
                                          uint16 index,
                                          uint32_t passCode)
{
    Std_ReturnType status = E_KEY_NOT_DELETED;

    memset(&LockKeyServeKeyList, 0, sizeof(LockKeyServeKeyList));

    ServeKeyList_Readflash(flashOffset);

    if(passCode != LockKeyServeKeyList.KeyList[index].KeyId)
    {
#ifdef ENABLE_CONSOLE_LOG
        printf("SUCCESS: Key delete\n\r");
#endif // ENABLE_CONSOLE_LOG
        status = E_OK;
    }
#ifdef ENABLE_CONSOLE_LOG
    else
    {
        printf("FAILURE: Key delete\n\r");
    }
#endif // ENABLE_CONSOLE_LOG
#ifdef ENABLE_CONSOLE_LOG
    printf("Type: %u\n\r", LockKeyServeKeyList.KeyList[index].Type);
    printf("Key ID: %lu\n\r", LockKeyServeKeyList.KeyList[index].KeyId);
#endif // ENABLE_CONSOLE_LOG

    return status;
}

Std_ReturnType ValidateFlashWriteOperation(uint8 flashOffset,
                                           uint16 index,
                                           uint32_t passCode,
                                           uint8_t IdType,
                                           uint32_t endTime,
                                           uint32_t startTime,
                                           uint8_t isCyclic)
{
    Std_ReturnType status = E_KEY_NOT_SET;
    memset(&LockKeyServeKeyList, 0, sizeof(LockKeyServeKeyList));
    ServeKeyList_Readflash(flashOffset);

    if(passCode == LockKeyServeKeyList.KeyList[index].KeyId &&
       IdType == LockKeyServeKeyList.KeyList[index].Type &&
       endTime == LockKeyServeKeyList.KeyList[index].EndTime &&
       startTime == LockKeyServeKeyList.KeyList[index].StartTime &&
       isCyclic == LockKeyServeKeyList.KeyList[index].Cycle)
    {
#ifdef ENABLE_CONSOLE_LOG
        printf("SUCCESS: Key set validation\n\r");
#endif // ENABLE_CONSOLE_LOG
        status = E_OK;
    }
#ifdef ENABLE_CONSOLE_LOG
    else
    {
        printf("FAILURE: Key set validation\n\r");
    }
#endif // ENABLE_CONSOLE_LOG
#ifdef ENABLE_CONSOLE_LOG
    printf("Type: %u\n\r", LockKeyServeKeyList.KeyList[index].Type);
    printf("Key ID: %lu\n\r", LockKeyServeKeyList.KeyList[index].KeyId);
    printf("Endtime: 0x%lx\n\r", LockKeyServeKeyList.KeyList[index].EndTime);
    printRTCTime(LockKeyServeKeyList.KeyList[index].EndTime);
    printf("Starttime: 0x%lx\n\r", LockKeyServeKeyList.KeyList[index].StartTime);
    printRTCTime(LockKeyServeKeyList.KeyList[index].StartTime);
    printf("Cycle: %u\n\r", LockKeyServeKeyList.KeyList[index].Cycle);
#endif // ENABLE_CONSOLE_LOG

    return status;
}

_Bool isKeyExpired(uint32 time)
{
    uint32 LocalTime = ProtoAnaly_RtcLocalTime;
#ifdef ENABLE_CONSOLE_LOG
    printRTCTime(LocalTime);
    printRTCTime(time);
#endif
    return (LocalTime > time);
}

void AddCPUfob_Store(uint32 id, uint8 *pkey)
{
 	uint8 i=0,j=0;
	for(j=0; j<CPUFOB_PAGEMAX; j++ )
	{
		CpuFobList_Readflash(j);
		for(i=0; i<CPUFOB_MAX; i++)
		{
			if ( CPUFobKey_List.CpuFobList[i].Fob_SN[0]==0xFF && 
				 CPUFobKey_List.CpuFobList[i].Fob_SN[1]==0xFF && 
				 CPUFobKey_List.CpuFobList[i].Fob_SN[2]==0xFF && 
				 CPUFobKey_List.CpuFobList[i].Fob_SN[3]==0xFF 
				)
				{
					CPUfobIndex = i;
					memcpy(&CPUFobRecord,
					(const CPUFob_DataType *)&CPUFobKey_List.CpuFobList[i],
					sizeof(CPUFobRecord));	
	 
					C32HEX2BYTESMSB(CPUFobKey_List.CpuFobList[i].Fob_SN, id);
					memcpy (&CPUFobKey_List.CpuFobList[i].Fob_KeyArray,pkey,16 );
					CpuFobList_Writeflash(j);
					return;
				}
		 }
	}
}

	
/****************************************************************************************************
**Function:
    void ProtoAnaly_AddId(uint8 idtpye,uint32 id)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
Std_ReturnType ProtoAnaly_AddId(uint8 idtpye,uint8 *idbuf,uint8 num)
{
    uint16 index = 0xff;
    uint8 flashoffset = 0;
    uint8 i = 0;
    uint8 j = 0;
    uint8 type;
    uint8 err = E_KEY_NOT_SET;
    uint32 keyId;
    uint32_t endTime = 0;
    uint32_t startTime = 0;
    uint32_t shiftBuffer = 0;
    uint8_t cyclicInfo = 0;
    uint8 decryptedData[16];

    keyId = BUILD_UINT32(idbuf[3], idbuf[2], idbuf[1], idbuf[0]);

    if(keyId == 0xffffffff)
    {
        return E_INVALID_KEY_VALUE;
    }

#ifdef ENABLE_CONSOLE_LOG
    printf("Pass-code: %lu: %lx\n\r", keyId, keyId);
#endif // ENABLE_CONSOLE_LOG
    for(j = 0; j < SERVE_KEY_PAGEMAX; j++)
    {
        ServeKeyList_Readflash(j);
        for(i = 0; i < SERVE_KEY_MAX; i++)
        {
            if(keyId == LockKeyServeKeyList.KeyList[i].KeyId)
            {//强制修改有效时间
#ifdef ENABLE_CONSOLE_LOG
                printf("Found same code: 0x%lx\n\r", keyId);
                printf("%u:%u\n\r", j, i);
                printRTCTime(ProtoAnaly_RtcLocalTime);
                printRTCTime(LockKeyServeKeyList.KeyList[i].EndTime);
#endif // ENABLE_CONSOLE_LOG
                if(COMD_UPDATE_PASSKEY == CommandId)
                {
#ifdef ENABLE_CONSOLE_LOG
                    printf("Received update passkey command\n\r");
#endif // ENABLE_CONSOLE_LOG
                    pageId = flashoffset = j;
                    keyIndex = index = i;
                }
                else if(isKeyExpired(LockKeyServeKeyList.KeyList[i].EndTime))
                {
#ifdef ENABLE_CONSOLE_LOG
                    printf("Found expired key with same code\n\r");
#endif // ENABLE_CONSOLE_LOG
                    pageId = flashoffset = j;
                    keyIndex = index = i;
                }
                else
                {
                    return E_DUPLICATE_KEY;
                }
            }
            if( j==0 && LockKeyBlackList.BlackList[i].KeyId == keyId)
            {
                return E_DUPLICATE_KEY;
            }
				
            if(index == 0xff
               && COMD_UPDATE_PASSKEY != CommandId )	
            {
#ifdef ENABLE_CONSOLE_LOG
                printf("%u::%u\n\r", j, i);
#endif // ENABLE_CONSOLE_LOG
                if(LockKeyServeKeyList.KeyList[i].KeyId == 0xffffffff)
                {
                    pageId = flashoffset = j;
                    keyIndex = index = i;
#ifdef ENABLE_CONSOLE_LOG
                    printf("flashoffset: %u::index: %u\n\r", j, i);
#endif // ENABLE_CONSOLE_LOG
                    //break;
                }
            }
        }
    }
#ifdef ENABLE_CONSOLE_LOG
    printf("flashoffset: %u::index: %u\n\r", flashoffset, index);
#endif // ENABLE_CONSOLE_LOG
    if(index != 0xff)
    {//增新
        ServeKeyList_Readflash(flashoffset);

        keyIndex = index;
        pageId = flashoffset;
        memcpy(&keyRecord,
               (const LockKey_ServeKeyListType *)&LockKeyServeKeyList.KeyList[index],
               sizeof(keyRecord));
        LockKeyServeKeyList.KeyList[index].Type = idtpye;
        LockKeyServeKeyList.KeyList[index].KeyId = keyId;
        if(COMD_ADD_CPUFOB == CommandId)
        {
            shiftBuffer = 16;
			
			AES_Decrypt(&idbuf[4], decryptedData);
			memcpy(&idbuf[4], decryptedData, 16);

			AddCPUfob_Store( keyId ,&idbuf[4]);
        }
        else
        {
            shiftBuffer = 0;
        }
        endTime = BUILD_UINT32(idbuf[7+shiftBuffer], idbuf[6+shiftBuffer], idbuf[5+shiftBuffer], idbuf[4+shiftBuffer]);
        startTime = BUILD_UINT32(idbuf[11+shiftBuffer], idbuf[10+shiftBuffer], idbuf[9+shiftBuffer], idbuf[8+shiftBuffer]);
        cyclicInfo = idbuf[12+shiftBuffer];

        LockKeyServeKeyList.KeyList[index].EndTime = endTime;
        LockKeyServeKeyList.KeyList[index].StartTime = startTime;
        LockKeyServeKeyList.KeyList[index].Cycle = cyclicInfo;
#ifdef ENABLE_CONSOLE_LOG
        printf("endTime: ");
        printRTCTime(endTime);
        printf("startTime: ");
        printRTCTime(startTime);
        printf("\n\r");
#endif
        if(COMD_UPDATE_PASSKEY != CommandId)
        {
            LockKeyStatistics.ServKeyCount++;
        }
        //LockKeyStatistics.ServKeyCount++;
#ifdef ENABLE_CONSOLE_LOG
        printf("Writing into flash...Type: %u::Key: %lu::offset: %u::index: %u::Count: %u\n\r", 
        idtpye, keyId, flashoffset, index, LockKeyStatistics.ServKeyCount);
#endif // ENABLE_CONSOLE_LOG
        ServeKeyList_Writeflash( flashoffset );
        return ValidateFlashWriteOperation(flashoffset, index, keyId, idtpye, endTime, startTime, cyclicInfo);
    }
	else
	{
        if(COMD_UPDATE_PASSKEY == CommandId)
        {
            return E_KEY_NOT_FOUND;
        }	
	}

    for(j = 0; j < SERVE_KEY_PAGEMAX; j++)
    {
        ServeKeyList_Readflash(j);
        for(i = 0; i < SERVE_KEY_MAX; i++)
        {
            type = LockKeyServeKeyList.KeyList[i].Type;
            if(((type == KEY_TYPE_ONE_TIME) || (type == KEY_TYPE_TIME_SLOT_7) 
                || (type == KEY_TYPE_TIME_SLOT_8)) && (LockKeyServeKeyList.KeyList[i].KeyId != 0xffffffff))
            {
                LockKeyServeKeyList.KeyList[i].Type = idtpye;
                LockKeyServeKeyList.KeyList[i].KeyId = keyId;
                LockKeyServeKeyList.KeyList[i].EndTime = BUILD_UINT32(idbuf[7], idbuf[6], idbuf[5], idbuf[4]);
                LockKeyServeKeyList.KeyList[i].StartTime = BUILD_UINT32(idbuf[11], idbuf[10], idbuf[9], idbuf[8]);
                LockKeyServeKeyList.KeyList[i].Cycle = idbuf[12];
#ifdef ENABLE_CONSOLE_LOG
                printf("???\n\r");
#endif // ENABLE_CONSOLE_LOG
                ServeKeyList_Writeflash(j);
                return E_OK;
            }
        }
    }
    return err;
}
extern uint32 Checkcnt;

/****************************************************************************************************
**Function:
    void ProtoAnaly_SetTimeIn(uint8_t *pInData ,uint8_t *pOutData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_SetTimeIn(uint8_t *pInData ,uint8_t *pOutData)
{
    Rtc_Type Rtc_data;
    Rtc_Timestamp	Rtc_Tsdata;

    pOutData[0] = 2;             /* 长度*/
    pOutData += 7;
    Rtc_data.Year = pInData[5];
    Rtc_data.Month= pInData[6];
    Rtc_data.DayOfMonth= pInData[7];
    Rtc_data.Hour= pInData[8];
    Rtc_data.Minute= pInData[9];
    Rtc_data.Second= pInData[10];
    setDstFlag = true;
    pOutData[1] = Rtc_Ioctl(RTC_CLOCKSET,&Rtc_data);

    //Rtc_data.Year = Rtc_data.Year + (2000-1970);
	Rtc_Tsdata.Year = Rtc_data.Year + 2000;
    Rtc_Tsdata.Month= Rtc_data.Month;
    Rtc_Tsdata.DayOfMonth= Rtc_data.DayOfMonth;
    Rtc_Tsdata.Hour= Rtc_data.Hour;
    Rtc_Tsdata.Minute= Rtc_data.Minute;
    Rtc_Tsdata.Second= Rtc_data.Second;	
	Sys_PataCfg.settimeBak = mktime(Rtc_Tsdata);

    RtcLocalTimebak = 0;
	
    Sys_StoreFlag = STD_TRUE;
	Checkcnt = 0;
    Access_BeepStart(BEEP_NORMAL,1);
}
/****************************************************************************************************
**Function:
    void ProtoAnaly_GetTimeIn(uint8_t *pInData ,uint8_t *pOutData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_GetTimeIn(uint8_t *pInData ,uint8_t *pOutData)
{
    pOutData[0] = 8; /* 长度*/
    pOutData += 7;
#if (defined RTC_EN) && (RTC_EN == STD_TRUE)
    ProtoAnaly_UpdateTime();
#endif

    if(true == RtcReadStatus)
    {
        pOutData[1]= E_OK;

        pOutData[2]= ProtoAnaly_Rtcinfo.Year;
        pOutData[3] = ProtoAnaly_Rtcinfo.Month;
        pOutData[4] = ProtoAnaly_Rtcinfo.DayOfMonth;
        pOutData[5]= ProtoAnaly_Rtcinfo.Hour;
        pOutData[6] =ProtoAnaly_Rtcinfo.Minute;
        pOutData[7]= ProtoAnaly_Rtcinfo.Second;

        pOutData[8]= Sys_PataCfg.bleDSTFlag & 0x01;
    }
    else
    {
        RtcReadStatus = true;
        pOutData[1]= E_RTC_READ_ERR;
    }
    
    // LOCK-240 - Disable beep sound
    //
    //Access_BeepStart(BEEP_NORMAL,1);
}
/****************************************************************************************************
**Function:
    void ProtoAnaly_ConfigHostIn(uint8_t *pInData ,uint8_t *pOutData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_ConfigHostIn(uint8_t *pInData ,uint8_t *pOutData)
{
    uint16 heart;
    pOutData[0] = 2;             /* 长度*/
    pOutData += 7;
    heart = BUILD_UINT16(pInData[7], pInData[6]);
    if((heart > 60*60) || (heart < 10))
    {
        return;
    }
    //Sys_PataCfg.UploadRecord = pInData[5];
    Sys_PataCfg.HeartTime = heart;
    pOutData[1] = E_OK;
    Access_BeepStart(BEEP_NORMAL,1);
}
/****************************************************************************************************
**Function:
    void ProtoAnaly_ReaderUploadId(uint8_t *pInData ,uint8_t *pOutData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_ReaderUploadCardId(uint8* pCardId, uint8 type)
{
    uint8 Databuffer[30];
//    uint8 index = 0;
    *((uint16*)Databuffer)= Sys_GenFunhtons(PROTO_COMMPACK_HOST_DAT_OUT);
    Databuffer[3] = COMD_READER_UNLOAD_CARD_ID; // command
    *((uint16*)(&Databuffer[4]))= Sys_GenFunhtons(Sys_PataCfg.SysId);
    *((uint32*)(&Databuffer[6]))= Sys_GenFunhtonl(Sys_PataCfg.Mac);
    Databuffer[10] = E_OK; // status
    Databuffer[11] = type; // Type

    if(type == KEY_TYPE_CARD)
    {
        Databuffer[2] = 14;  // length;
        Databuffer[12] = pCardId[0];
        Databuffer[13] = pCardId[1];
        Databuffer[14] = pCardId[2];
        Databuffer[15] = pCardId[3];
        Databuffer[16]  = Sys_GenFunChecksum(&Databuffer[2]);
    }
    else
    {
        Databuffer[2] = 18;  // length;
        Databuffer[12] = pCardId[0];
        Databuffer[13] = pCardId[1];
        Databuffer[14] = pCardId[2];
        Databuffer[15] = pCardId[3];
        Databuffer[16] = pCardId[0];
        Databuffer[17] = pCardId[1];
        Databuffer[18] = pCardId[2];
        Databuffer[19] = pCardId[3];
        Databuffer[20]  = Sys_GenFunChecksum(&Databuffer[2]);
    }
}

/****************************************************************************************************
**Function:
    void ProtoAnaly_ConfigDeviceExt(uint8_t *pInData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_ConfigDeviceExtIn(uint8_t *pInData, uint8_t *pOutData)
{
    pOutData[0] = 2;             /* 长度*/
    pOutData += 7;
    pOutData[1] = E_OK;

    if(pInData[5] == CONFIG_KEY_AUTOLOCK)
    {
        if(pInData[6] == 1)
        {
            Sys_PataCfg.CfgFlag |= AUTOLOCK;
            Sys_PataCfg.OpenLockTime = BUILD_UINT16(pInData[8], pInData[7]);
        }
        else
        {
            pOutData[1] = E_INVALID_MODE;
        }
    }
    else if(pInData[5] == CONFIG_KEY_BTSAVEMODE)
    {
        if(pInData[6] == 1)
        {
            Sys_PataCfg.CfgFlag |= BTSAVEMODE;
        }
        else
        {
            Sys_PataCfg.CfgFlag &= ~BTSAVEMODE;
        }
    }
    else if(pInData[5] == CONFIG_KEY_DETCARDSLEEP)
    {
        if(pInData[6] == 1)
        {
            Sys_PataCfg.CfgFlag |= DETCARDSLEEP;
        }
        else
        {
            Sys_PataCfg.CfgFlag &= ~DETCARDSLEEP;
        }
    }
    else if(pInData[5] == CONFIG_KEY_DISIBLEDOORLOCK)
    {
        if(pInData[6] == 1)
        {
            Sys_PataCfg.CfgFlag |= DISIBLEDOORLOCK;
        }
        else
        {
            Sys_PataCfg.CfgFlag &= ~DISIBLEDOORLOCK;
        }
    }
    
    Sys_StoreFlag = STD_TRUE;
	// External flash,not required reboot
    // Sys_McuRestFlag = STD_TRUE;
    // LOCK-240 - Disable beep sound
    //
    //Access_BeepStart(BEEP_FAST,1);
}


/****************************************************************************************************
**Function:
    void ProtoAnaly_ReadDeviceInfoExtIn(uint8_t *pInData ,uint8_t *pOutData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_ReadDeviceInfoExtIn(uint8_t *pInData ,uint8_t *pOutData)
{
    pOutData[0] = 2+2; /* 长度*/
    pOutData += 7;
    pOutData[1]= E_OK;
    pOutData[2] = pInData[5];
    pOutData[3] = 0;

    if(pInData[5] == CONFIG_KEY_AUTOLOCK)
    {
        if(Sys_PataCfg.CfgFlag&AUTOLOCK)
        {
            pOutData[3] = 1;
            pOutData[4] = HI_UINT16(Sys_PataCfg.OpenLockTime);
            pOutData[5] = LO_UINT16(Sys_PataCfg.OpenLockTime);
            pOutData -= 7;
            pOutData[0] += 2;
        }
    }
    else if(pInData[5] == CONFIG_KEY_BTSAVEMODE)
    {
        if(Sys_PataCfg.CfgFlag&BTSAVEMODE)
        {
            pOutData[3] = 1;
        }
    }
    else if(pInData[5] == CONFIG_KEY_DETCARDSLEEP)
    {
        if(Sys_PataCfg.CfgFlag&DETCARDSLEEP)
        {
            pOutData[3] = 1;
        }
    }
    else if(pInData[5] == CONFIG_KEY_DISIBLEDOORLOCK)
    {
        if(Sys_PataCfg.CfgFlag&DISIBLEDOORLOCK)
        {
            pOutData[3] = 1;
        }
    }
    // LOCK-240 - Disable beep sound
    //
    //Access_BeepStart(BEEP_FAST,1);
}

/****************************************************************************************************
**Function:
    void ProtoAnaly_ModifyPaswdIn(uint8_t *pInData ,uint8_t *pOutData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_ModifyPaswdIn(uint8_t *pInData ,uint8_t *pOutData)
{
    pOutData[0] = 2;             /* 长度*/
    pOutData += 7;

       Sys_PataCfg.Pswd = BUILD_UINT32(pInData[8], pInData[7], pInData[6], pInData[5]);
    pOutData[1] = E_OK;
    Sys_StorePara();
    Access_BeepStart(BEEP_FAST,1);
}

/****************************************************************************************************
**Function:
    void ProtoAnaly_GetTimeIn(uint8_t *pInData ,uint8_t *pOutData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_PowerOnIn(uint8 *pInData ,uint8 *pOutData)
{
    uint8 ret = E_OK;
    
    pOutData[0] = 3; /* 长度*/
    pOutData += 7;
#if (defined CLASS_ROOM_ENABLE) && (CLASS_ROOM_ENABLE == STD_FALSE)
    if(Sys_PataCfg.State == WORK_NORMALLY_OPEN)
    {
        Access_BeepStart(BEEP_LOW,BEEP_ERROR_EMERGENCY);
        pOutData[1]= ret;
        pOutData[2]= Access_BatteryData;
        return;
    }
#endif
    Access_Unlock();
    pOutData[1]= ret;
    pOutData[2]= Access_BatteryData;
}
/****************************************************************************************************
**Function:
    void ProtoAnaly_GetTimeIn(uint8_t *pInData ,uint8_t *pOutData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_PowerOffIn(uint8 *pInData ,uint8 *pOutData)
{
    
    pOutData[0] = 3; /* 长度*/
    pOutData += 7;
    pOutData[1]= E_OK ;
    pOutData[2]= Access_BatteryData;
    Access_Lock();
}
void ProtoAnaly_DFUEnable(uint8 *pInData ,uint8 *pOutData)
{
    pOutData[0] = 2;
	pOutData += 7;

    pOutData[1]= E_OK;
    DFUServiceFlag = true;
    DFUServiceTime = 60000 / 64;
}

uint8 tbuf[20];
/****************************************************************************************************
**Function:
    void ProtoAnaly_SetListIn(uint8_t *pInData ,uint8_t *pOutData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_SetListIn(uint8 *pInData ,uint8 *pOutData)
{
    pOutData[0] = 4;            /* 长度*/
    pOutData +=7;
    memcpy(tbuf,pInData,20);
    pInData +=4;

#ifdef ENABLE_CONSOLE_LOG
    printf("LockKeyStatistics.ServKeyCount: %u\n\r", LockKeyStatistics.ServKeyCount);
#endif
    if(LockKeyStatistics.ServKeyCount > (SERVE_KEY_PAGEMAX * SERVE_KEY_MAX))    //  SERVE_KEY_MAX
    {
        pOutData[1]= E_MAX_KEY_COUNT;
        return;
    }

    if((pInData[1] == KEY_TYPE_ONE_TIME) || (pInData[1] == KEY_TYPE_FOREVER)
            || (pInData[1] == KEY_TYPE_TIME_SLOT_7) || (pInData[1] == KEY_TYPE_CLEAR)
            || (pInData[1] == KEY_TYPE_TIME_SLOT_8))
    {
        pOutData[1] = E_INVALID_KEY_TYPE;
        return;
    }
    if(COMD_UPDATE_PASSKEY == CommandId)
    {
        pOutData[1] = ProtoAnaly_AddId(pInData[1],&pInData[2],pInData[2]);
    }
    else
    {
        pOutData[1] = ProtoAnaly_AddId(pInData[1],&pInData[3],pInData[2]);
    }
#ifdef ENABLE_CONSOLE_LOG
    printf("Key type: 0x%x\n\rNum entries: %u\n\r", pInData[1], pInData[2]);
#endif // ENABLE_CONSOLE_LOG
    pOutData[2]=(pInData[3+(pInData[2]<<3)]);
    pOutData[3]=(pInData[4+(pInData[2]<<3)]);

    // Time is correct on hardware, so existing keys can be checked for expiry
    //
    if(E_OK == pOutData[1])
    {
        // Check if number of keys set reached (maximum value - 1)
        //
        if(LockKeyStatistics.ServKeyCount >= ((SERVE_KEY_PAGEMAX * SERVE_KEY_MAX) - 1))
        {
            // Set flag to check and remove expired keys
            //
            checkKeyForExpiredKeys = true;
        }
    }
    // LOCK-240 - Disable beep sound
    //
    //if(pOutData[1] == E_OK)
    //{
    //    Access_BeepStart(BEEP_NORMAL,1);
    //}
}
/****************************************************************************************************
**Function:
    void ProtoAnaly_RomveIdin(uint8_t *pInData ,uint8_t *pOutData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_RomveIdin(uint8 *pInData ,uint8 *pOutData)
{
    pOutData[0] = 4;            /* 长度*/
    pOutData += 7;    
    pInData +=4;

#if (defined DEBUG_ENABLE) && (DEBUG_ENABLE == STD_TRUE)
    Sys_Debug(STD_OFF);
#endif
#ifdef ENABLE_CONSOLE_LOG
    printf("Key Type: 0x%x\n\r", pInData[1]);
#endif // ENABLE_CONSOLE_LOG
    pOutData[1] = ProtoAnaly_RemoveId(pInData[1],&pInData[2]);
#if (defined DEBUG_ENABLE) && (DEBUG_ENABLE == STD_TRUE)
    Sys_Debug(STD_ON);
#endif
    pOutData[2]=(pInData[2+(1<<3)]);
    pOutData[3]=(pInData[3+(1<<3)]);

    // LOCK-240 - Disable beep sound
    //
    //if(pOutData[1] == E_OK)
    //{
    //    Access_BeepStart(BEEP_NORMAL,1);
    //}
}

/****************************************************************************************************
**Function:
    void ProtoAnaly_SetTimeIn(uint8_t *pInData ,uint8_t *pOutData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/

void ProtoAnaly_SetParaDefIn(uint8_t *pInData ,uint8_t *pOutData)
{
    pOutData[0] = 2;    /* 长度*/
    pOutData += 7;
    pOutData[1] = E_OK;

    SetParaFlag = 1;

    Access_BeepStart(BEEP_FAST, 3);

// 等启动时自动复位所有参数 
}
/****************************************************************************************************
**Function:
    void ProtoAnaly_ReadKeyInfo(uint8_t *pInData ,uint8_t *pOutData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_ReadKeyInfo(uint8_t *pInData ,uint8_t *pOutData)
{
    pOutData[0] = 9+15;            /* 长度*/
    pOutData += 7;
    pOutData[1]  = E_OK;//LockKey_Read_Key(&pInData[5], &pOutData[2]);
    Access_BeepStart(BEEP_NORMAL,1);
}

/****************************************************************************************************
**Function:
    void ProtoAnaly_AddKeyInfo(uint8_t *pInData ,uint8_t *pOutData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_AddKeyInfo(uint8_t *pInData ,uint8_t *pOutData)
{
    pOutData[0] = 2;            /* 长度*/
    pOutData += 7;
    pOutData[1] =  E_OK;

#if (defined DEBUG_ENABLE) && (DEBUG_ENABLE == STD_TRUE)
    Sys_Debug(STD_OFF);
#endif
    if(Access_GetSupportType(pInData[5]) == E_NOT_OK)
    {
        pOutData[1] =  E_INVALID_KEY_TYPE;
        return;
    }
#ifdef ENABLE_CONSOLE_LOG
    printf("LockKeyStatistics.ServKeyCount: %u\n\r", LockKeyStatistics.ServKeyCount);
#endif
    if(LockKeyStatistics.ServKeyCount > (SERVE_KEY_PAGEMAX * SERVE_KEY_MAX))    //  SERVE_KEY_MAX
    {
        pOutData[1]= E_MAX_KEY_COUNT;
        return;
    }
    if(pInData[5] == KEY_TYPE_PASSWORD)
    {
        pOutData[1] = ProtoAnaly_AddId(KEY_TYPE_PASSWORD, &pInData[10], 1);
    }
    else if((pInData[5] == KEY_TYPE_CARD)||(pInData[5] == KEY_TYPE_IDENTITY_CARD))
    {
        pOutData[1] = LockKey_Start_LearnCardKey(&pInData[6]);
    }
    else
    {
        pOutData[1] =  E_INVALID_KEY_TYPE;
    }

    if(pOutData[1] == E_OK)
    {
        Access_BeepStart(BEEP_NORMAL,1);
    }
    else
    {
        Access_BeepStart(BEEP_FAST,2);
    }
#if (defined DEBUG_ENABLE) && (DEBUG_ENABLE == STD_TRUE)
    Sys_Debug(STD_ON);
#endif
}
/****************************************************************************************************
**Function:
    void ProtoAnaly_DeleteKeyInfo(uint8_t *pInData ,uint8_t *pOutData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_DeleteKeyInfo(uint8_t *pInData ,uint8_t *pOutData)
{
    pOutData[0] = 2;            /* 长度*/
    pOutData += 7;
#if (defined DEBUG_ENABLE) && (DEBUG_ENABLE == STD_TRUE)
    Sys_Debug(STD_OFF);
#endif
    pOutData[1] = LockKey_Remove_Key(&pInData[6], pInData[5]);
#if (defined DEBUG_ENABLE) && (DEBUG_ENABLE == STD_TRUE)
    Sys_Debug(STD_ON);
#endif
    if(pOutData[1] == E_OK)
    {
        Access_BeepStart(BEEP_NORMAL,1);
    }
    else
    {
        Access_BeepStart(BEEP_FAST,2);
    }
}
/****************************************************************************************************
**Function:
    void ProtoAnaly_SetTouchSenvedin(uint8_t *pInData ,uint8_t *pOutData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_SetTouchSenvedin(uint8_t *pInData ,uint8_t *pOutData)
{
    pOutData[0] = 2; /* 长度*/
    pOutData += 7;
    pOutData[1] = E_OK;
    Sys_PataCfg.touchSensitive = pInData[5];
}
/****************************************************************************************************
**Function:
    void ProtoAnaly_ConfigHostIn(uint8_t *pInData ,uint8_t *pOutData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_RecordAckIn(uint8_t *pInData ,uint8_t *pOutData)
{
    pOutData[0] = 0;             /* ????*/
}

/****************************************************************************************************
**Function:
    uint16 LogsCounterIndex(void)
**Author: sheva
**Description:  Detection storage position of the activity logs in the flash
               
**Input: 
**Output: If the activity logs in flash,return position  
          If the activity logs not in flash,return (0xffaa)  /65450 
****************************************************************************************************/
uint16 LogsCounterIndex(void)
{
	uint8 i = 0,j = 0;
    for ( j=0;j<ACCESS_PAGE_MAX;j++ )
    {
        RecordList_LoadPara( j );
        for ( i=0; i<ACCESS_RECORD_MAX; i++)
          {
            if ( AccRcord.RecordList[i].Id != 0xffffffff )
              {
                 return(j*ACCESS_PAGE_MAX+i);
              }
          }
    }	
	return(0xffaa);
}	

/****************************************************************************************************
**Function:
   void FlashPageCheck(void)
**Author: sheva
**Description: Detect whether the read pointer page is accumulated after the read pointer accumulation
               If the page pointer has accumulation, read out in advance into the structure.
**Input: 
**Output: 
****************************************************************************************************/

void FlashPageCheck(void)
{
    if(PAccRcord.Srecordindex.sectorIndex >= (ACCESS_RECORD_MAX))
    {
        PAccRcord.Srecordindex.sectorIndex = 0;
        PAccRcord.Srecordindex.pageIndex++;
        RecordList_LoadPara(PAccRcord.Srecordindex.pageIndex);
    }
    if(PAccRcord.Srecordindex.pageIndex >= (ACCESS_PAGE_MAX))
    {
        PAccRcord.Srecordindex.sectorIndex = 0;
        PAccRcord.Srecordindex.pageIndex = 0;
        RecordList_LoadPara(PAccRcord.Srecordindex.pageIndex);
    }
}

/****************************************************************************************************
**Function:
    void ProtoAnaly_LoadRecordIn(uint8_t *pInData ,uint8_t *pOutData)
**Author: sheva
**Description:  Read activity logs 
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_LoadRecordIn(uint8_t *pInData ,uint8_t *pOutData)
{
    uint16 index = 0;
    uint16 Preadbak = 0, Pread = 0;

    uint8 pageIndex = 0;
    uint8 sectorIndex = 0;
    uint16 Rcordcount = 0;
    uint8 flag = 0;
	
    pOutData[0] = 7 + ACCESS_RECORD_ONCE_MAX;
    pOutData += 7;
    pOutData[1] = E_OK;
    index = BUILD_UINT16(pInData[6], pInData[5]);
    pageIndex = index / ACCESS_RECORD_MAX;
    sectorIndex = index % ACCESS_RECORD_MAX;

#ifdef ENABLE_CONSOLE_LOG
    printf("GETindex: %d\n\r", index);
#endif // ENABLE_CONSOLE_LOG

    RecordIndex_LoadPara( );
    RecordList_LoadPara(PAccRcord.Srecordindex.pageIndex);
	
	if ( 0 == index || 0xffffffff == AccRcord.RecordList[PAccRcord.Srecordindex.sectorIndex].Id )
	{
        Rcordcount = LogsCounterIndex();
        if( 0xffaa != Rcordcount )
        {
            index = Rcordcount;
            pageIndex = Rcordcount / ACCESS_RECORD_MAX;
            sectorIndex = Rcordcount % ACCESS_RECORD_MAX;
            PAccRcord.Srecordindex.pageIndex = pageIndex;
            PAccRcord.Srecordindex.sectorIndex = sectorIndex;
            RecordList_LoadPara(PAccRcord.Srecordindex.pageIndex);	
        }
        else
        {// No logs   
            pOutData[2] = 0;
            pOutData[3] = 0;
            pOutData[4] = pOutData[2];
            pOutData[5] = pOutData[3];
            memset(&pOutData[6], 0xff, 14);
            Access_Record_Null = 1;
            Access_Record_Overflow = 0;
            PAccRcord.Wrecordindex.pageIndex = 0;
            PAccRcord.Wrecordindex.sectorIndex = 0;
            PAccRcord.Srecordindex.pageIndex = 0;
            PAccRcord.Srecordindex.sectorIndex = 0;
            RecordIndex_StorePara( );
#ifdef ENABLE_CONSOLE_LOG
    printf("Return1.\r\n");
#endif // ENABLE_CONSOLE_LOG			
            return;	  	  
        }
	}


#ifdef ENABLE_CONSOLE_LOG
    printf("p: %u::s: %u::Total: %u::%u::0x%lx\n\r", PAccRcord.Srecordindex.pageIndex, PAccRcord.Srecordindex.sectorIndex,
    (sectorIndex + (ACCESS_RECORD_MAX * (pageIndex))), PAccRcord.recordnum, AccRcord.RecordList[PAccRcord.Srecordindex.sectorIndex].Id);
#endif // ENABLE_CONSOLE_LOG

    FlashPageCheck( );

#ifdef ENABLE_CONSOLE_LOG
    printf("pageIndex: %d::sectorIndex: %d\n\r", pageIndex, sectorIndex);
    printf("PpageIndex: %d::PsectorIndex: %d\n\r", sectorIndex, PAccRcord.Srecordindex.sectorIndex);	
#endif // ENABLE_CONSOLE_LOG
  
    if(index != 0)
    {
        if(sectorIndex == (PAccRcord.Srecordindex.sectorIndex + 1)
           ||( sectorIndex == 0 && ACCESS_RECORD_MAX == ((PAccRcord.Srecordindex.sectorIndex + 1))))
        {
            AccRcord.RecordList[PAccRcord.Srecordindex.sectorIndex].Id = 0xffffffff;
            RecordList_StorePara(PAccRcord.Srecordindex.pageIndex);
			
            PAccRcord.Srecordindex.sectorIndex++;
            FlashPageCheck( );
        }
    }

//  The following code is to locate the read pointer with activity logs;
//  Read the marked "0xffffffff",Polling the entire storage area of activity logs.

    Preadbak = PAccRcord.Srecordindex.sectorIndex + PAccRcord.Srecordindex.pageIndex * ACCESS_RECORD_MAX;
    flag = 1;
    while(1)
    {
        if(AccRcord.RecordList[PAccRcord.Srecordindex.sectorIndex].Id != 0xffffffff)
        {
            break;
        }

        Pread = PAccRcord.Srecordindex.sectorIndex + PAccRcord.Srecordindex.pageIndex * ACCESS_RECORD_MAX;
        if ( 1 == flag )
        {
            flag = 0;
            PAccRcord.Srecordindex.sectorIndex++;
            FlashPageCheck( );	
            continue;
        }

        if( Preadbak != Pread )    
        {
            PAccRcord.Srecordindex.sectorIndex++;
            FlashPageCheck( );
            Sys_wdt_feed();
        }
        else
        {  /* No logs */ 
#ifdef ENABLE_CONSOLE_LOG
                printf("Preadbak333: %d::Pread333: %d\n\r", Preadbak, Pread); 
#endif // ENABLE_CONSOLE_LOG				
	            pOutData[2] = 0;
	            pOutData[3] = 0;
	            pOutData[4] = pOutData[2];
	            pOutData[5] = pOutData[3];
	            memset(&pOutData[6], 0xff, 14);
	            Access_Record_Null = 1;
                Access_Record_Overflow = 0;
	            PAccRcord.Wrecordindex.pageIndex = 0;
	            PAccRcord.Wrecordindex.sectorIndex = 0;
	            PAccRcord.Srecordindex.pageIndex = 0;
	            PAccRcord.Srecordindex.sectorIndex = 0;
	            RecordIndex_StorePara( );
#ifdef ENABLE_CONSOLE_LOG
                printf("Return2.\r\n");
#endif // ENABLE_CONSOLE_LOG
                return;
        }
    }
	RecordIndex_StorePara( );

//  The following code is to locate through the read pointer, read the log and package the data

    Access_GetRecord(PAccRcord.Srecordindex.sectorIndex, &pOutData[6]);
    index = PAccRcord.Srecordindex.sectorIndex + PAccRcord.Srecordindex.pageIndex * ACCESS_RECORD_MAX;

#ifdef ENABLE_CONSOLE_LOG
	printf("PUTTindex: %d\n\r", index);
#endif // ENABLE_CONSOLE_LOG

    pOutData[2] = (index >> 8);
    pOutData[3] = (index & 0xff);
    pOutData[4] = (PAccRcord.recordnum >> 8);
    pOutData[5] = (PAccRcord.recordnum & 0xff);
    return;
}

/****************************************************************************************************
**Function:
    void ProtoAnaly_RemoveRecordIn(uint8_t *pInData ,uint8_t *pOutData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_RemoveRecordIn(uint8_t *pInData ,uint8_t *pOutData)
{
    pOutData[0] = 2;             /* 长度*/
    pOutData += 7;
    pOutData[1] = E_OK;
    //Access_EraseRecordData();
    //Access_BeepStart(BEEP_NORMAL,1);
}
/****************************************************************************************************
**Function:
    void ProtoAnaly_GetRfParaIn(uint8_t *pInData ,uint8_t *pOutData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_ReadVerSionIn(uint8_t *pInData ,uint8_t *pOutData)
{
    pOutData[0] = 2+5+4;    /* 长度*/
    pOutData += 7;
    pOutData[1] = E_OK;
    pOutData[2] = 0;
    pOutData[3] = VER_MAJOR;
    pOutData[4] = VER_MAJOR;
    pOutData[5] = VER_MINOR;
    pOutData[6] = DATE_YEAR;
    pOutData[7] = DATE_MONTH;
    pOutData[8] = DATE_DAY;
    pOutData[9] = CUSTOM_CODE;
}        

/****************************************************************************************************
**Function:
    void ProtoAnaly_SetPropertyBuild(uint8_t *pInData ,uint8_t *pOutData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_FactoryTest(uint8_t *pInData ,uint8_t *pOutData)
{
        pOutData[0] = 2; /* 长度*/
        pOutData += 7;
        
        pOutData[1] = E_OK;

        switch(pInData[5])
        {
            case COMD_FATORY_ENTERN_SLEEP:
            {
                /*BLE_UPDATA_DISCONNECT_TIMER(2000/64);
                Access_CardLearnTimer64 = 0;
                Access_BeepStart(BEEP_FAST,1);*/
                Factory_State = COMD_FATORY_ENTERN_SLEEP;
                Factory_Sub = CASE_IN;
                Access_BeepStart(BEEP_FAST,1);
                //BLE_UPDATA_DISCONNECT_TIMER(2000);
                break;
            }
            case COMD_FATORY_PARA_INIT:
            {
                Sys_ParainitFirst();
                Sys_StorePara();
                Sys_McuRestFlag=STD_TRUE;
                break;
            }
            default: 
            {
                pOutData[1] = E_KEY_NO_SUPPORT;
                break;
            }
        }
}

void ProtoAnaly_AESkeySet(uint8_t *pInData,  uint8_t *pOutData)
{
    uint8 i=0;

    pOutData[0] = 3; /* 长度*/

    pOutData += 7;
    pInData  += 5;

    for(i = 0; i < 16; i++)
    {
        Aeskeytemp[i] = pInData[i];
    }

    AesKeySetFlag = 1;
    pOutData[1] = E_OK;
    pOutData[2]= Access_BatteryData;

    advertising_init();
    Sys_PataCfg.FactoryTimeout = 600;

    Beep_once();
}


void ProtoAnaly_ConfigReset(uint8_t *pInData, uint8_t *pOutData)
{
    uint8_t status = E_OK;

    pOutData[0] = 3; /* 长度*/
    pOutData += 7;

    if(RESET_ENABLE == pInData[5])
    {
       Sys_PataCfg.resetButtonEnable = true;
    }
    else if(RESET_DISABLE == pInData[5])
    {
       Sys_PataCfg.resetButtonEnable = false;
    }
    else
    {
        status = E_INVALID_RESET_CONFIG;
    }

    pOutData[1] = status;
    pOutData[2] = Access_BatteryData;

    if(E_OK == status)
    {
        Sys_StoreFlag = STD_TRUE;
    }
}

static void ProtoAnaly_SetMode(uint8_t *pInData, uint8_t *pOutData)
{
    // Assign length of data to be responded
    // This includes status byte as well
    //
    pOutData[0] = 2; /* 3¤?è*/

    // Shift pointer by 7
    // Reserved for 4 bytes for MAC address
    //              2 bytes for Custom ID
    //          and 1 byte for command byte
    //
    pOutData += 7;

    // Fill status byte as success
    //
    pOutData[1] = E_OK;

    // Shift input data pointer by 4 bytes to point to valid data field
    //
    pInData += 4;

    if(pInData[1] == CONFIG_VOICE)
    {
        if(pInData[2] == SET_MODE)
        {
            CLR_BIT(Sys_PataCfg.configMode, VOICEMODE);
        }
        else if(pInData[2] == CLEAR_MODE)
        {
            SET_BIT(Sys_PataCfg.configMode, VOICEMODE);
        }
        else
        {
            // Fill status byte as failure
            //
            pOutData[1] = E_INVALID_MODE;
        }
    }
    else
    {
        // Fill status byte as failure
        //
        pOutData[1] = E_INVALID_MODE;
    }

    Sys_StoreFlag = STD_TRUE;
}

static void ProtoAnaly_GetMode(uint8_t *pInData, uint8_t *pOutData)
{
    // Assign length of data to be responded
    // This includes status byte as well
    //
    pOutData[0] = 4; /* 3¤?è*/

    // Shift pointer by 7
    // Reserved for 4 bytes for MAC address
    //              2 bytes for Custom ID
    //          and 1 byte for command byte
    //
    pOutData += 7;

    // Fill status byte as success
    //
    pOutData[1] = E_OK;

    // Shift input data pointer by 4 bytes to point to valid data field
    //
    pInData += 4;

    if(pInData[1] == CONFIG_VOICE)
    {
        // Fill data with Voice Mode
        //
        pOutData[2] = CONFIG_VOICE;
        if(IS_SET(Sys_PataCfg.configMode, VOICEMODE))
        {
            // Voice mode is disabled
            //
            pOutData[3] = CLEAR_MODE;
        }
        else
        {
            // Voice mode is enabled
            //
            pOutData[3] = SET_MODE;
        }
    }
    else
    {
        // Fill status byte as failure
        //
        pOutData[1] = E_INVALID_MODE;
    }
}

static _Bool isTimeRangeValid(uint32_t start, uint32_t end)
{
    _Bool status = true;

    if(start >= end)
    {
        status = false;
    }

    return status;
}

static void ProtoAnaly_PassageMode(uint8 *pInData, uint8 *pOutData)
{
    uint32_t endTime = 0;
    uint32_t startTime = 0;
    uint8_t status = E_OK;
    uint8_t storeFlag = STD_TRUE;

    // Length of response data to be filled
    //        1 byte for Status of operation
    //        1 byte for Checksum
    //
    pOutData[0] = 2;

    // Shift pointer by 7 characters
    //       4 bytes for MAC
    //       2 bytes for Custom ID
    //       1 byte for Command number
    //
    pOutData +=7;

    // Shift input data pointer by 4 bytes to point to valid data field
    //
    pInData += 4;

    // If passage mode is recevied as True,
    //
    if(true == pInData[1])
    {
        // read time values
        //
        endTime   = BUILD_UINT32(pInData[5], pInData[4], pInData[3], pInData[2]);
        startTime = BUILD_UINT32(pInData[9], pInData[8], pInData[7], pInData[6]);
        // Check validity of time values
        //
        if(isTimeRangeValid(startTime, endTime))
        {
#ifdef ENABLE_CONSOLE_LOG
            printf("Passage mode is disabled\n\r");
#endif // ENABLE_CONSOLE_LOG
            // Set passage mode
            //
            SET_BIT(Sys_PataCfg.configMode, PASSMODE);
            // Store time values
            //
            Sys_PataCfg.passageStartTime = startTime;
            Sys_PataCfg.passageEndTime = endTime;
            // Load priodicity value from the input data
            //
            Sys_PataCfg.weeklyPeriodicity = pInData[10];
        }
        else
        {
            status = E_INVALID_TIME_RANGE;
            storeFlag = STD_FALSE;
        }
    }
    else
    {
#ifdef ENABLE_CONSOLE_LOG
        printf("Passage mode is disabled\n\r");
#endif // ENABLE_CONSOLE_LOG
        // Disable passage mode
        //
        CLR_BIT(Sys_PataCfg.configMode, PASSMODE);
        // Clear time values set previously
        //
        Sys_PataCfg.passageStartTime = 0;
        Sys_PataCfg.passageEndTime = 0;
        Sys_PataCfg.weeklyPeriodicity = 0;
    }

    // Fill status byte in the output data
    //
    pOutData[1] = status;
    // Set store flag to update system configuration
    //
    Sys_StoreFlag = storeFlag;
}

void ProtoAnaly_AddCPUFob(uint8_t *pInData,  uint8_t *pOutData)
{
    uint8_t status = E_OK;

    // Length of response data to be filled
    //        1 byte for Status of operation
    //        1 byte for Checksum
    //
    pOutData[0] = 2;            /* 长度*/

    // Shift pointer by 7 characters
    //       4 bytes for MAC
    //       2 bytes for Custom ID
    //       1 byte for Command number
    //
    pOutData += 7;
	
    if(Access_GetSupportType(pInData[5]) == E_NOT_OK)
    {
        pOutData[1] = E_INVALID_KEY_TYPE;
        return;
    }
    if(LockKeyStatistics.ServKeyCount > (SERVE_KEY_PAGEMAX * SERVE_KEY_MAX))
    {
        pOutData[1]= E_MAX_KEY_COUNT;
        return;
    }

    pInData += 4;

    status = ProtoAnaly_AddId(pInData[1], &pInData[2], 1);

#ifdef ENABLE_CONSOLE_LOG
    printf("LockKeyStatistics.ServKeyCount: %u\n\r", LockKeyStatistics.ServKeyCount);
#endif

    // Fill the status byte
    //
    pOutData[1] = status;
}

void ProtoAnaly_GetPasscode(uint8_t *pInData,  uint8_t *pOutData)
{
    uint32 keyId = 0;
    uint32 uid = 0;
    uint8 status = 0;

    pOutData[0] = 17; /* 3¤?è */
    pOutData += 7;
    pInData  += 5;

    keyId = BUILD_UINT32(pInData[3], pInData[2], pInData[1], pInData[0]);

    memset(&pOutData[1], 0, 17);
    for(uint32 j = 0; j < SERVE_KEY_PAGEMAX; j++)
    {
        ServeKeyList_Readflash(j);
        for(uint32 i = 0; i < SERVE_KEY_MAX; i++)
        {
            uid = LockKeyServeKeyList.KeyList[i].KeyId;
            if(keyId == uid)
            {
#ifdef ENABLE_CONSOLE_LOG
                printf("***Match found***\n\r");
#endif // ENABLE_CONSOLE_LOG
                status = status | 0x01;
                pOutData[3] = LockKeyServeKeyList.KeyList[i].Type;
                pOutData[4] = LockKeyServeKeyList.KeyList[i].Cycle;
                C32HEX2BYTESMSB(&pOutData[5], LockKeyServeKeyList.KeyList[i].StartTime);
                C32HEX2BYTESMSB(&pOutData[9], LockKeyServeKeyList.KeyList[i].EndTime);

                j = SERVE_KEY_PAGEMAX;
                break;
            }
        }
    }

    for(uint32 i = 0; i < BLACK_LIST_MAX; i++)
    {
        uid = LockKeyBlackList.BlackList[i].KeyId;
        if(keyId == uid)
        {
            status = status | 0x02;
            pOutData[13] = LockKeyBlackList.BlackList[i].Type;
            C32HEX2BYTESMSB(&pOutData[14], LockKeyBlackList.BlackList[i].InvalidTime);
            break;
        }
    }

    // Fill the status byte
    //
    pOutData[1] = E_OK;

    pOutData[2] = status;
}

void FactortAppTest_ResponseApp(void)
{
    uint8 encry[16]={0};
    uint8 actiondata[20]={0};
    uint8 checksum=0;
    uint8 lenth=0, k=0;

    if(Sys_PataCfg.FactoryTimeout==0) {return;}  //

    if(Sys_PataCfg.FactoryTimeout)
    {
        Sys_PataCfg.FactoryTimeout--;
        if(Sys_PataCfg.FactoryTimeout==0)
        {
           Sys_StorePara();
        }
    }

    if( FactoryAppTest.testTimeout.FT_FobCheckTimeout ) //Fob check Timeout
    {
        FactoryAppTest.testTimeout.FT_FobCheckTimeout--;
        if(FactoryAppTest.testTimeout.FT_FobCheckTimeout==0)
        {
            actiondata[10] = FACTORTTEST_FOB_TIMEOUT;
            actiondata[11] = FACTORTTEST_COMMD_FOBINPUT;
            FactoryAppTest.testFlag.FT_FobFlag = 0;
            k = 1;
        }
        else
        {
            actiondata[10] = FACTORYTEST_RESULT_SUCCESS;
            actiondata[11] = FACTORTTEST_COMMD_FOBINPUT;
            if( (fob_sn[0] != 0) && (fob_sn[1] != 0) && (fob_sn[2] != 0) && (fob_sn[3] != 0) )
            {
                actiondata[12] = fob_sn[3];
                actiondata[13] = fob_sn[2];
                actiondata[14] = fob_sn[1];
                actiondata[15] = fob_sn[0];
                FactoryAppTest.testTimeout.FT_FobCheckTimeout = 0;
                FactoryAppTest.testFlag.FT_FobFlag = 0;
                fob_sn[3] = 0x00;  fob_sn[2] = 0x00; 
                fob_sn[1] = 0x00;  fob_sn[0] = 0x00;
                k = 1;
            }
        }
    }

     if( FactoryAppTest.testTimeout.FT_KeyPressTimeout ) //Key press Timeout
     {
         FactoryAppTest.testTimeout.FT_KeyPressTimeout--;
         if(FactoryAppTest.testTimeout.FT_KeyPressTimeout==0)
         {
             FactoryAppTest.testFlag.FT_KeyFlag            = 0;
             FactoryAppTest.testTimeout.FT_KeyPressTimeout = 0;
             FactoryAppTest.testFlag.FT_KeyFinished = 0;
             actiondata[10] = FACTORYTEST_TOUCH_TIMEOUT;
             actiondata[11] = FACTORYTEST_COMMD_KEYINPUT;
             k = 1;
         }
         else
         {
             if(FactoryAppTest.testFlag.FT_KeyFinished==1)
             {
                 actiondata[10] = FACTORYTEST_RESULT_SUCCESS;
                 actiondata[11] = FACTORYTEST_COMMD_KEYINPUT;

                 for(k=0; k<6; k++)
                 {
                     actiondata[12+k] = FactoryAppTest.FactoryTestKeyIndex[k];
                 }
                 FactoryAppTest.testTimeout.FT_KeyPressTimeout = 0;
                 FactoryAppTest.testFlag.FT_KeyFinished = 0;
                 FactoryAppTest.testFlag.FT_KeyFlag = 0;
                 k = 1;
             }
         }
     }

     if(FactoryAppTest.testFlag.FT_MotorFlag==1)  //Motor Action response
     {
         if( FactoryAppTest.testFlag.FT_MotorON_Finished  == 1 )         
         {
            if(FactoryAppTest.testTimeout.FT_MotorActionTimeout)
            {
                actiondata[11] = FACTORTTEST_COMMD_AUTOLOCK;
                FactoryAppTest.testFlag.FT_MotorON_Finished = 0;
                FactoryAppTest.testFlag.FT_MotorFlag = 0;
                k = 1;
            }
         }
         else
         {
            FactoryAppTest.testTimeout.FT_AulockTimeout--;
            if( FactoryAppTest.testTimeout.FT_AulockTimeout == 0 )
            {
                Access_Lock();
                FactoryAppTest.testFlag.FT_MotorON_Finished = 1; 
            } 
         }
     }

    if(k == 1)
    {
        actiondata[0] = 0x55;
        actiondata[1] = 0xAA;

        actiondata[2] = 0x10;
        actiondata[3] = 0xBB; // cmd

        actiondata[4] = (unsigned char)((Sys_PataCfg.SysId&0xFF00)>>8);
        actiondata[5] = (unsigned char)((Sys_PataCfg.SysId&0x00FF)>>0);

        actiondata[6] = (unsigned char)((Sys_PataCfg.Mac&0xFF000000)>>24);
        actiondata[7] = (unsigned char)((Sys_PataCfg.Mac&0x00FF0000)>>16);
        actiondata[8] = (unsigned char)((Sys_PataCfg.Mac&0x0000FF00)>>8);
        actiondata[9] = (unsigned char)((Sys_PataCfg.Mac&0x000000FF)>>0);

        checksum = Sys_GenFunChecksum(&actiondata[2]);
        actiondata[actiondata[2] + 2] = checksum;
        lenth = actiondata[2] + 3;

        for(checksum = 0; (checksum < ((lenth-3)>>4)); checksum++)
        {
            AES_Encrypt(&actiondata[3+(checksum<<4)],encry);
            memcpy(&actiondata[3+(checksum<<4)],encry,16);
        }
        Proto_NetCommSend(NRF_BLE, actiondata,lenth);
   }
}

void ProtoAnaly_FactoryAppTest(uint8_t *pInData,  uint8_t *pOutData)
{
        unsigned char value=0x00, i=0;
        pOutData[0] = 2; 
        pOutData   += 7;
        pInData += 5;

        if( pInData[0] == FACTORYTEST_COMMD_DISBALED )
        {
             value = FACTORYTEST_RESULT_SUCCESS;
             pOutData[2] = FACTORYTEST_COMMD_DISBALED;
             FactoryAppTest.testFlag.FT_FobFlag=0;
             FactoryAppTest.testFlag.FT_MotorFlag=0;
             FactoryAppTest.testFlag.FT_KeyFlag=0;
             BLE_UPDATA_DISCONNECT_TIMER(7);
             Sys_PataCfg.FactoryTimeout = 0;
             Sys_StorePara();
        }
        else if( pInData[0] == FACTORYTEST_COMMD_KEYINPUT ) // read key
        {
            if(Sys_PataCfg.FactoryTimeout)
            {
               FactoryAppTest.testFlag.FT_KeyFlag=1;
               FactoryAppTest.testTimeout.FT_KeyPressTimeout = FACTORYTEST_BLE_CONNECT_TIMEOUT;
               for(i=0; i<6; i++)
               {
                  FactoryAppTest.FactoryTestKeyIndex[i]=0xFF;
               }
            }
            else
            {
                value = FACTORYTEST_TEST_MODE_DISBALED;
            }
            pOutData[2] = FACTORYTEST_COMMD_KEYINPUT;
        }
        else if(pInData[0] == FACTORTTEST_COMMD_FOBINPUT)  // read fob
        {
             fob_sn[3] = 0x00;  fob_sn[2] = 0x00; 
             fob_sn[1] = 0x00;  fob_sn[0] = 0x00;
             if(Sys_PataCfg.FactoryTimeout)
             {
                FactoryAppTest.testFlag.FT_FobFlag=1;
                FactoryAppTest.testTimeout.FT_FobCheckTimeout = FACTORYTEST_BLE_CONNECT_TIMEOUT;
                pOutData[2] = FACTORTTEST_COMMD_FOBINPUT;
             }
             else 
             {
                value = FACTORYTEST_TEST_MODE_DISBALED;
             }
        }
        else if(pInData[0] == FACTORTTEST_COMMD_AUTOLOCK) //motor action
        {
           pOutData[2] = FACTORTTEST_COMMD_AUTOLOCK;
            if(Sys_PataCfg.FactoryTimeout)
            {
                FactoryAppTest.testFlag.FT_MotorFlag=1;
                FactoryAppTest.testTimeout.FT_MotorActionTimeout = FACTORYTEST_BLE_CONNECT_TIMEOUT;
                FactoryAppTest.testTimeout.FT_AulockTimeout =  FACTORYTEST_AUTLOCK_TIMEOUT;
                pOutData[2] = FACTORTTEST_COMMD_AUTOLOCK;
                Access_Unlock();
            } 
            else 
            {
                 value = FACTORYTEST_TEST_MODE_DISBALED;
            }
        }
        else
        {
            pOutData[2] = pInData[0];
            value = FACTORYTEST_RESULT_SUCCESS;
        }
        pOutData[1] = value;
}

const ProtoAnaly_ComdUpType ProtoAnalyAppTable[]=
{
    //COMD_MODIFY_PWSD, ProtoAnaly_ModifyPaswdIn,
    COMD_LOCK_UP,ProtoAnaly_PowerOnIn,
    COMD_LOCK_DOWN,ProtoAnaly_PowerOffIn,
    
    COMD_SET_TIME_APP,ProtoAnaly_SetTimeIn,
    COMD_GET_TIME_APP,ProtoAnaly_GetTimeIn,
    //COMD_UPLOAD_RECORD,ProtoAnaly_RecordAckIn,
#if(SUPPORT_RECORD_LOC_STORE == STD_TRUE)
    COMD_LOAD_RECORD,ProtoAnaly_LoadRecordIn,
#endif
   // CMD_O2O_READ_OTHER_KEY, ProtoAnaly_ReadKeyInfo,  // 增加密码/卡片23
   // CMD_O2O_ADD_OTHER_KEY, ProtoAnaly_AddKeyInfo,
   // CMD_O2O_DELETE_OTHER_KEY, ProtoAnaly_DeleteKeyInfo,
    COMD_SET_LIST,ProtoAnaly_SetListIn,
    COMD_REMOVE_ID,ProtoAnaly_RomveIdin,
   // COMD_O2O_SET_TOUCH_SEN,ProtoAnaly_SetTouchSenvedin,
    COMD_READ_CONFIG_NEW, ProtoAnaly_ReadDeviceInfoExtIn,
    COMD_SET_CONFIG_NEW, ProtoAnaly_ConfigDeviceExtIn,
   // CMD_O2O_READ_VERSION,ProtoAnaly_ReadVerSionIn,
   // COMD_FACTORY_TEST,ProtoAnaly_FactoryTest,
    COMD_AESKEY_SET, ProtoAnaly_AESkeySet,
    COMD_MODE_SET,         ProtoAnaly_SetMode,     // 0xA6
    COMD_MODE_GET,         ProtoAnaly_GetMode,     // 0xA7
    COMD_PASSAGE_MODE,     ProtoAnaly_PassageMode, // 0xA8
    COMD_ADD_CPUFOB,       ProtoAnaly_AddCPUFob,   // 0xB5
    COMD_DFU_ENABLE,     ProtoAnaly_DFUEnable,
    COMD_UPDATE_PASSKEY, ProtoAnaly_SetListIn,
    COMD_CONFIG_RESET,   ProtoAnaly_ConfigReset,
    COMD_GET_PASSCODE,   ProtoAnaly_GetPasscode, // 0xBA
    COMD_GET_FACTORY_TEST,  ProtoAnaly_FactoryAppTest,    
};
#define PROTOANALY_APPCOMMAND_NUM (sizeof(ProtoAnalyAppTable) / sizeof(ProtoAnalyAppTable[0]))

//mac层处理列表
const ProtoAnaly_ComdUpType ProtoAnalyMacTable[]=
{
    COMD_SET_PARA_DEF,ProtoAnaly_SetParaDefIn,
};
#define PROTOANALY_MACCOMMAND_NUM (sizeof(ProtoAnalyMacTable) / sizeof(ProtoAnalyMacTable[0]))
/****************************************************************************************************
**Function:
Std_ReturnType ProtoAnaly_CmdAppProc(Proto_NetcommType netype,uint8 *pindat,uint8 *poutdat)
**Author: lxw
**Description:   0 透传  1    命令被处理 3 命令无效 交由下一级处理
**Input: 
**Output: 
****************************************************************************************************/
uint8    ProtoAnaly_CmdMacProc(Proto_NetcommType netype,uint8 *pindat,uint8 *poutdat)
{
     uint8 i;
     for(i= 0;i< PROTOANALY_MACCOMMAND_NUM;i++)
     {
         if(ProtoAnalyMacTable[i].ComdId == pindat[1])
         {//是否支持mac 处理
             pindat += 6;       
             if(&(ProtoAnalyMacTable[i].comdinmanage)== NULL)
             {
                 break;
             }             
             ProtoAnalyMacTable[i].comdinmanage(&pindat[1],poutdat);
             poutdat[1] = ProtoAnalyMacTable[i].ComdId;
             return 1;
         }
     }
     return 2;
}
 /****************************************************************************************************
 **Function:
      Std_ReturnType ProtoAnaly_CmdAppProc(Proto_NetcommType netype,uint8 *pindat,uint8 *poutdat)
 **Author: lxw
 **Description:  
 **Input: 
 **Output: 
 ****************************************************************************************************/
Std_ReturnType ProtoAnaly_CmdAppProc(Proto_NetcommType netype,uint8 *pindat,uint8 *poutdat)
{
    uint8 i;
    for(i= 0;i< PROTOANALY_APPCOMMAND_NUM;i++)
    {
        if(ProtoAnalyAppTable[i].ComdId == pindat[1])
        {
            CommandId = pindat[1];
            pindat += 6;     
            if(&(ProtoAnalyAppTable[i].comdinmanage)== NULL)
            {
                break;
            }
            ProtoAnalyAppTable[i].comdinmanage(&pindat[1],poutdat);
            poutdat[1] = ProtoAnalyAppTable[i].ComdId;
            return E_OK;
        }
    }
    return E_NOT_OK;
}
/****************************************************************************************************
**Function:
   Std_ReturnType ProtoAnaly_ProcEnd(Proto_NetcommType netype,uint8 cmd)
**Author: lxw
**Description:  
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_ProcEnd(Proto_NetcommType netype,uint8 cmd)
{
    if((cmd == COMD_SET_REST)||(cmd == COMD_SET_PARA_DEF)||(cmd == COMD_SET_DECRP_WORD)
    ||(cmd == COMD_CONFIG_HOST)||(cmd == COMD_SET_DECRP_WORD) || (cmd == COMD_MODIFY_CONFIG)
    ||(cmd == COMD_O2O_SET_TOUCH_SEN)  ||(cmd == COMD_SET_LIST)
    ||(cmd == COMD_REMOVE_ID))
    {
        Sys_StoreFlag=1;
    }
    if((cmd == COMD_SET_REST)||(cmd == COMD_SET_PARA_DEF) ||(cmd == COMD_SET_DECRP_WORD)
    ||(cmd == COMD_MODIFY_CONFIG) ||(cmd == COMD_O2O_SET_TOUCH_SEN) ||(cmd == COMD_SET_RF_PARA))
    {
        Sys_McuRestFlag=1;
    }
    Proto_NetcommBack = netype;
}
/****************************************************************************************************
**Function:
   void ProtoAnaly_AddRecordCmd(uint8* pKeyId,uint32 time,uint8 type, uint8 action)
**Author: lxw
**Description:  
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_AddRecordCmd(uint8* pKeyId,uint32 time,uint8 type, uint8 action)
{
    if(ProtoAnalyRecordIndex>= PROTOANALY_RECORD_CACHEMAX)
    {
        return ;
    }
    ProtoAnalyRecordBuf[ProtoAnalyRecordIndex][0]= type;
    if((type == KEY_TYPE_IDENTITY_CARD) || (type == ACCESS_LEARN_IDENTITY_CARD_TPYE))
    {
        memcpy(&ProtoAnalyRecordBuf[ProtoAnalyRecordIndex][1],pKeyId,8);
        ProtoAnalyRecordBuf[ProtoAnalyRecordIndex][9] = action;
        Sys_GenFun32To8(time,&ProtoAnalyRecordBuf[ProtoAnalyRecordIndex][10]);
        ProtoAnalyRecordBuf[ProtoAnalyRecordIndex][14] = Access_BatteryData;
    }
    else 
    {
        memcpy(&ProtoAnalyRecordBuf[ProtoAnalyRecordIndex][1],pKeyId,4);
        ProtoAnalyRecordBuf[ProtoAnalyRecordIndex][5] = action;
        Sys_GenFun32To8(time,&ProtoAnalyRecordBuf[ProtoAnalyRecordIndex][6]);
        ProtoAnalyRecordBuf[ProtoAnalyRecordIndex][10] = Access_BatteryData;
    }
    ProtoAnalyRecordIndex++;
}
/****************************************************************************************************
**Function:
   void ProtoAnaly_LoraSlaveCallBackProc(uint32 id)
**Author: lxw
**Description:  
**Input: 
**Output: 
****************************************************************************************************/
uint8 ProtoAnaly_RecordGetCmd(uint8 *pdat)
{
    uint8 lenth;

    if(ProtoAnalyRecordIndex== 0)
    {
        return 0;
    }
    else 
    {
        ++ProtoAnalyRecordSendErr;
        if(ProtoAnalyRecordSendErr >3)
        {
            ProtoAnaly_RecordRemoveCmd();
            ProtoAnalyRecordSendErr = 0;
            if(ProtoAnalyRecordIndex == 0)
            {
                return 0;
            }
        }
        Sys_GenFun32To8(Sys_PataCfg.Mac,&pdat[0]);
        pdat[4]= (COMD_LORA_RECORE);
        if((ProtoAnalyRecordBuf[0][0] == KEY_TYPE_IDENTITY_CARD) || (ProtoAnalyRecordBuf[0][0] == ACCESS_LEARN_IDENTITY_CARD_TPYE))
        {
            memcpy(&pdat[5],&ProtoAnalyRecordBuf[0][0],10+4);
            lenth = (10+3+4+4);
            pdat[19]= Access_BatteryData;
            pdat[20]= Access_LockStatus; /* 门状态  1:开 0:关*/
            pdat[21]=Sys_GenFunLChecksum(lenth,pdat);
            lenth++;
            ProtoAnaly_LoraEncrypt(lenth,pdat);
            return lenth;
        }
        else 
        {
            memcpy(&pdat[5],&ProtoAnalyRecordBuf[0][0],6+4);
            lenth = (6+3+4+4);
            pdat[15]= Access_BatteryData;
            pdat[16]= Access_LockStatus; /* 门状态  1:开 0:关*/
            pdat[17]=Sys_GenFunLChecksum(lenth,pdat);
            lenth++;
            ProtoAnaly_LoraEncrypt(lenth,pdat);
            return lenth;
        }
    }
}
/****************************************************************************************************
**Function:
   void ProtoAnaly_LoraSlaveRemoveCmd(void)
**Author: lxw
**Description:  
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_RecordRemoveCmd(void)
{
    uint8 i;
    if(ProtoAnalyRecordIndex == 0)
    {
        return ;
    }
    for(i= 0;i< ProtoAnalyRecordIndex;i++)
    {
        memcpy(ProtoAnalyRecordBuf[i],ProtoAnalyRecordBuf[i+1],12);
    }
    ProtoAnalyRecordIndex--;
    ProtoAnalyRecordSendErr=0;
}
/****************************************************************************************************
**Function:
    void ProtoAnaly_ModifyCustomInfo(uint8_t *pInData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_ModifyCustomInfo(uint8_t *pInData)
{
    uint8 checksum;
    uint8 i,length;
    uint8 temp_buf[8];

    memcpy(temp_buf, &pInData[PROTO_COMM_COMD_CMDADD], 8);
    Encpt_DecrpytExt(temp_buf);
    memcpy(&pInData[PROTO_COMM_COMD_CMDADD], temp_buf, 8);
    checksum = 0;
    length = 28;
    for(i=0; length > 0; length--)
    {
        checksum ^= pInData[PROTO_COMM_COMD_LENTHADD+i];
        i++;
    }
    if(checksum != pInData[30])
    {
        Access_BeepStart(BEEP_FAST,1);
        return;
    }
    if(pInData[PROTO_COMM_COMD_CMDADD] != COMD_MODIFY_CUSTOM_INFO)
    {
        Access_BeepStart(BEEP_FAST,1);
        return;
    }
    if(Sys_PataCfg.Mac != Sys_GenFun8To32(&pInData[6]))
    {
        Access_BeepStart(BEEP_FAST,1);
        return;
    }
    Sys_PataCfg.SysId = BUILD_UINT16(pInData[5], pInData[4]);
    memcpy(Sys_PataCfg.Aeskey, &pInData[14], 16);
    Sys_StorePara();
    Access_BeepStart(BEEP_OPEN,1);
    Sys_McuRestFlag =STD_TRUE;//复位
}
/****************************************************************************************************
**Function:
    void ProtoAnaly_HextoAsc(uint8 *pInData,uint8 *pOutData,uint8 lenth)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_HextoAsc(uint8 *pInData,uint8 *pOutData,uint8 lenth)
{
   uint8 i;
   uint8 halfbyte;
   for(i= 0;i< lenth;i++)
   {
       halfbyte= *pInData>>4;
       if(halfbyte<=9)
       {
           *pOutData = '0'+halfbyte;
       }
       else 
       {
           *pOutData = 0x37+halfbyte;
       }
       pOutData++;
       halfbyte= *pInData&0x0f;
       if(halfbyte<=9)
       {
           *pOutData = '0'+halfbyte;
       }
       else 
       {
           *pOutData = 0x37+halfbyte;
       }
        pOutData++;
        pInData++;
   }
}
/****************************************************************************************************
**Function:
    uint32 BKDRHash(uint8 *pbuf,uint8 lenth)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
uint32 BKDRHash(uint8 *pbuf,uint8 lenth)
{
    uint8 cbuf[20];
    uint32 seed = 131313; // 31 131 1313 13131 131313 etc..
    uint32 hash = 0;
    uint8 i= 0;

    memset(cbuf,0,20);
    ProtoAnaly_HextoAsc(pbuf,cbuf,lenth);
    while (cbuf[i])
    {
        hash = hash * seed + (cbuf[i++]);
    }
    return hash;
}

/****************************************************************************************************
**Function:
    void ProtoAnaly_PublicProc(uint8 *pInData,uint8 *pOutData,uint8 lenth)
**Author: rory
**Description:    
**Input: 
**Output: 
****************************************************************************************************/
Std_ReturnType ProtoAnaly_PublicProc(Proto_NetcommType netype,uint8 *pindat,uint16 lenth,uint8 *poutdat)
{
    return E_NOT_OK;
}

#if (defined LORA_ENABLE) && (LORA_ENABLE == STD_TRUE)
/****************************************************************************************************
**Function:
   void ProtoAnaly_LoraSlaveCallBackProc(void)
**Author: lxw
**Description:  
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_LoraSlaveCallBackProc(void)
{
    uint8_t datbuf[30]={0};
    datbuf[0] = 4;
    datbuf[1] = COMD_GET_HEARTBEAT;
    datbuf[8] = 0;
    datbuf[9] = TYPE_LOCK_ACCESS_REMOTE;
    datbuf[10] = 0;
    Proto_CommSend(LORA,datbuf);
}

uint8 LoraRecSnytimes;
/****************************************************************************************************
**Function:
   void ProtoAnaly_LoraSlaveTickSnyc(uint8 maxunm,uint8 index)
**Author: lxw
**Description:  
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_LoraSlaveTickSnyc(uint8 wktype,uint8 index,uint8 dereindex)
{
    int16 delaytime;
    if(wktype == 0)
    {
        LoraRcing_timer_Delay(10);
    }
    else if(wktype == 1)
    {
        if(((Sys_PataCfg.SyncIndex-1)&0x07) == ((index&0x07)))
        {
            delaytime= (((((PROTOANALY_CYCLETIMES_MAX >>Sys_PataCfg.rfPara.Speed)-(index+1))>>3)<<3)-1)*(PROTOANALY_REC_WINDOWTIMES *(1<<Sys_PataCfg.rfPara.Speed))-30;
            if((delaytime >0 )&&((index + Sys_PataCfg.SyncIndex)<(PROTOANALY_CYCLETIMES_MAX >>Sys_PataCfg.rfPara.Speed)))
            {
                LoraWakeup_timer_Start(RADIO_SET_RX,(uint16)delaytime);
            }
            else 
            {
                LoraWakeup_timer_Start(RADIO_SET_CAD_DET,(PROTOANALY_REC_WINDOWTIMES*PROTOANALY_CYCLETIMES_MAX)/2-20);
            }
        }
        else if((index + Sys_PataCfg.SyncIndex)<(PROTOANALY_CYCLETIMES_MAX >>Sys_PataCfg.rfPara.Speed))
        {
            delaytime = PROTOANALY_CYCLETIMES_ONCE_MAX*(PROTOANALY_REC_WINDOWTIMES*(1<<Sys_PataCfg.rfPara.Speed) )-(((index&0x07)+1)*(PROTOANALY_REC_WINDOWTIMES*(1<<Sys_PataCfg.rfPara.Speed) )) +(((dereindex-1) &0x07)*PROTOANALY_REC_WINDOWTIMES)-25;
            if(delaytime >0 )
            {
                LoraWakeup_timer_Start(RADIO_SET_RX,(uint16)delaytime);
            }
            else 
            {
                LoraRcing_timer_Delay(10);
            }
        }
        else 
        {
            LoraWakeup_timer_Start(RADIO_SET_CAD_DET,(PROTOANALY_REC_WINDOWTIMES*PROTOANALY_CYCLETIMES_MAX)/2-20);
        }
    }
    else if(wktype == 2)
    {
        //继续接收下个周期数据
        LoraRecSnytimes = 0;
        LoraWakeup_timer_Start(RADIO_SET_CAD_DET,PROTOANALY_REC_WINDOWTIMES*PROTOANALY_CYCLETIMES_MAX-20);
    }
    else if(wktype == 3)
    {
//        RadioDrive.ioctl(RADIO_SET_SLEEP,NULL);
    }
}
#endif
/****************************************************************************************************
**Function:
**Description:
void ProtoAnaly_LoraEncrypt(uint8 lenth,uint8 *dat)
**Input: 
**Output: 
****************************************************************************************************/
void ProtoAnaly_LoraEncrypt(uint8 lenth,uint8 *dat)
{
    uint8 i;
#if(COMMENCPT_ENABLE == STD_TRUE)    
    for(i= 0;i< (lenth>>3);i++)
    {
        Encpt_EncryptExt(&dat[(i<<3)]);
    }
#endif
}
/****************************************************************************************************
**Function:
**Description:
Std_ReturnType ProtoAnaly_LoraDecrpyt(uint8 lenth,uint8 *dat)
**Input: 
**Output: 
****************************************************************************************************/
Std_ReturnType ProtoAnaly_LoraDecrpyt(uint8 lenth,uint8 *dat)
{
    uint8 i;
#if   (COMMENCPT_ENABLE == STD_TRUE)
    for(i= 0;i< (lenth>>3);i++)
    {
        Encpt_DecrpytExt(&dat[i<<3]);
    }
#endif
        return E_OK;
}

void performRevertOperation(void)
{
    uint8_t tempCommandId = CommandId;

    CommandId = 0;

#ifdef ENABLE_CONSOLE_LOG
    printf("Reverting operation for command: 0x%x\n\r", tempCommandId);
#endif // ENABLE_CONSOLE_LOG
    if(0xFF != keyIndex &&
       0xFF != pageId)
    {
        if(COMD_REMOVE_ID        == tempCommandId ||
           COMD_SET_LIST         == tempCommandId ||
           COMD_UPDATE_PASSKEY   == tempCommandId ||
           CMD_O2O_ADD_OTHER_KEY == tempCommandId)
        {
            ServeKeyList_Readflash(pageId);

            memcpy(&LockKeyServeKeyList.KeyList[keyIndex],
                   (const LockKey_ServeKeyListType *)&keyRecord,
                   sizeof(keyRecord));
            if((COMD_SET_LIST == CommandId         ||
               CMD_O2O_ADD_OTHER_KEY == CommandId)
                                                   &&
               LockKeyStatistics.ServKeyCount > 0)
            {
                LockKeyStatistics.ServKeyCount--;
            }
            else
            {
                if(COMD_REMOVE_ID == CommandId)
                {
                    LockKeyStatistics.ServKeyCount++;
                }
            }
            ServeKeyList_Writeflash(pageId);
       }
       keyIndex = 0xFF;
       pageId = 0xFF;
   }
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

