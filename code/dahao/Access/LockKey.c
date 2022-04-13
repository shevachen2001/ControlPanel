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
#include "KEY\Touch.h"
#include "SYSTEM\Sys_TimeBase.h"
#include "Protocol\factInfo.h"
#include "Access\LockKey.h"
#include "ble_dahao.h"
#include "Access\Access.h"
#include "RTC\Rtc.h"
#include "protocol\Proto_Analy.h"
#include "Flash\Flash.h"
#include "Flash\Flash_Nrf.h"

#define C32HEX2BYTESMSB(pBuf, datas)          \
        *(pBuf + 0) = (uint8)((datas) >> 24); \
        *(pBuf + 1) = (uint8)((datas) >> 16); \
        *(pBuf + 2) = (uint8)((datas) >> 8);  \
        *(pBuf + 3) = (uint8)((datas) >> 0)

extern Std_ReturnType Access_ComTimeLoop(uint32 startTime, uint32 endTime);
#ifdef ENABLE_CONSOLE_LOG
extern void printRTCTime(uint32_t rawTime);
#endif // ENABLE_CONSOLE_LOG

extern uint8  Access_AddId_NewFlag;
extern uint8  Access_FlashProcFlag;
extern uint32 Access_UserId_Temp;
extern uint32 Access_ScanInex_Temp;
extern uint8 Access_AttTIme_Temp[];
extern uint32 ProtoAnaly_RtcLocalTime;
extern uint8_t debugPasscodeFail;

LockKey_StatisticsType   LockKeyStatistics;
LockKey_BlackListType    LockKeyBlackList;
LockKey_ServeKeyListType LockKeyServeKeyList;
CPUFob_DataType          CPUFobKey_List;
Store_AddrType           Store_Addr;

uint8 Access_Record_Null = 0;
_Bool ErrCodeBlackListFull = false;
unsigned char LogsPswTemp[4] = {0};
uint8 ServeKeyFlashOffset;
uint8 keyHexLen;
uint8 keyStoreLen;
char keyInaAscBuf[32];
uint8 keyStoreHexBuf[4];
char keyStoreAscBuf[8];

/****************************************************************************************************
**Function:
    void LockKey_RemoveKey(uint8 index)
**Author: rory
**Description: 
**Input: 
**Output: 
****************************************************************************************************/
void LockKey_RemoveKey(uint8 index)
{
    if(index == 0)
    {
        return;
    }
    index--;
    ServeKeyList_Readflash(ServeKeyFlashOffset);
    memset(&LockKeyServeKeyList.KeyList[index], 0xff, sizeof(LockKey_ServeKeyType));
    if(LockKeyStatistics.ServKeyCount > 0)
    {
        LockKeyStatistics.ServKeyCount--;
    }
    ServeKeyList_Writeflash(ServeKeyFlashOffset);
}

void  hexToAscs(uint8_t hex,char *ascs)
{
    uint8_t h,l;
    h=(hex>>4)&0x0f;
    l=(hex&0x0f);

    if(h<=9)   //lgf0503  ((h>=0)&&(h<=9))
        ascs[0]=h+0x30;
    else if((h>=10)&&(h<=15)){
        ascs[0]=h+0x41-10;
    }else{
        ascs[0]=0xff;
    }

    if(l<=9)   //lgf0503   ((l>=0)&&(l<=9))
        ascs[1]=l+0x30;
    else if((l>=10)&&(l<=15)){
        ascs[1]=l+0x41-10;
    }else{
        ascs[1]=0xff;
    }
}

uint16_t HexsToAscs(uint8_t *hexs,char * ascs,uint16_t length)
{
    uint16_t i,j=0;
    for( i=0;i<length;i++){
        hexToAscs(hexs[i],ascs+j);
        j+=2;
    }
    return j;
}

_Bool numcmp(char* str, char *substr,uint8_t lenstore,uint8_t leninput)
{
    char* s = str;
    uint8_t i = 0;
    _Bool rflag = true;

    if(lenstore > leninput)
    {
#ifdef ENABLE_CONSOLE_LOG
        printf("D1: lenth: %u::keyStoreLen: %u\n\r", leninput, lenstore);
#endif
        return false;
    }

    while (rflag)
    {
        if (memcmp(s, substr, lenstore) == 0)
        {
            rflag = false;
            return true;
        }
        else
        {
            if ( ++i <= (leninput - lenstore))
            {
                s++;
            }
            else
            {
                rflag = false;
                return false;
            }
        }
    }
    return false;
}

uint8 LockKey_AlternateCommPswdId(uint8 idtpye, uint8 *pid, uint32 *pkeyId, uint8 lenth)
{
    uint32 keyData = 0;
    uint8 keyType = 0;

    if ( lenth &0x01 )
    {
        keyHexLen = (lenth>>1)+1; // 奇数长度
    }
    else
    {
        keyHexLen = lenth>>1; // 偶数长度
    }
    HexsToAscs(pid,keyInaAscBuf,keyHexLen);

    for(uint8 k = 0; k < SERVE_KEY_PAGEMAX; k++)
    {
        ServeKeyList_Readflash(k);
        for(uint8 i = 0; i < SERVE_KEY_MAX; i++)
        {
            keyType = LockKeyServeKeyList.KeyList[i].Type;
            if((keyType == KEY_TYPE_PASSWORD) || (keyType == KEY_TYPE_TIME_SLOT_7)
                || (keyType == KEY_TYPE_TIME_SLOT_8) || (keyType == KEY_TYPE_ONE_TIME) || (keyType == KEY_TYPE_FOREVER))
            {
                ServeKeyFlashOffset = k;
                keyData = LockKeyServeKeyList.KeyList[i].KeyId;
            }
            else
            {
                continue;
            }

            if(keyData == 0xffffffff)
            {
                continue;
            }

            C32HEX2BYTESMSB(keyStoreHexBuf, keyData );
            HexsToAscs(keyStoreHexBuf,keyStoreAscBuf,4);

            if ( 'F' == keyStoreAscBuf[0] && 'F' == keyStoreAscBuf[1]
              && 'F' == keyStoreAscBuf[2] && 'F' == keyStoreAscBuf[3] )
            {
                keyStoreLen = 4;
                memcpy(keyStoreAscBuf, &keyStoreAscBuf[4], 4 );
            }
            else if ( 'F' == keyStoreAscBuf[0] && 'F' == keyStoreAscBuf[1]
                   && 'F' == keyStoreAscBuf[2] )
            {
                keyStoreLen = 5;
                memcpy(keyStoreAscBuf, &keyStoreAscBuf[3], 5 );
            }
            else if  ( 'F' == keyStoreAscBuf[0] && 'F' == keyStoreAscBuf[1] )
            {
                keyStoreLen = 6;
                memcpy(keyStoreAscBuf, &keyStoreAscBuf[2], 6 );
            }
            else if ('F' == keyStoreAscBuf[0] )
            {
                keyStoreLen = 7;
                memcpy(keyStoreAscBuf, &keyStoreAscBuf[1], 7 );
            }
            else
            {
                keyStoreLen = 8;
            }

            if ( 1 ==  numcmp(keyInaAscBuf,keyStoreAscBuf,keyStoreLen,lenth))
            {
                *pkeyId = keyData;
                return (i+1);
            }
        }
    }
    return 0;
}


/****************************************************************************************************
**Function:
    uint16 LockKey_CommPswdId(uint8 idtpye, uint8 *pid, uint32 *pkeyId,uint8 lenth)
**Author: rory
**Description: 返回id所在色索引 
**Input: 0:黑名单  1:服务下发 2:APP下发
**Output:  0 表示没找到
****************************************************************************************************/
uint16 LockKey_CommPswdId(uint8 idtpye, uint8 *pid, uint32 *pkeyId, uint8 lenth)
{
    uint32 keytemp = 0;
    uint32 rdkeytemp = 0;
    uint32 keyData = 0;
    uint8 lenthhfbyte = 0;
    uint8 currentMax = 0;
    uint8 keyType = 0;

    if(idtpye == 0)
    {
        currentMax = LockKeyStatistics.BlackListCount;
    }
    else
    {
        return 0;
    }

    if(currentMax == 0)
    {
        return 0;
    }

    lenthhfbyte = lenth;

    for(;;)
    {
LOCKKEY_COMMAGAIN:
        keytemp = 0;
        if(lenthhfbyte >= 8)
        {
            if(lenthhfbyte & 0x01)
            {
                keytemp = BUILD_UINT32(pid[(lenthhfbyte >> 1) - 1],
                                       pid[(lenthhfbyte >> 1) - 2],
                                       pid[(lenthhfbyte >> 1) - 3],
                                       pid[(lenthhfbyte >> 1) - 4]);
                keytemp <<= 4;
                keytemp |= (pid[(lenthhfbyte >> 1)] >> 4);
            }
            else 
            {
                keytemp = BUILD_UINT32(pid[(lenthhfbyte >> 1) - 1],
                                       pid[(lenthhfbyte >> 1) - 2],
                                       pid[(lenthhfbyte >> 1) - 3],
                                       pid[(lenthhfbyte >> 1) - 4]);
            }
            lenthhfbyte--;
        }
        else 
        {
            keytemp = 0;
            if(lenthhfbyte & 0x01)
            {
                for(uint8 i = 0; i < (lenthhfbyte >> 1) + 1; i++)
                {
                    keytemp <<= 8;
                    keytemp |= pid[i];
                }
                keytemp >>= 4;

                if(lenthhfbyte == 7)
                {
                    keytemp |= 0xF0000000;
                }
                if(lenthhfbyte == 5)
                {
                    keytemp |= 0xFFF00000;
                }
            }
            else 
            {
                for(uint8 i = 0; i < (lenthhfbyte >> 1); i++)
                {
                    keytemp <<= 8;
                    keytemp |= pid[i];
                }
                if(lenthhfbyte == 6)
                {
                    keytemp |= 0xFF000000;
                }
                if(lenthhfbyte == 4)
                {
                    keytemp |= 0xFFFF0000;
                }
            }
            lenthhfbyte--;
        }
        for(uint16 j= 0;j< currentMax;)
        {
            if(idtpye == 0)
            {
                for(uint8 i =0;i <BLACK_LIST_MAX;i++)
                {
                    keyType = LockKeyBlackList.BlackList[i].Type;
                    if( (keyType == KEY_TYPE_PASSWORD) || (keyType == KEY_TYPE_TIME_SLOT_7) || (keyType == KEY_TYPE_TIME_SLOT_8)
                        || (keyType == KEY_TYPE_ONE_TIME) || (keyType == KEY_TYPE_FOREVER)    || (keyType == KEY_TYPE_CLEAR))
                    {
                        keyData = LockKeyBlackList.BlackList[i].KeyId;
                        if(keyData != 0xffffffff)
                        {
                            if(BlackList_Remove(i) == E_OK)
                            {// 3é1|é?3yoú??μ￥
                                continue;
                            }
                        }
                    }
                    else
                    {
                        continue;
                    }

                    if(keyData == 0xffffffff)
                      {
                          continue;
                      }
                      j += 1;
                      rdkeytemp = keyData;
                      if(( rdkeytemp&0xf0000000)!= 0xf0000000)
                      {
                          if(keytemp == rdkeytemp)
                          {
                              *pkeyId = rdkeytemp;
                              return (i+1);
                          }
                      }
                      else if((rdkeytemp &0xff000000)!= 0xff000000)
                      {
                          if((keytemp&0x0fffffff) == (rdkeytemp&0x0fffffff))
                          {
                              *pkeyId = (rdkeytemp&0x0fffffff)|0xf0000000;
                              return (i+1);
                          }
                      }
                      else if((rdkeytemp &0xfff00000)!= 0xfff00000)
                      {
                          if((keytemp&0x00ffffff) == (rdkeytemp&0x00ffffff))
                          {
                                  *pkeyId = (rdkeytemp&0x00ffffff)|0xff000000;
                                  return (i+1);
                          }
                      }
                      else if((rdkeytemp &0xffff0000)!= 0xffff0000)
                      {
                          if((keytemp&0x000fffff) == (rdkeytemp&0x000fffff))
                          {
                              *pkeyId = (rdkeytemp&0x000fffff)|0xfff00000;
                              return (i+1);
                          }
                      }
                      else if((rdkeytemp &0xffff0000)== 0xffff0000)
                      {
                          if((keytemp&0x0000ffff) == (rdkeytemp&0x0000ffff))
                          {
                              *pkeyId = (rdkeytemp&0x0000ffff);
                              return (i+1);
                          }
                      }
                  }
            }
            if(lenthhfbyte < 4)
            {
                return 0;
            }
            else
            {
                goto LOCKKEY_COMMAGAIN;
            }
        }
    }
}

/****************************************************************************************************
**Function:
    uint8 LockKey_Check_PaswdKey(uint8 *pInData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
uint8 LockKey_Check_PaswdKey(uint8* pTouchKey,uint8 lenth )
{
    uint8 index;
    uint8 temp_buf[8];
    uint32 keyId, startTime, endTime;
    uint8 ret = E_NOT_OK, cycledat;
#if (SUPPORT_RECORD_RTIME_SEND == STD_TRUE)
    uint8 keyType;
#endif
    index =  LockKey_CommPswdId(0, pTouchKey, &keyId, lenth);
    if(index != 0)
    {
        debugPasscodeFail = 19;
        return E_TIME_OUT;
    }

    index =  LockKey_AlternateCommPswdId(1, pTouchKey, &keyId, lenth);

    if(index != 0)
    {
#ifdef ENABLE_CONSOLE_LOG
        printf("B1\n\r");
#endif // ENABLE_CONSOLE_LOG
        startTime = LockKeyServeKeyList.KeyList[index-1].StartTime;
        endTime = LockKeyServeKeyList.KeyList[index-1].EndTime;
#if (SUPPORT_RECORD_RTIME_SEND == STD_TRUE)
        keyType = LockKeyServeKeyList.KeyList[index-1].Type;
#endif
        cycledat =  LockKeyServeKeyList.KeyList[index-1].Cycle;

        LogsPswTemp[0]=(unsigned char)(((keyId)&0xFF000000)>>24);
        LogsPswTemp[1]=(unsigned char)(((keyId)&0x00FF0000)>>16);
        LogsPswTemp[2]=(unsigned char)(((keyId)&0x0000FF00)>>8);
        LogsPswTemp[3]=(unsigned char)(((keyId)&0x000000FF)>>0);
    }
    else
    {
        debugPasscodeFail = 20;
#ifdef ENABLE_CONSOLE_LOG
        printf("B2\n\r");
#endif // ENABLE_CONSOLE_LOG
        return E_NOT_OK;
    }
#if (defined RTC_EN) && (RTC_EN == STD_TRUE)
    ret = Access_ComTime(startTime, endTime);
#ifdef ENABLE_CONSOLE_LOG
    printf("Match Found: End time:");
    printRTCTime(endTime);
    printf("Local time:");
    printRTCTime(ProtoAnaly_RtcLocalTime);
#endif // ENABLE_CONSOLE_LOG
    if(ret == E_TIME_OUT)
    {
#ifdef ENABLE_CONSOLE_LOG
        printf("Time out of range\n\r");
#endif // ENABLE_CONSOLE_LOG
        LockKey_RemoveKey(index);
    }
    if((cycledat&0x80) ==0)
    {
        uint8 t = 0;

        if(ProtoAnaly_Rtcinfo.DayOfWeek == 7)
        {
            t = 0;
        }
        else
        {
            t = ProtoAnaly_Rtcinfo.DayOfWeek;
        }
        if((cycledat&(1<<t)) ==0)
        {
            debugPasscodeFail = 23;
            return E_TIME_OUT;
        }
        if(Access_ComTimeLoop(startTime, endTime) != E_OK)
        {
            debugPasscodeFail = 24;
            return E_TIME_OUT;
        }
    }
#else 
    return E_OK;
#endif
    Sys_GenFun32To8(keyId, temp_buf);
#if (SUPPORT_RECORD_RTIME_SEND == STD_TRUE)
    ProtoAnaly_AddRecordCmd(temp_buf,ProtoAnaly_RtcLocalTime,keyType,ACCESS_OPEN_LOCK_TPYE);
#endif
    return ret;
}


/****************************************************************************************************
**Function:
    uint8 LockKey_Check_CardKey(uint8 idtpye,uint8 *pid)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
uint8 LockKey_Check_CardKey(uint8 idtpye,uint8 *pid)
{
    uint32 keyId;
    uint8 ret=E_NOT_OK;

    if(idtpye == KEY_TYPE_IDENTITY_CARD)
    {
        keyId = BUILD_UINT32(pid[7],pid[6],pid[5],pid[4]);
    }
    else
    {
    keyId = BUILD_UINT32(pid[3], pid[2], pid[1], pid[0]);
    }

#ifdef ENABLE_CONSOLE_LOG
    printf("keyId: %lu::0x%lx\n\r", keyId, keyId);
#endif // ENABLE_CONSOLE_LOG
    /*  检测黑名单*/
    for(uint8 i=0; i<BLACK_LIST_MAX; i++)
    {
        if((LockKeyBlackList.BlackList[i].Type == KEY_TYPE_CARD) || (LockKeyBlackList.BlackList[i].Type == KEY_TYPE_IDENTITY_CARD))
        {
            if(LockKeyBlackList.BlackList[i].KeyId != 0xffffffff)
            {
                if(BlackList_Remove(i) == E_OK)
                {// 成功删除黑名单
                    continue;
                }
            }
        }
        else
        {
            continue;
        }
        if(idtpye == LockKeyBlackList.BlackList[i].Type)
        {
            if(LockKeyBlackList.BlackList[i].KeyId == keyId)
            {
                return E_TIME_OUT;
            }
        }
    }
    if((LockKeyStatistics.ServKeyCount == 0) && (LockKeyStatistics.AppKeyCount == 0))
    {
        return E_NOT_OK;
    }
    for (uint8 j=0;j<SERVE_KEY_PAGEMAX;j++ )
    {
        ServeKeyList_Readflash( j );    
        for(uint8 i=0; i<SERVE_KEY_MAX; i++)
        {
            if(idtpye == LockKeyServeKeyList.KeyList[i].Type)
            {
                if(LockKeyServeKeyList.KeyList[i].KeyId == keyId)
                {
                #if (defined RTC_EN) && (RTC_EN == STD_TRUE)
                    uint8 t = 0;

                    if(ProtoAnaly_Rtcinfo.DayOfWeek == 7)
                    {
                        t = 0;
                    }
                    else
                    {
                        t = ProtoAnaly_Rtcinfo.DayOfWeek;
                    }
                    ret = Access_ComTime(LockKeyServeKeyList.KeyList[i].StartTime, LockKeyServeKeyList.KeyList[i].EndTime); 
                    if(ret != E_OK) 
                    {
#ifdef ENABLE_CONSOLE_LOG
                        printf("***Access_ComTime\n\r");
#endif // ENABLE_CONSOLE_LOG
                        return E_NOT_OK;
                    }
                    else if(LockKeyServeKeyList.KeyList[i].Cycle&0x80)
                    {
                        return E_OK;
                    }
                    else if(LockKeyServeKeyList.KeyList[i].Cycle & (1<<t))
                    {
                        if(Access_ComTimeLoop(LockKeyServeKeyList.KeyList[i].StartTime, LockKeyServeKeyList.KeyList[i].EndTime) == E_OK)
                        {
                            return E_OK;
                        }
                        else
                        {
#ifdef ENABLE_CONSOLE_LOG
                            printf("***Access_ComTimeLoop\n\r");
#endif // ENABLE_CONSOLE_LOG
                            return E_NOT_OK;
                        }
                    }
                #else
                    return E_OK;
                #endif
                }
            }
        }
    }
    return E_NOT_OK;
}

/****************************************************************************************************
**Function:
    uint8 LockKey_Remove_PasswordKey(BlueKey_AllType BlueKey)
**Author: rory
**Description:
**Input: type: 0(All)  1(one)
             limitTime:限制时间
**Output: 
****************************************************************************************************/
uint8 LockKey_Remove_PasswordKey(void)
{
    uint8 offset;
    uint8 year,month,day;
    uint32 tempTime;
    for (uint8 j=0;j<SERVE_KEY_PAGEMAX;j++ )
    {
        ServeKeyList_Readflash( j );    

        for(uint8 i=0; i<SERVE_KEY_MAX; i++)
        {
            if(LockKeyServeKeyList.KeyList[i].KeyId != 0xffffffff)
            {
                if((LockKeyServeKeyList.KeyList[i].Type == KEY_TYPE_TIME_SLOT_8)
                    || (LockKeyServeKeyList.KeyList[i].Type == KEY_TYPE_FOREVER)
                    || (LockKeyServeKeyList.KeyList[i].Type == KEY_TYPE_ONE_TIME)
                    || (LockKeyServeKeyList.KeyList[i].Type == KEY_TYPE_TIME_SLOT_7))
                {
                    offset = j;
                    tempTime = LockKeyServeKeyList.KeyList[i].StartTime;
                    year = BUILD_YEAR(tempTime);
                    month = BUILD_MONTH(tempTime);
                    day = BUILD_DAY(tempTime);
                    if((ProtoAnaly_Rtcinfo.Year == year)&&(ProtoAnaly_Rtcinfo.Month == month)
                    &&(ProtoAnaly_Rtcinfo.DayOfMonth == day))
                    { /* 加入黑名单*/
                        BlackList_Add(LockKeyServeKeyList.KeyList[i].Type, LockKeyServeKeyList.KeyList[i].KeyId,
                        BUILD_TIME(ProtoAnaly_Rtcinfo.Year,ProtoAnaly_Rtcinfo.Month,ProtoAnaly_Rtcinfo.DayOfMonth,
                        23,59,59));
                    }
                }
                memset(&LockKeyServeKeyList.KeyList[i], 0xff, sizeof(LockKey_ServeKeyType));
            }
        }
    }
    BlackList_StorePara();
    ServeKeyList_Writeflash( offset );    
    //ServeKeyList_StorePara1();//
    return E_OK;
}

/****************************************************************************************************
**Function:
    uint8 LockKey_Add_VisitCodeKey(uint8 *pData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
uint8 LockKey_Add_VisitCodeKey(uint8 idtype,uint32 keyId,  uint32 startTime, uint32 endTime)
{
    uint8 buftemp[13];
    uint8 err = E_NOT_OK;

    Sys_GenFun32To8(keyId, &buftemp[0]);
    Sys_GenFun32To8(endTime, &buftemp[4]);
    //buftemp[8] = 0;
    Sys_GenFun32To8(startTime, &buftemp[8]);
    buftemp[12] = 0xff;
    err = ProtoAnaly_AddId(idtype, buftemp, 1);
    return err;
}

/****************************************************************************************************
**Function:
    uint8 LockKey_Add_BlueKey(BlueKey_AllType BlueKey)
**Author: rory
**Description:
**Input: type: 0(All)  1(one)
             limitTime:限制时间
**Output: 
****************************************************************************************************/
uint8 LockKey_Remove_VisitCodeKey(void)
{
    uint8 offset;
    uint8 year,month,day;
    uint32 tempTime;
    for (uint8 j=0;j<SERVE_KEY_PAGEMAX;j++ )
    {
        ServeKeyList_Readflash( j );    
        for(uint8 i=0; i<SERVE_KEY_MAX; i++)
        {
            if(LockKeyServeKeyList.KeyList[i].Type == KEY_TYPE_TIME_SLOT_7)
            {
                offset = j;
                tempTime = LockKeyServeKeyList.KeyList[i].StartTime;
                year = BUILD_YEAR(tempTime);
                month = BUILD_MONTH(tempTime);
                day = BUILD_DAY(tempTime);
                if((ProtoAnaly_Rtcinfo.Year == year)&&(ProtoAnaly_Rtcinfo.Month == month)
                &&(ProtoAnaly_Rtcinfo.DayOfMonth == day))
                { /* 加入黑名单*/
                    BlackList_Add(LockKeyServeKeyList.KeyList[i].Type, LockKeyServeKeyList.KeyList[i].KeyId,
                    BUILD_TIME(ProtoAnaly_Rtcinfo.Year,ProtoAnaly_Rtcinfo.Month,ProtoAnaly_Rtcinfo.DayOfMonth,
                    23,59,59));
                }
                memset(&LockKeyServeKeyList.KeyList[i], 0xff, sizeof(LockKey_ServeKeyType));
            }
        }
    }
    BlackList_StorePara();
    
    ServeKeyList_Writeflash( offset );    
    //ServeKeyList_StorePara1();//
    return E_OK;
}

/****************************************************************************************************
**Function:
    uint8 LockKey_Add_CardKey(uint8 *pData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
uint8 LockKey_Start_LearnCardKey(uint8 *pData)
{
    Access_UserId_Temp = BUILD_UINT32(pData[3],pData[2],pData[1],pData[0]);
    memcpy(Access_AttTIme_Temp, pData+12, 4); /* 开始时间*/
    memcpy(&Access_AttTIme_Temp[4], pData+8, 4); /* 结束j时间*/
        Access_AttTIme_Temp[8] = pData[16];
    Sys_PataCfg.State = WORK_LEARN_CARD;
    Access_CardLearnTimer64 = 30000/64;
#ifdef ENABLE_CONSOLE_LOG
        printf("D2\n\r");
#endif // ENABLE_CONSOLE_LOG
    BLE_UPDATA_DISCONNECT_TIMER(30000/64);
    return E_OK;
}
/****************************************************************************************************
**Function:
    uint8 LockKey_Add_CardKey(uint8 *pData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
uint8 LockKey_Start_LearnCardKeyExt(uint8 *pData)
{
    Access_UserId_Temp = BUILD_UINT32(pData[3],pData[2],pData[1],pData[0]);
    memcpy(Access_AttTIme_Temp, pData+8, 4); /* 开始时间*/
    memcpy(&Access_AttTIme_Temp[4], pData+4, 4); /* 结束j时间*/
    Sys_PataCfg.State = WORK_LEARN_CARD;
    Access_CardLearnTimer64 = 30000/64;
    return E_OK;
}

/****************************************************************************************************
**Function:
    uint8 LockKey_Add_BlueKey(BlueKey_AllType BlueKey)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
uint8 LockKey_Remove_Key(uint8 *pData, uint8 type)
{
    uint8 i;
    uint32 userId = BUILD_UINT32(pData[3], pData[2], pData[1], pData[0]);

        if(LockKeyStatistics.ServKeyCount == 0)
        {
            return E_OK;
        }
        for (uint8 j=0;j<SERVE_KEY_PAGEMAX;j++ )
        {
            ServeKeyList_Readflash( j );            
            for(i=0; i<SERVE_KEY_MAX; i++)
            {
                if(LockKeyServeKeyList.KeyList[i].KeyId == userId)
                {
                    uint8 year, month, day;
                    uint32 tempTime;
                    tempTime = LockKeyServeKeyList.KeyList[i].StartTime;
                    year = BUILD_YEAR(tempTime);
                    month = BUILD_MONTH(tempTime);
                    day = BUILD_DAY(tempTime);
                    if((ProtoAnaly_Rtcinfo.Year == year)&&(ProtoAnaly_Rtcinfo.Month == month)
                    &&(ProtoAnaly_Rtcinfo.DayOfMonth == day))
                    { /* 加入黑名单*/
                        BlackList_Add(LockKeyServeKeyList.KeyList[i].Type, LockKeyServeKeyList.KeyList[i].KeyId,
                        BUILD_TIME(ProtoAnaly_Rtcinfo.Year,ProtoAnaly_Rtcinfo.Month,ProtoAnaly_Rtcinfo.DayOfMonth,
                        23,59,59));
                        BlackList_StorePara();
                    }
                    memset(&LockKeyServeKeyList.KeyList[i], 0xff, sizeof(LockKey_ServeKeyType));
                    if(LockKeyStatistics.ServKeyCount > 0)
                    {
                        LockKeyStatistics.ServKeyCount--;
                    }
                    
                    ServeKeyList_Writeflash( j );
                    //ServeKeyList_StorePara1();
                    return E_OK;
                }
            }
        }
        /* 密码为未使用*/
        BlackList_Add(type, userId,
                    BUILD_TIME(ProtoAnaly_Rtcinfo.Year,ProtoAnaly_Rtcinfo.Month,ProtoAnaly_Rtcinfo.DayOfMonth,
                    23,59,59));
        BlackList_StorePara();
    return E_OK;
}

/****************************************************************************************************
**Function:
    void BlackList_LoadPara(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void BlackList_Add(uint8 Type, uint32 KeyId, uint32 InvalidTime)
{
    uint8 index = 0xff;

    for(uint8 i=0; i<BLACK_LIST_MAX; i++)
    {
        if(LockKeyBlackList.BlackList[i].KeyId == KeyId)
        {
            LockKeyBlackList.BlackList[i].Type = Type;
            LockKeyBlackList.BlackList[i].KeyId = KeyId;
            LockKeyBlackList.BlackList[i].InvalidTime = InvalidTime;
            return;
        }
        else if(index == 0xff)
        {
            if(LockKeyBlackList.BlackList[i].KeyId == 0xffffffff)
            {
                index = i;
            }
        }
    }
    if(index != 0xff)
    {
        LockKeyBlackList.BlackList[index].Type = Type;
        LockKeyBlackList.BlackList[index].KeyId = KeyId;
        LockKeyBlackList.BlackList[index].InvalidTime = InvalidTime;
        LockKeyStatistics.BlackListCount++;
    }

    if(index==0xff && LockKeyStatistics.BlackListCount>BLACK_LIST_MAX)
    {
        ErrCodeBlackListFull = true;
    }
}


/****************************************************************************************************
**Function:
    void LockKey_CheckRemoveBlackList(uint8 index)
**Author: rory
**Description: 返回id所在色索引 
**Input: 
**Output: 
****************************************************************************************************/
uint8 BlackList_Remove(uint8 index)
{    
    if(LockKeyBlackList.BlackList[index].InvalidTime < ProtoAnaly_RtcLocalTime)
    {
        memset(&LockKeyBlackList.BlackList[index], 0xff, sizeof(LockKey_BlackType));
        if(LockKeyStatistics.BlackListCount > 0)
        {
            LockKeyStatistics.BlackListCount--;
        }
        BlackList_StorePara();
        return E_OK;
    }
    return E_NOT_OK;
}


/****************************************************************************************************
**Function:
    void BlackList_LoadPara(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void BlackList_LoadPara(void)
{
    FlashDrive.open(NULL); 
    Flash_ComType eepdata;
    eepdata.pData = (uint8_t *)&LockKeyBlackList;
    eepdata.Lenth = sizeof(LockKeyBlackList);
    eepdata.Add = ACCESS_BLACK_LIST_OFFSET;
    FlashDrive.read(&eepdata);
    if(LockKeyBlackList.KeyWord!= 0x12345678)
    {
        memset(&LockKeyBlackList,0xff,sizeof(LockKeyBlackList));
        BlackList_StorePara();
    }
}

/****************************************************************************************************
**Function:
    void BlackList_StorePara(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void BlackList_StorePara(void)
{
#if ((defined USE_FLASH_NRF) && (USE_FLASH_NRF == STD_TRUE))
    NrfFlash_PstorageType Message;

    if(NrfFlash_Store(NRF_FLASH_BLACK_LIST) == E_OK)
    {
        Message.Sector = NRF_FLASH_BLACK_LISTBK;
    }
    else
    {
        Message.Sector = NRF_FLASH_BLACK_LIST;
    }
    Message.pData= NULL;
    Message.Add = 0;
    Message.Lenth = 0;
    NrfFlashDrive.ioctl(NRFFLASH_STORE, &Message);
#else
    FlashDrive.open(NULL); 

    Flash_ComType eepdata;

    LockKeyBlackList.KeyWord = 0x12345678;
    eepdata.pData = (uint8_t *)&LockKeyBlackList;
    eepdata.Lenth = sizeof(LockKeyBlackList);
    eepdata.Add = ACCESS_BLACK_LIST_OFFSET;
    FlashDrive.ioctl(FLASH_ERASE_SECT,&eepdata.Add);
    FlashDrive.write(&eepdata);
#endif
}


/****************************************************************************************************
**Function:
    void BlackList_RemoveKey(uint8 idtpye, uint8 pData)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void BlackList_RemoveKey(uint8 idtpye, uint32 keyId)
{
    uint8 modify = 0;
    
    for(uint8 i=0; i<BLACK_LIST_MAX; i++)
    {
        if(idtpye == LockKeyBlackList.BlackList[i].Type)
        {
            if(keyId == LockKeyBlackList.BlackList[i].KeyId)
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
#ifdef ENABLE_CONSOLE_LOG
        printf("Blacklist entry removed\n\r");
#endif // ENABLE_CONSOLE_LOG
    }
}

void ServeKeyList_Readflash( uint8 offset )
{
    FlashDrive.open(NULL); 

    Flash_ComType eepdata;
    eepdata.pData = (uint8_t *)&LockKeyServeKeyList;
    eepdata.Lenth = sizeof(LockKeyServeKeyList);
    eepdata.Add = ACCESS_SERVE_KEY_OFFSET + (offset << 12);    // 
    FlashDrive.read(&eepdata);
}


void ServeKeyList_Writeflash(uint8 offset)
{
    FlashDrive.open(NULL); 
    Flash_ComType eepdata;
    
    eepdata.pData = (uint8_t *)&LockKeyServeKeyList;
    eepdata.Lenth = sizeof(LockKeyServeKeyList);
    eepdata.Add = ACCESS_SERVE_KEY_OFFSET + (offset << 12);
    FlashDrive.ioctl(FLASH_ERASE_SECT,&eepdata.Add);
    FlashDrive.write(&eepdata);
}


void CpuFobList_Readflash( uint8 offset )
{
    FlashDrive.open(NULL); 

    Flash_ComType eepdata;
    eepdata.pData = (uint8_t *)&CPUFobKey_List;
    eepdata.Lenth = sizeof(CPUFobKey_List);
    eepdata.Add = ACCESS_CPUFOB_OFFSET + (offset << 12);    // 
    FlashDrive.read(&eepdata);
}



void CpuFobList_Writeflash(uint8 offset)
{
    FlashDrive.open(NULL); 
    Flash_ComType eepdata;
    
    eepdata.pData = (uint8_t *)&CPUFobKey_List;
    eepdata.Lenth = sizeof(CPUFobKey_List);
    eepdata.Add = ACCESS_CPUFOB_OFFSET + (offset << 12);
    FlashDrive.ioctl(FLASH_ERASE_SECT,&eepdata.Add);
    FlashDrive.write(&eepdata);
}


static void RecordAddr_Writeflash( void )
{
    FlashDrive.open(NULL); 
    Flash_ComType eepdata;
    
    eepdata.pData = (uint8_t *)&Store_Addr;
    eepdata.Lenth = sizeof(Store_Addr);
    eepdata.Add = ACCESS_RECORDADDR_OFFSET ;
    FlashDrive.ioctl(FLASH_ERASE_SECT,&eepdata.Add);
    FlashDrive.write(&eepdata);
}
	
void  RecordAddr_Readflash( void )
{
    FlashDrive.open(NULL); 

    Flash_ComType eepdata;
    eepdata.pData = (uint8_t *)&Store_Addr;
    eepdata.Lenth = sizeof(Store_Addr);
    eepdata.Add = ACCESS_RECORDADDR_OFFSET;
    FlashDrive.read(&eepdata);

	if ( 0xffffffff == Store_Addr.RecordList )
	{
	   Store_Addr.RecordList = ACCESS_RECORD_OFFSET;
	   RecordAddr_Writeflash();
	}

	if ( 0xffffffff == Store_Addr.RecordIndex )
	{
	   Store_Addr.RecordIndex = ACCESS_RECORDINDEX_OFFSET;
	   RecordAddr_Writeflash();
	}
}


/****************************************************************************************************
**Function:
    void RecordList_LoadPara(uint8 offset)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void RecordList_LoadPara(uint8 offset)
{
    Flash_ComType eepdata;

    FlashDrive.open(NULL);
    eepdata.pData = (uint8_t *)&AccRcord;
    eepdata.Lenth = sizeof(AccRcord);
    eepdata.Add = Store_Addr.RecordList + (offset << 12);
    FlashDrive.read(&eepdata);

    if(AccRcord.TimerBk == 0xffffffff)
    {
        AccRcord.TimerBk = 0;
    }
}

/****************************************************************************************************
**Function:
    void RecordList_StorePara(void)
**Author: lxw
**Description:
**Input: mode:1  直接存储  0:延时存储
**Output: 
****************************************************************************************************/
void RecordList_StorePara(uint8 offset)
{
    AccRcord.EraseCnt++;
	if ( AccRcord.EraseCnt == ACCESS_PAGE_CNT )
	   {
		   Store_Addr.RecordList = ACCESS_RECORD_OFFSET1;
		   RecordAddr_Writeflash();
	   }
	else if ( AccRcord.EraseCnt == ACCESS_PAGE_CNT1 )
	   {
		   Store_Addr.RecordList = ACCESS_RECORD_OFFSET2;
		   RecordAddr_Writeflash();
	   }
	else if ( AccRcord.EraseCnt == ACCESS_PAGE_CNT2 )
	   {
		   Store_Addr.RecordList = ACCESS_RECORD_OFFSET3;
		   RecordAddr_Writeflash();
	   } 
	else if ( AccRcord.EraseCnt == ACCESS_PAGE_CNT4 )
	   {
		   Store_Addr.RecordList = ACCESS_RECORD_OFFSET4;
		   RecordAddr_Writeflash();
	   }
	else if ( AccRcord.EraseCnt == ACCESS_PAGE_CNT5 )
	   {
		   Store_Addr.RecordList = ACCESS_RECORD_OFFSET5;
		   RecordAddr_Writeflash();
	   } 
	else if ( AccRcord.EraseCnt == ACCESS_PAGE_CNT6 )
	   {
		   Store_Addr.RecordList = ACCESS_RECORD_OFFSET6;
		   RecordAddr_Writeflash();
	   }
	else if ( AccRcord.EraseCnt == ACCESS_PAGE_CNT7 )
	   {
		   Store_Addr.RecordList = ACCESS_RECORD_OFFSET7;
		   RecordAddr_Writeflash();
	   } 
	else if ( AccRcord.EraseCnt == ACCESS_PAGE_CNT8 )
	   {
		   Store_Addr.RecordList = ACCESS_RECORD_OFFSET8;
		   RecordAddr_Writeflash();
	   }
	else if ( AccRcord.EraseCnt == ACCESS_PAGE_CNT9 )
	   {
		   Store_Addr.RecordList = ACCESS_RECORD_OFFSET9;
		   RecordAddr_Writeflash();
	   }	
    FlashDrive.open(NULL); 

    Flash_ComType eepdata;

    eepdata.pData = (uint8_t *)&AccRcord;
    eepdata.Lenth = sizeof(AccRcord);

    eepdata.Add = Store_Addr.RecordList + (offset << 12);
    FlashDrive.ioctl(FLASH_ERASE_SECT,&eepdata.Add);
    FlashDrive.write(&eepdata);
}


void RecordIndex_LoadPara( void )
{
    Flash_ComType eepdata;

    FlashDrive.open(NULL);
    eepdata.pData = (uint8_t *)&PAccRcord;
    eepdata.Lenth = sizeof(PAccRcord);
    eepdata.Add = Store_Addr.RecordIndex ;
    FlashDrive.read(&eepdata);

    if(PAccRcord.Srecordindex.sectorIndex > ACCESS_RECORD_MAX)
    {
        PAccRcord.Srecordindex.sectorIndex = 0;
    }
    if(PAccRcord.Srecordindex.pageIndex > ACCESS_PAGE_MAX)
    {
        PAccRcord.Srecordindex.pageIndex = 0;
    }	
	
    if(PAccRcord.Wrecordindex.sectorIndex > ACCESS_RECORD_MAX)
    {
        PAccRcord.Wrecordindex.sectorIndex = 0;
    }
    if(PAccRcord.Wrecordindex.pageIndex > ACCESS_PAGE_MAX)
    {
        PAccRcord.Wrecordindex.pageIndex = 0;
    }	
}

void RecordIndex_StorePara( void )
{
    PAccRcord.EraseCnt++;
	
	if ( PAccRcord.EraseCnt == ACCESS_PAGE_CNT )
	   {
		   Store_Addr.RecordIndex = ACCESS_RECORDINDEX_OFFSET1;
		   RecordAddr_Writeflash();
	   }
	else if ( PAccRcord.EraseCnt == ACCESS_PAGE_CNT1 )
	   {
		   Store_Addr.RecordIndex = ACCESS_RECORDINDEX_OFFSET2;
		   RecordAddr_Writeflash();
	   }
	else if ( PAccRcord.EraseCnt == ACCESS_PAGE_CNT2 )
	   {
		   Store_Addr.RecordIndex = ACCESS_RECORDINDEX_OFFSET3;
		   RecordAddr_Writeflash();
	   } 
	else if ( PAccRcord.EraseCnt == ACCESS_PAGE_CNT4 )
	   {
		   Store_Addr.RecordIndex = ACCESS_RECORDINDEX_OFFSET4;
		   RecordAddr_Writeflash();
	   }
	else if ( PAccRcord.EraseCnt == ACCESS_PAGE_CNT5 )
	   {
		   Store_Addr.RecordIndex = ACCESS_RECORDINDEX_OFFSET5;
		   RecordAddr_Writeflash();
	   } 
	else if ( PAccRcord.EraseCnt == ACCESS_PAGE_CNT6 )
	   {
		   Store_Addr.RecordIndex = ACCESS_RECORDINDEX_OFFSET6;
		   RecordAddr_Writeflash();
	   }
	else if ( PAccRcord.EraseCnt == ACCESS_PAGE_CNT7 )
	   {
		   Store_Addr.RecordIndex = ACCESS_RECORDINDEX_OFFSET7;
		   RecordAddr_Writeflash();
	   } 
	else if ( PAccRcord.EraseCnt == ACCESS_PAGE_CNT8 )
	   {
		   Store_Addr.RecordIndex = ACCESS_RECORDINDEX_OFFSET8;
		   RecordAddr_Writeflash();
	   }
	else if ( PAccRcord.EraseCnt == ACCESS_PAGE_CNT9 )
	   {
		   Store_Addr.RecordIndex = ACCESS_RECORDINDEX_OFFSET9;
		   RecordAddr_Writeflash();
	   }	
    
    FlashDrive.open(NULL); 

    Flash_ComType eepdata;

    eepdata.pData = (uint8_t *)&PAccRcord;
    eepdata.Lenth = sizeof(PAccRcord);

    eepdata.Add = Store_Addr.RecordIndex;
    FlashDrive.ioctl(FLASH_ERASE_SECT,&eepdata.Add);
    FlashDrive.write(&eepdata);
}

