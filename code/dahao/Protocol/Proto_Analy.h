/*****************************************************************************
|  File Name: ProtoAnaly.h
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

#ifndef PROTOANALY_H
#define PROTOANALY_H


#include"PUBLIC\Std_Types.h"
#include"Protocol\Proto_ComDef.h"
#include "RTC\Rtc.h"
#include "protocol\Proto_NetComm.h"


/******************************************************************************
**********************Macro definitiont*******************************************
******************************************************************************/
#define    PROTOANALY_GETACK_TIME        60/2
#define    PROTOANALY_BLE_RST_TIME        30*60*1000/64
#define    PROTOANALY_GETACK_TIMES        3

#define PROTOANALY_RECORD_CACHEMAX    3

#define    PROTOANALY_GPRS_RSTTIM        60*1000/64
#define    PROTOANALY_GPRS_ERR_MAX        3

#define    PROTOANALY_LAN_RSTTIM        6
#define    PROTOANALY_LAN_ERR_MAX        2

#define    FACTORYTEST_BLE_CONNECT_TIMEOUT   30
#define    FACTORYTEST_RESULT_SUCCESS        0x00
#define    FACTORYTEST_AUTLOCK_TIMEOUT       3

#if 0
#define    PROTOANALY_SEND_WINDOWTIMES        100
#define    PROTOANALY_REC_WINDOWTIMES        80
#define    PROTOANALY_CYCLETIMES_MAX            40
#define    PROTOANALY_CYCLETIMES_ONCE_MAX    8        
#else 
#define    PROTOANALY_SEND_WINDOWTIMES        50
#define    PROTOANALY_REC_WINDOWTIMES        40
#define    PROTOANALY_CYCLETIMES_MAX            80
#define    PROTOANALY_CYCLETIMES_ONCE_MAX    8        
#endif

enum
{
    FACTORYTEST_COMMD_DISBALED = 0x00,
    FACTORYTEST_COMMD_KEYINPUT = 0x01,
    FACTORTTEST_COMMD_FOBINPUT = 0x02,
    FACTORTTEST_COMMD_AUTOLOCK = 0x03,
};

enum
{
    FACTORYTEST_TEST_MODE_DISBALED = 0xB0,
    FACTORYTEST_TOUCH_TIMEOUT      = 0xB1,
    FACTORTTEST_FOB_TIMEOUT        = 0xB2,
    FACTORTTEST_MOTOR_LOCKED       = 0xB3,
};

enum
{
    PROTO_ID_ALL = 0,
    PROTO_ID_BLE,
    PROTO_ID_PASWD,
    PROTO_ID_M1,
    PROTO_ID_FINGERPRINT,
    PROTO_ID_IDENTITY_CARD,
    PROTO_ID_OTHER,
};

enum
{
    ACTIVE_CLOSE = 0,
    ACTIVE_OPEN,
    ACTIVE_ERROR,
};

enum
{
    PROTO_TEMP_INIT,
    PROTO_TEMP_SET_PARA,
    PROTO_TEMP_SNYC,
};
enum
{
    PROTO_ETH_LAN = 0,
    PROTO_ETH_GPRS,
    PROTO_ETH_WIFI,
};

enum
{
    PROTO_LORA_SNYC_TIME_OUT_CAD = 0,
    PROTO_LORA_SNYC_TIME_OUT_RX,
    PROTO_LORA_SNYC_TIME_OUT_TX,
    PROTO_LORA_SNYC_TIME_OUT_WKUP,
    PROTO_LORA_TIME_OUT_CAD,
    PROTO_LORA_TIME_OUT_RX,
    PROTO_LORA_TIME_OUT_TX,
    PROTO_LORA_TIME_OUT_WKUP,
    
    PROTO_LORA_TIME_CAD_CLOSE,
    PROTO_LORA_TIME_RX_CLOSE,
};

/******************************************************************************
**********************Type statement*******************************************
******************************************************************************/
typedef struct {
    uint8 ComdId;
    void (*comdinmanage)(uint8_t *pindata);
    void (*comdOutmanage)(uint8_t *poutdata);
}ProtoAnaly_ComdDnType;




typedef struct {
    uint8 ComdId;
    void (*comdinmanage)(uint8_t *pindata,uint8_t *poutdata );
}ProtoAnaly_ComdUpType;

/******************************************************************************
**********************Variable definition***************************************
******************************************************************************/

extern uint16    ProtoAnaly_HeartTimeOutx16ms;
extern uint32 ProtoAnaly_RtcLocalTime;
extern Rtc_Type ProtoAnaly_Rtcinfo;
extern uint8  ProtoAnaly_GetAckTimeOutx1S;
extern uint8 protoAnaly_netCommType;
extern uint16 ProtoAnaly_HeartTimeOutx64ms;
extern uint16 ProtoAnaly_LoraBrgTimeoutx64ms;
extern uint32 ProtoAnalyHashKey;
extern uint8  ProtoAnaly_LoraSendTims;
extern uint8  ProtoAnalyRecordIndex;

extern const uint8 PROTO_BREAD_TIME_TABLE[];
extern const uint32 PROTO_SLEEP_TIME_TABLE[];
/******************************************************************************
**********************Function declared****************************************
******************************************************************************/
extern void ProtoAnaly_CommUpProc(void);
extern void ProtoAnaly_StoreUpdate(void);
extern void ProtoAnaly_AutoGetTemp(void);
//extern ProtoAnaly_ComdFifoType *ProtoAnaly_GetCmdFifo(void);
extern void ProtoAnaly_Init(void);
extern uint32 ProtoAnaly_HashKeyInit(uint32 paswd);
extern void ProtoAnaly_CommEndCallBack(void);

extern void ProtoAnaly_CommTcpProc(void *ptmr, void *parg,uint8 retype);
extern void ProtoAnaly_SendBct(void);

extern void ProtoAnaly_PrivateCommProc(void);
extern void  Proto_SendAddIndex(void); 
extern void Proto_DpramUpdate(void);
extern void ProtoAnaly_AutoSendProc(void);
//Std_ReturnType ProtoAnaly_FindId(uint8 idtpye,uint32 id,uint32 *time);
extern void ProtoAnaly_CommTftpInfo(uint8 *ptmr);
extern void ProtoAnaly_CompTest(void);
extern void ProtoAnaly_UsartProc(void);
extern void ProtoAnaly_HeartTimeOutProc(void);
extern void ProtoAnaly_UpdateTime(void);
extern void ProtoAnaly_GprsHeartTimeOutProc(void);
extern void ProtoAnaly_ModifyCustomInfo(uint8_t *pInData);


extern void ProtoAnaly_AutoSenDatProc(void);


extern Std_ReturnType ProtoAnaly_CmdAppProc(Proto_NetcommType netype,uint8 *pindat,uint8 *poutdat);
extern uint8  ProtoAnaly_CmdMacProc(Proto_NetcommType netype,uint8 *pindat,uint8 *poutdat);
extern void ProtoAnaly_ProcEnd(Proto_NetcommType netype,uint8 cmd);
extern Std_ReturnType ProtoAnaly_PublicProc(Proto_NetcommType netype,uint8 *pindat,uint16 lenth,uint8 *poutdat);

extern Std_ReturnType ProtoAnaly_AddId(uint8 idtpye,uint8 *idbuf,uint8 num);
extern Std_ReturnType ProtoAnaly_RemoveId(uint8 idtpye,uint8* id);

extern void ProtoAnaly_SendHeartBeat(uint8 cmd,uint8 ch,uint8 state);
extern void ProtoAnaly_GetAckProc(void);
extern void ProtoAnaly_LoraSlaveTickSnyc(uint8 wktype,uint8 index,uint8 dereindex);
extern uint8 ProtoAnaly_RecordGetCmd(uint8 *pdat);
extern void ProtoAnaly_RecordRemoveCmd(void);
extern void ProtoAnaly_AddRecordCmd(uint8* pKeyId,uint32 time,uint8 type, uint8 action);
extern void Access_EraseRecordData(void);
extern Std_ReturnType ProtoAnaly_LoraDecrpyt(uint8 lenth,uint8 *dat);
extern void ProtoAnaly_LoraEncrypt(uint8 lenth,uint8 *dat);
extern uint32 BKDRHash(uint8 *pbuf,uint8 lenth);

extern void LoraWakeup_timer_Start(uint8 lstate, uint16 time);
extern void LoraRcing_timer_Delay(uint16 time);

extern void DeleteCPUID(uint32 keyid);


#endif    /* #ifndef MOAP_H */


