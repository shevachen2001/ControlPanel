/*****************************************************************************
|  File Name: DataUp.h
|
|  Description: Header File defined
|
|-----------------------------------------------------------------------------
|               C O P Y R I G H T
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
|               A U T H O R   I D E N T I T Y
|-----------------------------------------------------------------------------
| Initials     Name                      Company
| --------     ---------------------     -------------------------------------
| LXW          LXW               
|-----------------------------------------------------------------------------
|               R E V I S I O N   H I S T O R Y
|-----------------------------------------------------------------------------
| Date        Ver     Author  Description
| ---------   ----    ------  ------------------------------------------------
| 2016-05      Rory     Initial version 
|****************************************************************************/

#ifndef __ACCESS_H
#define __ACCESS_H
#include "PUBLIC\Std_Types.h"
#include "Card\Rc522.h"
#include "Beep\Beep.h"
#include "Light\light.h"

#define ACCESS_RECORD_ONCE_MAX             14
#define ACCESS_RECORD_MAX                  250//250     //  sizeof(AccRcord) = 0xBCC =3020<4096
#define ACCESS_PAGE_MAX                    4 


#define ACCESS_OPEN_LOCK_TPYE              1
#define ACCESS_CLOSE_LOCK_TPYE             0
#define ACCESS_NG_TPYE                     0x02

#define FLASH_ONCEBLOCK_SIZE               128*1024
#define ACCESS_SYNC_PASWD_MAX              10

// MAX 4M = 4096k = 0x40 0000
#define ACCESS_SYS_PARA_OFFSET          0x10000                // BLOCK 1         
#define ACCESS_BLACK_LIST_OFFSET        0x20000                // BLOCK 2   USE 2 BLOCK   
#define ACCESS_SERVE_KEY_OFFSET         0x40000                // BLOCK 4   USE 2 BLOCK   
#define ACCESS_CPUFOB_OFFSET            0x60000                // BLOCK 6   USE 2 BLOCK   

#define ACCESS_RECORDINDEX_OFFSET        0xA0000
#define ACCESS_RECORDINDEX_OFFSET1       ACCESS_RECORDINDEX_OFFSET +0x1000 
#define ACCESS_RECORDINDEX_OFFSET2       ACCESS_RECORDINDEX_OFFSET1+0x1000 
#define ACCESS_RECORDINDEX_OFFSET3       ACCESS_RECORDINDEX_OFFSET2+0x1000 
#define ACCESS_RECORDINDEX_OFFSET4       ACCESS_RECORDINDEX_OFFSET3+0x1000 
#define ACCESS_RECORDINDEX_OFFSET5       ACCESS_RECORDINDEX_OFFSET4+0x1000 
#define ACCESS_RECORDINDEX_OFFSET6       ACCESS_RECORDINDEX_OFFSET5+0x1000 
#define ACCESS_RECORDINDEX_OFFSET7       ACCESS_RECORDINDEX_OFFSET6+0x1000 
#define ACCESS_RECORDINDEX_OFFSET8       ACCESS_RECORDINDEX_OFFSET7+0x1000 
#define ACCESS_RECORDINDEX_OFFSET9       ACCESS_RECORDINDEX_OFFSET8+0x1000 
#define ACCESS_RECORDINDEX_OFFSETMAX     ACCESS_RECORDINDEX_OFFSET9+0x1000 


#define ACCESS_RECORDADDR_OFFSET         0xB0000


#define ACCESS_RECORD_OFFSET             0x100000 
#define ACCESS_RECORD_OFFSET1            ACCESS_RECORD_OFFSET + 0x4000 
#define ACCESS_RECORD_OFFSET2            ACCESS_RECORD_OFFSET1 + 0x4000 
#define ACCESS_RECORD_OFFSET3            ACCESS_RECORD_OFFSET2 + 0x4000 
#define ACCESS_RECORD_OFFSET4            ACCESS_RECORD_OFFSET3 + 0x4000 
#define ACCESS_RECORD_OFFSET5            ACCESS_RECORD_OFFSET4 + 0x4000 
#define ACCESS_RECORD_OFFSET6            ACCESS_RECORD_OFFSET5 + 0x4000 
#define ACCESS_RECORD_OFFSET7            ACCESS_RECORD_OFFSET6 + 0x4000 
#define ACCESS_RECORD_OFFSET8            ACCESS_RECORD_OFFSET7 + 0x4000 
#define ACCESS_RECORD_OFFSET9            ACCESS_RECORD_OFFSET8 + 0x4000 
#define ACCESS_RECORD_OFFSETMAX          ACCESS_RECORD_OFFSET9 + 0x4000 


#define ACCESS_PAGE_CNT               100000 
#define ACCESS_PAGE_CNT1              ( ACCESS_PAGE_CNT ) 
#define ACCESS_PAGE_CNT2              ( ACCESS_PAGE_CNT1 + ACCESS_PAGE_CNT )  
#define ACCESS_PAGE_CNT3              ( ACCESS_PAGE_CNT2 + ACCESS_PAGE_CNT )  
#define ACCESS_PAGE_CNT4              ( ACCESS_PAGE_CNT3 + ACCESS_PAGE_CNT )  
#define ACCESS_PAGE_CNT5              ( ACCESS_PAGE_CNT4 + ACCESS_PAGE_CNT )  
#define ACCESS_PAGE_CNT6              ( ACCESS_PAGE_CNT5 + ACCESS_PAGE_CNT )  
#define ACCESS_PAGE_CNT7              ( ACCESS_PAGE_CNT6 + ACCESS_PAGE_CNT )  
#define ACCESS_PAGE_CNT8              ( ACCESS_PAGE_CNT7 + ACCESS_PAGE_CNT )  
#define ACCESS_PAGE_CNT9              ( ACCESS_PAGE_CNT8 + ACCESS_PAGE_CNT )  


/******************************************************************************
**********************Macro definition*******************************************
******************************************************************************/
enum
{
    WORK_NORMAL, /* ???*/
    WORK_NORMALLY_OPEN,   /* ??????*/
    WORK_LEARN_CARD,
    WORK_TEST,
};

enum
{
    TYPE_LOCK_HOME = 0, /* ????????????*/
    TYPE_LOCK_FLAT, /* ?????????*/ 
    TYPE_LOCK_CAR, /* ?????????*/
    TYPE_KEY, /* ??????????????????*/
    TYPE_LOCK_HANDUP, /* ????????????*/
    TYPE_LOCK_ACCESS , /* ??????*/
    TYPE_LOCK_ACCESS_WIG , /* ??????????????????*/
    TYPE_LOCK_ACCESS_REMOTE, /* ??????????????????*/
    TYPE_LOCK_VIDEO, /* ??????????????????*/
    TYPE_LOCK_BIKE, /* ??????????????????*/
    TYPE_LOCK_BIKE_REMOTE, /* ??????????????????*/
    TYPE_LOCK_HOTEL_REMOTE = 11,  /* ??????????????????????????????*/
    TYPE_LOCK_HOTEL = 12,  /* ??????????????????*/
    TYPE_LOCK_FIGNERPRINT , /* ?????????*/
    TyPE_LOCK_LORA_GATEWAY = 16, /* Lora??????*/
    TyPE_LOCK_SEND_CARD = 31, /* ?????????*/
    TYPE_LAB = 0x20, /* ??????????????????*/
};


enum
{
	KEY_TYPE_ALL = 0,
	KEY_TYPE_APP,
	KEY_TYPE_PASSWORD,
	KEY_TYPE_CARD,
	KEY_TYPE_CPUCARD,
	KEY_TYPE_FINGERPRINT,
	KEY_TYPE_IDENTITY_CARD,
	KEY_TYPE_QRCODE , // ???????????????
	KEY_TYPE_USERID,
	KEY_TYPE_MAX,
	KEY_TYPE_CLEAR_ALL=KEY_TYPE_MAX,//= 10, ?????????????????????
	
	KEY_TYPE_TIME_ONE,            /*6?????????????????????*/
	KEY_TYPE_TIME_INDEX,          /*6????????????????????????*/
	KEY_TYPE_TIME_SLOT,           /*6????????????????????????*/
	KEY_TYPE_TIME_DATE,            /*8???????????????*/
	KEY_TYPE_MECHANICAL_KEY = 13,  /* ??????????????????*/
	
	RECORD_EMERGENCY_CARD=15,       /* ?????????*/
	RECORD_TOTAL_CARD=16,           /* ??????*/
	RECORD_STAFF_CARD=17,           /* ???????????????*/
	RECORD_STAFF_LOOP_CARD=18,      /* ????????????*/
	RECORD_CUSTOM_CARD = 19,          /* ?????????*/
  RECORD_INSPECTION_CARD = 20,      /* ?????????*/

	KEY_TYPE_ONE_TIME = 21,        //???????????????
	KEY_TYPE_FOREVER = 22,         // ????????????
	KEY_TYPE_TIME_SLOT_7 = 23,     //7??? ?????????
	KEY_TYPE_CLEAR = 24,           // ????????????
	KEY_TYPE_TIME_SLOT_8 = 25,     //8??? ?????????

	ACCESS_INFO_LOCK_STATUS = 28,          /* ?????????????????????:  act:   bit0:?????????(1:??? 0:???) bit1:??????(1:??????)*/
	ACCESS_FALSE_LOCK_ALARM = 29,          /* ????????????*/
	ACCESS_LEARN_IDENTITY_CARD_TPYE = 30,  /* ?????????*/
	ACCESS_LEARN_CARD_TPYE = 31,
	
	KEY_TYPE_FAST_APP_NOID_OPEN = 0xf1,  //APP????????????
	KEY_TYPE_FAST_PSWD_OPEN,             //?????????????????????
	KEY_TYPE_FAST_CARD_OPEN,             //??????????????????
	KEY_TYPE_FAST_QRCODE_OPEN,           //?????????????????????
};


enum
{
    ACCESS_DET_WAIT = 0,
    ACCESS_DET_M1,
    ACCESS_DET_T5557ST,
    ACCESS_DET_T5557STOP,
};

enum
{
    CARD_TYPE_STAFF_LOOP = 0,  /* ????????????????????????:??????*/
    CARD_TYPE_STAFF,  /* ????????? ?????????*/
    CARD_TYPE_LOSS, /* ?????????*/
    CARD_TYPE_EMERGENCY, /* ????????? */
    CARD_TYPE_ALL, /* ??????*/
    CARD_TYPE_INSPECTION, /* ?????????*/
    CARD_TYPE_TESK, /* ????????? */
    CARD_TYPE_GUEST, /* ?????????*/
    CARD_TYPE_NULL, /* ?????????*/
};

enum
{
	CARD_TYPE_READ_OK = 0,  
	CARD_CPU_APDU_ERR,
	CARD_CPU_AUTH_ERR,
	CARD_CPU_LOCK_ERR,
	CARD_CPU_FINDKEY_ERR,
	CARD_READ_NZ3801_ERR,	
};


enum
{
    ACCESS_AUTO_TEST_IDLE= 0,
    ACCESS_AUTO_TEST_CLOCK,
    ACCESS_AUTO_TEST_BLE,
    ACCESS_AUTO_TEST_IC_ID_IDENTITY,    
    ACCESS_AUTO_TEST_KEYTABLE,
    ACCESS_AUTO_TEST_WIFI,
    ACCESS_AUTO_TEST_GPRS,
};

#define LOCK_HOME   0
#define LOCK_FLAT    1
#define LOCK_HOTEL_FLAT    2
#define LOCK_HOTEL_FLAT_REMOTE    3

#if (defined LORA_ENABLE) && (LORA_ENABLE == STD_TRUE)    
#define LOCK_TYPE   LOCK_HOTEL_FLAT_REMOTE
#else
#define LOCK_TYPE   LOCK_HOTEL_FLAT
#endif


#define LOCK_TOUCH_REMOTE_OPEN_TIME    20000/64

//#define ACCESS_FASTCMPMAX    1024
//#define ACCESS_FASTCMPMIN    512
#define OPEN_REVERSE_DOOR            0x01   /* ??????????????????*/
#define OPEN_COMMOM_DOOR            0x02   /* ???????????????*/
#define LOSS_OLD_CARD                   0x03   /* ???????????????*/

/******************************************************************************
**********************Type statement*******************************************
******************************************************************************/
enum
{
	SUMMER_TIMER_NOSKIPED_FLAG   = 0x00,
	WINTER_TIMER_NOSKIPED_FLAG   = 0x00,
	SUMMER_TIMER_SKIPED_FLAG     = 0x01,
	WINTER_TIMER_SKIPED_FLAG     = 0x01,
};

enum
{
	SUMMER_TIMER_FLAG = 0x00,
	WINTER_TIMER_FLAG = 0x01,
	DEC_TIMER_FLAG = 0x02,  //Minus One hour sign in winter time to prevent triggering the Minus operation again.
};

typedef enum
{
	SUMMER_TIMER = 0x00,
	WINTER_TIMER = 0x01,
}SummerAndWinterTimeTypedef;

typedef enum
{
	PANEL_SUMMER_TIMER = 0x00,
	PANEL_WINTER_TIMER = 0x01,
}ControlPanelTimeTypedef;

typedef struct {
    uint8  WegeMode; /* 0: 34; 1:26 */
    uint8  ErrorTimes;
    uint16 ErrorTimesfob;

    //uint8 State;
    uint8 DefaultTimes;
    uint8 UserIndex;
}Access_ParaType;    

typedef struct {
    uint16 UserIndex; 
    uint16 FlashIndex;
}Access_IdIndxType;    

typedef struct{
    uint8 keyId;//[ACCESS_SYNC_PASWD_MAX];
    uint8 KeyIndex;
    //uint8 KeyMax;
    //uint8 Year;
    //uint8 Month;
    //uint8 DayOfMonth;
}Access_SyncPaswdType;

typedef struct {
    uint8     FlashInitFlag;
}Access_flahInfoType;

typedef struct {
    uint8  cardtype;
    uint16 hotelId;
    uint32 deviceId;
    uint32 devicePaswd;
    uint16 HotelPaswd;
    uint32 Timest;
    uint32 Timeend;
}Access_CardDatType;

typedef struct {
    uint8 TypeResult;//????????????| ????????????
    uint32 Id;//??????ID
    uint32 Time;//????????????
}AccRcordDataType;

typedef struct {
    uint8 pageIndex;
    uint8 sectorIndex;
} ReadIndexDataType;

typedef struct {
    uint8 pageIndex;
    uint8 sectorIndex;
} WriteIndexDataType;

typedef struct {
    uint32 TimerBk;
    AccRcordDataType RecordList[ACCESS_RECORD_MAX+1];
    uint32 EraseCnt;
}AccRcordParaType;

typedef struct {
	WriteIndexDataType Wrecordindex;
	ReadIndexDataType Srecordindex;
	uint16 recordnum;
	uint32 EraseCnt;
}
AccRcordParaindex;

#define CPUFOB_INVALID   33
#define ADV_ERROR        34


typedef struct {
    uint16   Year;
    uint8    Month;
    uint8    DayOfMonth;
    uint8    Hour;
    uint8    Minute;
    uint8    Second;
    uint8     DayOfWeek;
}Rtc_Timestamp;


/******************************************************************************
**********************Variable definition*******************************************
******************************************************************************/
extern Access_ParaType Access_Globle;
extern uint16 Access_ResetDeviceTimer64;
extern uint16 Access_OpenRedIndex ;
extern uint16 Access_OpenWriteaIndex ;
extern uint16 Access_Store_Recordx64ms;
extern uint8 protoAnaly_netCommType;

extern uint16 Access_LockTimer64;
extern uint16 Access_DefaultTimer64;
extern uint16 Access_CardLearnTimer64;
extern uint16 Access_BatteryData;

extern AccRcordParaType AccRcord;
extern AccRcordParaindex PAccRcord;

extern uint32 Access_UserId_Temp;
extern uint16 Access_LockDeviceTimer64ms;
extern uint8 Access_LockStatus;

extern uint8 cardopenFlag;
extern uint32 RtcLocalTimebak;
extern uint32 RtcLocalTime;
extern uint8 Access_Record_Overflow;
/****************************************************************************/
/* Function                                                                 */
/****************************************************************************/
extern void Access_Init(void);
extern void Access_Unlock(void);
extern void Access_Lock(void);
extern void Access_OpenError(void);


extern Std_ReturnType Access_ComTime(uint32 startTime, uint32 endTime);
extern Std_ReturnType Access_LearnCardOk(uint8 idtpye,uint8*cardid);
extern void Access_CardProcess(uint8 idtpye, uint8* pUid,Access_CardDatType *CardDat);

extern void Access_FlashArrang(void);
extern Std_ReturnType Access_GetSupportType(uint8 idtype);
extern void Access_TimerProc(void);

extern void Access_EraseAllKey(uint8 idtpye);
extern void Access_DefaultPaswd(void);
extern void Access_SendRecordProc(void);
extern void Access_WriteRecordFlash(uint8* pKeyId,uint32 time ,uint8 type, uint8 action);
extern void Access_BeepStart(Beep_StateType mode, uint8 times);
extern void Access_LightStart(Light_NameType index,Light_StateType mode, uint8 times,uint16 data);
extern void Vdd_PowerOff(void);
extern uint8  Access_GetRecord(uint16 index,uint8 *poutdat);
#define Access_DelayReset(x)   Access_ResetDeviceTimer64 = x/64

extern void Access_DetCardProc(void);
extern void Access_UpdateStatus(void);
extern void CheckRtc(void);

#endif    
/************************ (C) COPYRIGHT DaHao electronics *****END OF FILE****/

