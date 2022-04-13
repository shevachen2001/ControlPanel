/*****************************************************************************
|  File Name: sys.h
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
| 2013-10      LXW     Initial version 
|****************************************************************************/

#ifndef SYS_H_
#define SYS_H_

#include "PUBLIC\Std_Types.h"
#include "lora\radio.h"
#include "Access\Access.h"

/******************************************************************************
**********************Macro definitiont*******************************************
******************************************************************************/
#define BATT_VOLPER_MAX    4

#define                SYSID                        0xffff
#define                CARLOCKID                    10

#define             PUBLIC_BUID_MAX                   64  
#define             BUILD_MAX                               10
#define            UPDATE_CHECKSUM                0x55667788


#define                SYSBIGFLG                        0x01
#define                SYSCARD3BYTE                    0x02
#define                DETCARDSLEEP                    0x04   /* 卡片休眠检测*/
#define                AUTOLOCK                           0x08   /* 自动上锁*/
#define                BTSAVEMODE                       0x10  /* 蓝牙触摸广播*/
#define                        DISIBLEDOORLOCK                          0X20  /* 门磁检测*/
#define                        DISIBLEBLUETOOTH                        0X40


#define WKUP_BLE                     0                                         
#define WKUP_KEY_INT                   1                                         
#define WKUP_LORA_INT                2                                         
#define WKUP_LORA_TIME                3                                         
#define             SYS_MAXID                    48
#define             SYS_MINID                    20

#define Sys_CheckCardSleepDet()      (Sys_PataCfg.CfgFlag&DETCARDSLEEP)
#define Sys_CheckBTSleepMode()      (Sys_PataCfg.CfgFlag&BTSAVEMODE)
#define Sys_CheckAutoLock()             (Sys_PataCfg.CfgFlag&AUTOLOCK)
#define Sys_CheckDisableAutoLock() ((Sys_PataCfg.CfgFlag&AUTOLOCK) == 0)
#define Sys_CheckDisableDoorLock() (Sys_PataCfg.CfgFlag&DISIBLEDOORLOCK)
#define Sys_CheckEnableDoorLock()  ((Sys_PataCfg.CfgFlag&DISIBLEDOORLOCK) == 0)
#define Sys_EnableBlueTooth()      ((Sys_PataCfg.CfgFlag&(uint8)DISIBLEBLUETOOTH) == 0)
#define Sys_DisableBlueTooth()      ((Sys_PataCfg.CfgFlag&(uint8)DISIBLEBLUETOOTH))
#define Sys_ConfigResetButton()     (true == Sys_PataCfg.resetButtonEnable)
#define SIZE_AES_KEY 16

#define VOICEMODE     0x00
#define PASSMODE      0x01
#define NBOPENMODE    0x02
#define REFACTORYMODE 0x03

#define SET_MODE   1
#define CLEAR_MODE 2

//#define ENABLE_CONSOLE_LOG
/******************************************************************************
**********************Type statement*******************************************
******************************************************************************/
typedef enum 
{
    SYS_STARTUP         = 0,
    SYS_NORMAL,
    SYS_SLEEP,
}Sys_StateType ;

enum
{
    CONFIG_VOICE = 1,
    CONFIG_PASS,
    CONFIG_NBMODE,
    CONFIG_REFACTORY,
};

typedef struct {
    uint16     SerSocketPort;        // 远程端口
    uint32  remoteCommIp;        //服务器  IP
    char     TargetDnsName[40];    // 域名
    char     ApnCh[20];
}TcpIpPataType;//运行模式

typedef struct {
    uint32    remoteCommIp;        //服务器  IP
    char     TargetDnsName[40];
    uint32    CheckSum;
}FtpInfoType;

typedef struct {
    uint16    connectOff;        //断开连接
    uint8       txPower;             // 发射功率
    uint16     advInt;                // 广播间隔
}BLEInfoType;

typedef struct {
    Radio_ParaType  rfPara;
    BLEInfoType  bleCfg;
    uint16   CustomId; /* 用于发卡器加密密码*/
    uint8     AppSleepBread;
    uint8     Headadd;
    uint8     SyncIndex;
    uint8   usedpage;
    uint8     EncpyPsd[8];
    uint8     AarmEnable; /* 报警提示1:开启;  0:关闭*/
    uint16  randindex;
    uint16     SysId;
    uint8     Aeskey[16];
    uint32    HeadMac;
    uint32     Pswd;
    uint32     Mac;
    uint16     FactoryTimeout;	
    uint16     OpenLockTime; /* 开锁保持时间*/
    uint16     HeartTime; /* 心跳帧时间* 16ms */
    uint8     BuildId; /* 组ID  1 ~ 255 */    // 0 :公共门
    uint16     PropertyId; /*酒店ID :0 ~ 65535 */
    uint16  HotelPswd;
    uint8     CfgFlag; /*配置信息*/
    uint32  StartTime; // 动态密码卡片开始时间
    uint8   touchSensitive; // 触摸板灵敏度调节
    uint8   UploadRecord;
    uint8   State;
    uint8   keyPadInx;       //keypad 密码
    uint8   keyPadPsw[200];  //keypad 密码
    uint32  KeyWord;
    uint8   LongKeyFlag;
    _Bool   bleDSTFlag;
    _Bool   lockDSTFlagbak;
    _Bool   lockTime_SummerTimer_SkippedFlag;
    _Bool   lockTime_WinterTimer_SkippedFlag;	
    _Bool   resetButtonEnable;
    uint8   configMode;
    uint32_t passageEndTime;
    uint32_t passageStartTime;
    uint8_t  weeklyPeriodicity;
    uint32_t settimeBak;
    uint8    lockDSTFlag;	
}Sys_PataCfgType;

/******************************************************************************
**********************Variable definition******************************************
******************************************************************************/
extern Sys_PataCfgType Sys_PataCfg;
extern uint8 Sys_CheckUpFlag ;
extern uint8  Sys_McuRestFlag;
extern uint8  Sys_StoreFlag;
extern uint16 Sys_RstTimsoutx64ms;
extern uint16 Sys_StoreTimsoutx64ms;
extern uint32 Sys_Tick ;
extern uint8  Sys_WkupType;
extern uint8  Sys_ResetDevice;
extern Sys_StateType Sys_State;
extern uint8  Sys_frstPoweron;
extern const uint16 Batt_VolPerTable[BATT_VOLPER_MAX][2];
extern uint16 Sys_SleepErrTimsoutx64ms ;

/******************************************************************************
**********************Function declared*******************************************
******************************************************************************/
extern void Sys_EnterWaitWakeUp(void);
extern void Sys_IoEnterSleep(void);
extern void Sys_IoExitSleep(void);
extern void Sys_IoStartUp(void);
extern void Sys_InitClock(void);
extern void Sys_StorePara(void);
extern void Sys_LoadPara(void);
extern void Sys_Init(void);
extern void Sys_StateProc(void);
extern void Sys_Parainit(void);
extern void Sys_ParainitFirst(void);
extern void Sys_RestProc(void);
extern void Sys_wdt_feed(void);  
extern void Sys_wdt_start(void);  
#if (defined DEBUG_ENABLE) && (DEBUG_ENABLE == STD_TRUE)
extern void  Sys_Debug(uint8 mode);
#endif

extern void Sys_SpiOpen(void);
extern void Sys_SpiClose(void);
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

