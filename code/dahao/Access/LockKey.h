/*****************************************************************************
|  File Name: LockKey.h
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

#ifndef __LOCK_KEY_H
#define __LOCK_KEY_H

#include "PUBLIC\Std_Types.h"
#include "Access\LockKey.h"
#include "KEY\key.h"

/******************************************************************************
**********************Macro definition*******************************************
******************************************************************************/
#define KEY_READ_ONE_SIZE          10 /* 单次读取大小*/

#define KEY_SIZE_MAX                    50  /* 总的存储数量*/

#define USER_ID_ADDR                    0 /* user ID 起始地址*/
#define ACTIVE_TIME_ADDR             (USER_ID_ADDR + KEY_SIZE_MAX*4) /* Active Time地址*/
#define APP_KEY_ADDR                 (ACTIVE_TIME_ADDR + KEY_SIZE_MAX*4) /* APP 起始地址*/
#define APP_PASWD_ADDR               (APP_KEY_ADDR + KEY_SIZE_MAX*4) /* PASWD起始地址*/
#define APP_CARD_ADDR                (APP_PASWD_ADDR + KEY_SIZE_MAX*4) /* CARD起始地址*/
#define APP_FINGER_PRINT_ADDR        (APP_CARD_ADDR + KEY_SIZE_MAX*4) /* 指纹起始地址*/


#define BLACK_LIST_MAX               200//84//168//84
//#define BLACK_LIST_PAGEMAX         16
#define CPUFOB_MAX                200
#define CPUFOB_PAGEMAX            10

/******************************************************************************
**********************Type statement*******************************************
******************************************************************************/
typedef struct {
    uint16 BlackListCount; 
    uint16 ServKeyCount;
    uint16 AppKeyCount;
}LockKey_StatisticsType;    
    
typedef struct {
    uint8 Type; 
    uint32 KeyId;
    uint32 InvalidTime;
    
}LockKey_BlackType;    
typedef struct {
    LockKey_BlackType BlackList[BLACK_LIST_MAX];
    uint8 flashIndex;
    uint32 KeyWord;
}LockKey_BlackListType;    

typedef struct {
    uint8 Type; 
    uint8 Cycle;
    uint32 KeyId;
    uint32 StartTime;
    uint32 EndTime;
}LockKey_ServeKeyType;

typedef struct {
    uint32 KeyId1; 
    uint32 KeyId2;
    uint32 StartTime;
    uint32 EndTime;
}LockKey_IdentityKeyType; /* 身份证*/

typedef struct {
    LockKey_ServeKeyType KeyList[SERVE_KEY_MAX];
    uint32 KeyWord;
}LockKey_ServeKeyListType;

typedef struct 
{
	 uint8 Fob_SN[4];
   uint8 Fob_KeyArray[16];
}CPUFobType;


typedef struct 
{
	CPUFobType CpuFobList[CPUFOB_MAX];
	uint32 EraseCnt;
}CPUFob_DataType;	


typedef struct
{
   uint32 RecordList;
   uint32 RecordIndex;
}Store_AddrType;


/******************************************************************************
**********************Variable definition*******************************************
******************************************************************************/
extern uint16 Access_CardLearnTimer64;

extern LockKey_StatisticsType LockKeyStatistics;
extern LockKey_BlackListType LockKeyBlackList;
extern LockKey_ServeKeyListType LockKeyServeKeyList;
extern uint8 Access_Record_Null;

/****************************************************************************/
/* Function                                                                 */
/****************************************************************************/
extern uint8 LockKey_Check_PaswdKey(uint8* pTouchKey,uint8 lenth);
extern uint8 LockKey_Check_CardKey(uint8 idtpye,uint8 *pid);
extern uint8 LockKey_Add_VisitCodeKey(uint8 idtype, uint32 keyId, uint32 startTime, uint32 endTime);
extern uint8 LockKey_Remove_VisitCodeKey(void);
extern uint8 LockKey_Start_LearnCardKey(uint8 *pData);
extern uint8 LockKey_Start_LearnCardKeyExt(uint8 *pData);

extern void LockKey_Default_Key(void);
extern uint8 LockKey_Read_Key(uint8 *pInData, uint8 *pOutData);
extern uint8 LockKey_Remove_Key(uint8 *pData, uint8 type);
extern uint8 LockKey_Remove_PasswordKey(void);

extern void BlackList_LoadPara(void);
extern void BlackList_StorePara(void);
extern void ServeKeyList_StorePara(void);
extern uint8 ServeKeyList_Remove(uint8 index);
extern void BlackList_Add(uint8 Type, uint32 KeyId, uint32 InvalidTime);
extern void BlackList_RemoveKey(uint8 idtpye, uint32 keyId);
extern uint8 BlackList_Remove(uint8 index);
extern void RecordList_LoadPara(uint8 offset);
extern void RecordList_StorePara(uint8 mode);
extern void RecordIndex_LoadPara( void );
extern void RecordIndex_StorePara( void );


extern void ServeKeyList_Readflash(uint8 offset);
extern void ServeKeyList_Writeflash(uint8 offset);
extern void CpuFobList_Readflash(uint8 offset);
extern void CpuFobList_Writeflash(uint8 offset);

extern void  RecordAddr_Readflash( void );

//extern void BlackList_Readflash(uint8 offset);
//extern void BlackList_Writeflash(uint8 offset);

#endif    
/************************ (C) COPYRIGHT DaHao electronics *****END OF FILE****/

