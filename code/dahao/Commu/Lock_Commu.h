#ifndef LOCK_COMMU_H
#define LOCK_COMMU_H

#include "PUBLIC\Std_Types.h"

//------------------------------------------------------------------------------
//--------------------------------------------------------------------------------

//#define RELEASE_VER                      //定义版本发布用

//------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
#define FRAME_LENGTH         0
#define FRAME_CMD               1
#define FRAME_SYS_PAWSD  2
#define FRAME_LOCK_ID        4
#define FRAME_USER_PAWSD  8
#define FRAME_RETURN_ST   8

#define SEND_MAX                 60
#define RECEIVE_MAX            60

#define SEND_ONE_FRAME_TIME  100


#define APP_LOCK_HEAD01   0xAA
#define APP_LOCK_HEAD02   0x55

#define LOCK_APP_HEAD01   0x55
#define LOCK_APP_HEAD02   0xAA
enum
{
    RXD_MODE_IDLE = 0,
    RXD_MODE_SYNC,
    RXD_MODE_START,
    RXD_MODE_RECEIVE,
    RXD_MODE_END,
    RXD_MODE_ERROR
};

enum
{
    TXD_MODE_IDLE = 0,
    TXD_MODE_WAIT
};

/******************************************************************************
**********************Type statement*******************************************
******************************************************************************/
typedef struct 
{
    //uint8 sendBuf[SEND_MAX+4];
    uint8 sendIndex;
    uint8 sendLength;
    uint8 sendSt;
    uint8 sendError;
    uint8 pRecBuf[RECEIVE_MAX+1];
    uint8 pSendBuf[SEND_MAX+1];
    
    //uint8 *pSendBuf;
//    uint8 recSt;
    uint8 recIndex;
//    uint8 recTime;
    uint8 sendTime;
}LockCommu_Type;




/******************************************************************************
**********************Variable definition***************************************
******************************************************************************/
extern LockCommu_Type g_LockComuData;

#define LockCommu_SendOk() (g_LockComuData.sendIndex == 0)
/******************************************************************************
**********************Function declared****************************************
******************************************************************************/
extern void LockCommu_Init(void);
extern void LockCommu_Proc(void);
extern void LockCommu_SendStart(uint8 *pData, uint8 Length);
extern void LockCommu_SendProc(void);
//extern void LockCommu_TimerProc(void);
extern Std_ReturnType LockCommu_TimerProc(void);


#endif /* HOMELOCK_H */
