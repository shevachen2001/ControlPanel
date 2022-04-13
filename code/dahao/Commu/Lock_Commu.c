
#include "COMMU\Lock_Commu.h"
#include "SYSTEM\Sys_GenFun.h"
#include "Encpt\Aes.h"
#include "Protocol\Proto_CommPack.h"
#include "Protocol\Proto_NetComm.h"
#include "Protocol\Proto_Analy.h"
#include "ble_dahao.h"
#include "SYSTEM\sys.h"
#include "Beep\Beep.h"
#include "RTC\rtc.h"
#include "Std_Types.h"

#define MAX_TIME_DIFF_TOLERANCE 10

extern void BleProc_timer_start(uint16 time);

extern Rtc_Type  ProtoAnaly_Rtcinfo;
extern uint8 TouchTimer64ms;
extern _Bool daylightSavingTimeFlag;

LockCommu_Type g_LockComuData;
uint32 MTime;
uint8 ERespData[64];
uint32_t responseStatus = E_OK;
_Bool revertOperation = false;
uint8 ble_confirmStatus = 0;


//extern void Send_timer_start(Proto_NetcommType sttype,uint16 time);
#ifdef ENABLE_CONSOLE_LOG
void printRTCTime(uint32_t rawTime)
{
    uint32_t YEAR = 0;
    uint32_t MONTH = 0;
    uint32_t DAY = 0;
    uint32_t HOUR = 0;
    uint32_t MINUTE = 0;

    YEAR   = ((rawTime & 0xFC000000) >> 26);
    MONTH  = ((rawTime & 0x03C00000) >> 22);
    DAY    = ((rawTime & 0x003E0000) >> 17);
    HOUR   = ((rawTime & 0x0001F000) >> 12);
    MINUTE = ((rawTime & 0x00000FC0) >> 6);

    printf("%u/%u/%u %u:%2u\n\r", DAY, MONTH, YEAR, HOUR, MINUTE);
}
#endif
/****************************************************************************************************
**Function:
    void LockCommu_Init(void)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void LockCommu_Init(void)
{
    g_LockComuData.sendSt = TXD_MODE_IDLE;
//    g_LockComuData.recSt = RXD_MODE_IDLE;
//    g_LockComuData.recTime = 0;
//  g_LockComuData.sendTime = 0;
}

/****************************************************************************************************
**Function:
    void LockCommu_SendStart(uint8 *pData)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void LockCommu_SendStart(uint8 *pData, uint8 Length)
{
    uint8 sendLength;

    memcpy(g_LockComuData.pSendBuf, pData, Length);
    g_LockComuData.sendLength = Length;
    g_LockComuData.sendSt = TXD_MODE_WAIT;
    if(g_LockComuData.sendLength <= BLE_FRAME_MAX_DATA_LEN)
    {
        sendLength = g_LockComuData.sendLength;
    }
    else
    {
        sendLength = BLE_FRAME_MAX_DATA_LEN;
        ble_confirmStatus = 1;
    }
    responseStatus = ble_dahao_notify_data(&m_ble_dahao, g_LockComuData.pSendBuf, sendLength);
    if(E_OK != responseStatus)
    {
        revertOperation = true;
    }
    g_LockComuData.sendIndex = sendLength;
    if(g_LockComuData.sendLength > g_LockComuData.sendIndex)
    {
      //Send_timer_start(NRF_BLE,50);
    }
    else 
    {
        g_LockComuData.sendIndex = 0;
        g_LockComuData.sendLength = 0;
    }
}

#ifdef ENABLE_CONSOLE_LOG
unsigned char BleDataTab[128] = {0};
unsigned char BledataLen = 0;
extern uint32_t app_uart_put(uint8_t byte);

void xPrintfBleData(uint8 cmd)
{
    unsigned char i = 0, a = 0, b = 0;

    printf("%s\n\r", cmd ? "REQ" : "RESP");
    for(i=0; i<BledataLen; i++)
    {
        a = ((BleDataTab[i]&0xF0) >> 4);
        a = (a < 10) ? (a + '0') : (a - 10 + 'A');

        b = ((BleDataTab[i]&0x0F) >> 0);
        b = (b < 10) ? (b + '0') : (b - 10 + 'A');

        app_uart_put(a);
        app_uart_put(b);
        app_uart_put(' ');
    }
    printf("\n\r");
    BledataLen = 0;
}
#endif

void ErrorResponse(uint8_t cmd, uint8_t errStatus)
{
    uint8 encryptedRespMsg[16]={0};
    uint8 checksum=0;
    uint8 lenth=0;

		ERespData[0]=0x55;
		ERespData[1]=0xAA;
		
    ERespData[2] = 15;
    ERespData[3] = cmd; // cmd
		
		ERespData[4]=(unsigned char)((Sys_PataCfg.SysId&0xFF00)>>8);
		ERespData[5]=(unsigned char)((Sys_PataCfg.SysId&0x00FF)>>0);
		ERespData[6]=(unsigned char)((Sys_PataCfg.Mac&0xFF000000)>>24);
	  ERespData[7]=(unsigned char)((Sys_PataCfg.Mac&0x00FF0000)>>16);
	  ERespData[8]=(unsigned char)((Sys_PataCfg.Mac&0x0000FF00)>>8);
	  ERespData[9]=(unsigned char)((Sys_PataCfg.Mac&0x000000FF)>>0);

    ERespData[10]= errStatus; // errStatus
    ERespData[11]= ProtoAnaly_Rtcinfo.Year;
    ERespData[12] = ProtoAnaly_Rtcinfo.Month;
    ERespData[13] = ProtoAnaly_Rtcinfo.DayOfMonth;
    ERespData[14]= ProtoAnaly_Rtcinfo.Hour;
    ERespData[15] =ProtoAnaly_Rtcinfo.Minute;
    ERespData[16]= ProtoAnaly_Rtcinfo.Second;

    if((ERespData[2]&0x0f) != 0)
    {
			 ERespData[2] = (ERespData[2]&0xF0)+ 0x10;
    }

    lenth = ERespData[2] + 3;
   //Ôö¼Óchecksum

    checksum = Sys_GenFunChecksum(&ERespData[2]);
    ERespData[ERespData[2] + 2] = checksum;
    lenth = ERespData[2] + 3;
  
#ifdef ENABLE_CONSOLE_LOG
    for(int i=0; i < lenth; i++)
    {
        BleDataTab[BledataLen++]=ERespData[i];
    }
    xPrintfBleData(0);
#endif // #ifdef ENABLE_CONSOLE_LOG

    for(uint32_t index = 0;index < ((lenth - 3) >> 4); index++)
    {
        AES_Encrypt(&ERespData[3 + (index << 4)], encryptedRespMsg);
        memcpy(&ERespData[3 + (index << 4)], encryptedRespMsg, 16);
    }

    Proto_NetCommSend(NRF_BLE,ERespData,lenth);
}

void getReplayTimeOffset(uint8_t cmdNum, uint8_t *pOffset)
{
    switch(cmdNum)
    {
        case 0x27: *pOffset=1;  break;
        case 0xb6: *pOffset=2;  break;
        case 0x28: *pOffset=11; break;
        case 0x7D: *pOffset=10; break;
        case 0x5C: *pOffset=15; break;
        case 0x5B: *pOffset=12; break;
        case 0x23: *pOffset=14; break;
        case 0x46: *pOffset=14; break;
        case 0xA8: *pOffset=6;  break;
        case 0xA6: *pOffset=14; break;
        case 0xA7: *pOffset=15; break;
        case 0xB8: *pOffset=15; break;
        case 0xB5: *pOffset=2;  break;
        case 0xBB: *pOffset=15; break;	
        case 0xBA: *pOffset=12; break;	
        default:   *pOffset=0;  break;
     }
}

int32_t calculateTimeDiff(uint8_t *replayTime)
{
    uint32_t timeRcvdFromClient = 0;
    uint32_t timeLockHw = 0;

    // Read replay time field and build an integer from it
    //
    timeRcvdFromClient = BUILD_UINT32(replayTime[3], replayTime[2], replayTime[1], replayTime[0]);
    
    // Read latest updated time of lock HW
    //
    ProtoAnaly_UpdateTime();
    // Build an integer from date and time
    //
    timeLockHw = BUILD_TIME(ProtoAnaly_Rtcinfo.Year,
                            ProtoAnaly_Rtcinfo.Month,
                            ProtoAnaly_Rtcinfo.DayOfMonth,
                            ProtoAnaly_Rtcinfo.Hour, 
                            ProtoAnaly_Rtcinfo.Minute,
                            ProtoAnaly_Rtcinfo.Second);
#ifdef ENABLE_CONSOLE_LOG
    printf("Timestamp 0x%x::%u\n\rDiff::%d\n\r", timeRcvdFromClient,
                                                 timeLockHw, 
                                                 timeLockHw - timeRcvdFromClient);
    printRTCTime(timeRcvdFromClient);
    printRTCTime(timeLockHw);
#endif // ENABLE_CONSOLE_LOG
    return (timeLockHw - timeRcvdFromClient);
}

void setTimeOnInitOperation(uint8_t *replayTime)
{
    Rtc_Type Rtc_data;
    uint32_t timeRcvdFromClient = 0;

    // Read replay time field and build an integer from it
    //
    timeRcvdFromClient = BUILD_UINT32(replayTime[3], replayTime[2], replayTime[1], replayTime[0]);

    Rtc_data.Year       = BUILD_YEAR(timeRcvdFromClient);
    Rtc_data.Month      = BUILD_MONTH(timeRcvdFromClient);
    Rtc_data.DayOfMonth = BUILD_DAY(timeRcvdFromClient);
    Rtc_data.Hour       = BUILD_HOUR(timeRcvdFromClient);
    Rtc_data.Minute     = BUILD_MINUTE(timeRcvdFromClient);
    Rtc_data.Second     = BUILD_SECOND(timeRcvdFromClient);

    // Intitiate clock set operation
    //
    Rtc_Ioctl(RTC_CLOCKSET, &Rtc_data);
	RtcLocalTimebak = 0;
}


uint8_t isReplayTimeValid(uint8_t* pindat)
{
    uint8_t replayTimeOffset = 0;
    int32_t timeDiff = 0;
    uint8_t status = true;
    uint8_t commandNum = 0;

    commandNum = pindat[PROTO_COMM_COMD_CMDADD];
#ifdef ENABLE_CONSOLE_LOG
    printf("Command: 0x%x\n\r", commandNum);
#endif // ENABLE_CONSOLE_LOG
    // Ignore replay time value for read lock time command
    //
    if(COMD_GET_TIME_APP != commandNum)
    {
        getReplayTimeOffset(commandNum, &replayTimeOffset);
        replayTimeOffset = (g_LockComuData.recIndex - 5 - replayTimeOffset);

#ifdef ENABLE_CONSOLE_LOG
        printf("Recvd: %u::Timeoffset: %u\n\r", g_LockComuData.recIndex, replayTimeOffset);
#endif // ENABLE_CONSOLE_LOG
        // Check if this is AES key set operation
        //
        if(COMD_AESKEY_SET == commandNum)
        {
            // Set lock time with time received from client
            // and ignore replay time
            //
            setTimeOnInitOperation(&pindat[replayTimeOffset]);
        }
        else
        {
            timeDiff = calculateTimeDiff(&pindat[replayTimeOffset]);
            if(timeDiff > MAX_TIME_DIFF_TOLERANCE || timeDiff < -MAX_TIME_DIFF_TOLERANCE)
            {
                status = false;
            }

            if(false != status)
            {
                // For set lock time command check configuration for DST flag
                //
                if(COMD_SET_TIME_APP == commandNum)
                {
                    // Extract DST flag located after REPLAYTIME field
                    //
                    daylightSavingTimeFlag = pindat[replayTimeOffset + 4] & 0x01;
                }
            }
        }
    }

    return status;
}

uint8_t isChecksumValid(uint8_t* pindat)
{
    uint8_t status = false;
    uint8_t checksum = 0;

    checksum = pindat[pindat[PROTO_COMM_COMD_LENTHADD] + 2];
    if(checksum == Sys_GenFunChecksum(&pindat[PROTO_COMM_COMD_LENTHADD]))
    {
        status = true;
    }
    return status;
}

void LockCommu_Proc(void)
{
    uint8_t decryptedData[16] = {0};

    if(Sys_DisableBlueTooth())
    {
        return;
    }
    if(g_LockComuData.recIndex != 0)
    {
        uint8 i;
        for(i= 0;i< (g_LockComuData.recIndex-3>>4);i++)
        {
            AES_Decrypt(&g_LockComuData.pRecBuf[3 + ( i<< 4)], decryptedData);
            memcpy(&g_LockComuData.pRecBuf[3 + (i << 4)], decryptedData, 16);
        }

#ifdef ENABLE_CONSOLE_LOG
        for(i = 0; i < g_LockComuData.recIndex; i++)
        {
            BleDataTab[BledataLen++] = g_LockComuData.pRecBuf[i];
        }
      // xPrintfBleData(1);
#endif
        // Validate checksum
        //
        if(!isChecksumValid(&g_LockComuData.pRecBuf[0]))
        {
#ifdef ENABLE_CONSOLE_LOG
            printf("%s: %s: %u: %s\n\r", "Failure: \n\r", __FILE__, __LINE__, "Invalid Checksum");
#endif // ENABLE_CONSOLE_LOG
            // Respond with error code 0x0D
            //
            ErrorResponse(g_LockComuData.pRecBuf[PROTO_COMM_COMD_CMDADD], 0x0D);
        }else if(!isReplayTimeValid(&g_LockComuData.pRecBuf[0]))
        {
#ifdef ENABLE_CONSOLE_LOG
            printf("%s: %s: %u: %s\n\r", "Failure: \n\r", __FILE__, __LINE__, "Invalid Time");
#endif // ENABLE_CONSOLE_LOG
            // Respond with error code 0x05
            //
            ErrorResponse(g_LockComuData.pRecBuf[PROTO_COMM_COMD_CMDADD], 0x05);
        }
        else
        {
            Proto_CommHostUnPackRec(NRF_BLE, &g_LockComuData.pRecBuf[0], g_LockComuData.recIndex);
        }
        //BLE_UPDATA_DISCONNECT_TIMER(7000/64);
        if( g_LockComuData.pRecBuf[PROTO_COMM_COMD_CMDADD] == COMD_GET_FACTORY_TEST )
        {
           BLE_UPDATA_DISCONNECT_TIMER(30000/64);  
        }        
        g_LockComuData.recIndex = 0;
        TouchTimer64ms=0;
        BleProc_timer_start(50);
        return ;
    }  
    
    
    if( Sys_DisableBlueTooth())
    {
        return ;
    }
    if(LockCommu_TimerProc() != E_OK)
    {
        BleProc_timer_start(50);
    }
}

Std_ReturnType LockCommu_TimerProc(void)
{
    uint8 Length = 0, sendLength = 0;

    if(0 == ble_confirmStatus)
    {
        return E_OK;
    }

    if(1 == ble_confirmStatus)
    {
        return E_NOT_OK;
    }

    Length = g_LockComuData.sendLength - g_LockComuData.sendIndex;
    if(Length > 0)
    {
        if(Length <= BLE_FRAME_MAX_DATA_LEN)
        {
            sendLength = Length;
            responseStatus = ble_dahao_notify_data(&m_ble_dahao, &g_LockComuData.pSendBuf[g_LockComuData.sendIndex], sendLength);

            if(E_OK != responseStatus)
            {
                revertOperation = true;
            }

            g_LockComuData.sendLength= 0;
            g_LockComuData.sendIndex = 0;
            ble_confirmStatus = 0;
            return E_OK;
        }
        else
        {
            sendLength = BLE_FRAME_MAX_DATA_LEN;
            responseStatus = ble_dahao_notify_data(&m_ble_dahao, &g_LockComuData.pSendBuf[g_LockComuData.sendIndex], sendLength);
            if(E_OK != responseStatus)
            {
                revertOperation = true;
            }
            g_LockComuData.sendIndex += sendLength;
            ble_confirmStatus = 1;
        }
        return E_NOT_OK;
    }
    else
    {
        return E_OK;
    }
}
