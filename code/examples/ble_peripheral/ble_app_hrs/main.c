/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
/** @example examples/ble_peripheral/ble_app_hrs/main.c
 *
 * @brief Heart Rate Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Heart Rate service
 * (and also Battery and Device Information services). This application uses the
 * @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "Access\Access.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_dahao.h"
#ifdef BLE_DFU_APP_SUPPORT
#include "ble_dfu.h"
#include "dfu_app_handler.h"
#endif // BLE_DFU_APP_SUPPORT
#include "ble_conn_params.h"
#include "boards.h"
#include "sensorsim.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "device_manager.h"
#include "pstorage.h"
#include "app_trace.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "bsp_btn_ble.h"
#include "SYSTEM\sys.h"
#include "Beep\Beep.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "Access\Access.h"
#include "Protocol\Proto_NetComm.h"
#include "RTC\Rtc.h"
#include "KEY\key.h"
#include "Card\Rc522.h"
#include "HC595\HC595.h"
#include "COMMU\Lock_Commu.h"
#include "Flash\Flash.h"
#include "SYSTEM\Sys_TimeBase.h"
#include "Light\light.h"
#include "Beep\Beep.h"
#include "KEY\Touch.h"
#include "protocol\Proto_Analy.h"
#include "lora\radio.h"
#include "KEY\Touch.h"
#include <cstdlib>
#include "Flash\Flash_Nrf.h"
#include "FACTORY\Factory.h"
#include "app_uart.h"
#include "ACCESS\LockKey.h"

extern uint32 Sys_RestTimex64ms;
extern uint8  protoAnaly_freqsetflag;
extern _Bool setDstFlag;
extern uint32_t DFUServiceTime;
extern _Bool checkKeyForExpiredKeys;
extern LockKey_ServeKeyListType LockKeyServeKeyList;;

extern void ble_dahao_get_manual_data(void);
extern Std_ReturnType power_out_sleep(void);
extern void Lora_timer_start(uint8 sttype,uint16 time);
extern void Carddet_timer_start(uint16 time);
extern void Tsmxx_Irq_Init(void);
extern void ReLay_Timepro(void);
extern void LogsCounter(ReadIndexDataType *readIndexPtr);
extern void FactortAppTest_ResponseApp(void);
extern void Ageing_Timepro(void);
extern void PcbaTest_pro(void);

Std_ReturnType Access_ComTimeLoop(uint32 startTime, uint32 endTime);
void advertising_init(void);
void ServeKeyList_Readflash(uint8 offset);
#ifdef ENABLE_CONSOLE_LOG
extern void printRTCTime(uint32_t rawTime);
#endif // ENABLE_CONSOLE_LOG

#define CENTRAL_LINK_COUNT               0                                          /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT            1                                          /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                      "YYLOCK"                               /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "DH"                      /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                 160                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_INTERVAL_ALL          160*9   // 160*9                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS       0                                        /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER              0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE          6                                          /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(60, UNIT_1_25_MS) //MSEC_TO_UNITS(100, UNIT_1_25_MS)//MSEC_TO_UNITS(400, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)// MSEC_TO_UNITS(200, UNIT_1_25_MS)//MSEC_TO_UNITS(650, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                    0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                   1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                   0                                          /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS               0                                          /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                         /**< Maximum encryption key size. */

#define DEAD_BEEF                        0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define POWER_RESET 0x00000005u

#define MARCH             3
#define NOVEMBER         11
#define FIRST_DAY         1
#define NUM_DAYS_IN_WEEK  7

#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define DFU_REV_MAJOR                    0x00                                       /** DFU Major revision number to be exposed. */
#define DFU_REV_MINOR                    0x01                                       /** DFU Minor revision number to be exposed. */
#define DFU_REVISION                     ((DFU_REV_MAJOR << 8) | DFU_REV_MINOR)     /** DFU Revision number to be exposed. Combined of major and minor versions. */

STATIC_ASSERT(IS_SRVC_CHANGED_CHARACT_PRESENT);                                     /** When having DFU Service support in application the Service Changed Characteristic should always be present. */

/*static */ uint16_t                          m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */
ble_dahao_t                        m_ble_dahao;

//APP_TIMER_DEF(LoraWakeup);                                                              /**< Battery timer. */
//APP_TIMER_DEF(LoraEnsleep);                                                              /**< Battery timer. */
//APP_TIMER_DEF(LoraDetBeacon);                                                              /**< Battery timer. */
//APP_TIMER_DEF(LoraCallBack);                                                              /**< Battery timer. */
//APP_TIMER_DEF(LoraSend);                                                              /**< Battery timer. */

APP_TIMER_DEF(carddet);
APP_TIMER_DEF(SysBase);   
//APP_TIMER_DEF(TimeOoutQuic);   

APP_TIMER_DEF(BleProc);                                                              

dm_application_instance_t         m_app_handle;                              /**< Application identifier allocated by device manager */

static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_DAHAO_SERVICE,         BLE_UUID_TYPE_BLE},
                                  // {BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE},
                                  /*{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}*/}; /**< Universally unique service identifiers. */
#ifdef BLE_DFU_APP_SUPPORT
ble_dfu_t                         m_dfus;                                    /**< Structure used to identify the DFU service. */
#endif // BLE_DFU_APP_SUPPORT


uint8  Lora_SendTimes = 0;
uint8  Lora_State;
uint8  Lora_CadCount;
Proto_NetcommType Proto_NetSendBack = NET_NULL;
uint8  Lorasendx64ms= 0;
uint8 SysSleepFlag = STD_FALSE;
uint8 BattdetTimer = STD_FALSE;
// Default values for key
//
uint8_t pKey[4] = {0xAA, 0xAA, 0xAA, 0xAA};
uint32_t reset_reason = 0;
_Bool LogOperationStatus = false;
_Bool DFUServiceFlag = false;

extern ble_adv_mode_t                  m_adv_mode_current;

static void setDSTLockTime(_Bool dstFlag);
static void increaseHourValue(Rtc_Type *ptrTime);
static _Bool isLastDayOfMonth(uint8_t dayOfMonth, uint8_t month, uint16_t year);
static void decreaseHourValue(Rtc_Type *ptrTime);
static uint8 calculateDaysInAMonth(uint8_t month, uint16_t year);
static _Bool isLeapYear(uint16_t year);
static int dayOfWeek(int y, int m, int d);
static void checkDST(void);
static void ble_stack_init(void);
static void device_manager_init(bool erase_bonds);
static void gap_params_init(void);
static void services_init(void);
static void conn_params_init(void);

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

void ble_dahao_timer_proc(void)
{
    uint32_t                err_code;
    
    if(Sys_DisableBlueTooth())
    {
        return;
    }
    if(m_disconnect_timer != 0)
    {
        --m_disconnect_timer;
        if(m_disconnect_timer == 0)
        {
            g_LockComuData.recIndex = 0;
             if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
                {
                    err_code = sd_ble_gap_disconnect(m_conn_handle,
                                                     BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                    APP_ERROR_CHECK(err_code);
                }
        }
    }
    if(m_ble_adv_timer != 0)
    {
        m_ble_adv_timer--;
    }
    if(m_ble_adv_restart_timer != 0)
    {
        m_ble_adv_restart_timer--;
        if((m_ble_adv_restart_timer == 0) && (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            && ((m_ble_adv_status != BLE_ADV_FAST) && (m_ble_adv_status != BLE_ADV_SLOW)))
        {
            err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
                APP_ERROR_CHECK(err_code);    
            m_ble_adv_timer =500/64;
        }
    }
}


void SendUart_data(uint8_t * p_data, uint16_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        while (app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
}


void BleProc_timer_start(uint16 time)
{
    app_timer_stop(BleProc);
    app_timer_start(BleProc, APP_TIMER_TICKS(time, APP_TIMER_PRESCALER), NULL);
}
static void BleProc_timer_handler(void * p_context)
{
    LockCommu_Proc();
}

void SysBasetimer_Start(uint16 time)
{
    app_timer_stop(SysBase);
    app_timer_start(SysBase, APP_TIMER_TICKS(time, APP_TIMER_PRESCALER), NULL);
    SysSleepFlag = STD_FALSE;
}


uint8_t wdat[4] = {0x11,0x22,0x33,0x44};



static void DFUServiceCheck(void)
{
    if(DFUServiceFlag)
    {
        if(--DFUServiceTime <= 0)
        {

            DFUServiceFlag = false;
        }
    }
}

void removeExpiredKeys(void)
{
    uint8 i = 0;
    uint8 j = 0;
    _Bool expiredKeyFound = false;
    _Bool isCPUfob = false;
    uint32 CPUfobid;
    if(true == checkKeyForExpiredKeys)
    {
#ifdef ENABLE_CONSOLE_LOG
        printf("Deleting expired keys\n\r");
#endif // ENABLE_CONSOLE_LOG
        for(j = 0; j < SERVE_KEY_PAGEMAX; j++)
        {
            ServeKeyList_Readflash(j);
            for(i = 0; i < SERVE_KEY_MAX; i++)
            {
                // Check if key is set
                //
                if(LockKeyServeKeyList.KeyList[i].KeyId != 0xffffffff)
                {
                    if(LockKeyServeKeyList.KeyList[i].EndTime < ProtoAnaly_RtcLocalTime)
                    {
                        if ( KEY_TYPE_CPUCARD == LockKeyServeKeyList.KeyList[i].Type  )
                        {
                          isCPUfob = true;
						  CPUfobid = LockKeyServeKeyList.KeyList[i].KeyId;
                        }
                        expiredKeyFound = true;
                        memset(&LockKeyServeKeyList.KeyList[i], 0xff, sizeof(LockKey_ServeKeyType));
                        if(LockKeyStatistics.ServKeyCount > 0)
                        {
                            LockKeyStatistics.ServKeyCount--;
                        }
                    }
                }

                if(true == expiredKeyFound)
                {
                    expiredKeyFound = false;
                    ServeKeyList_Writeflash(j);
                }

               if(true == isCPUfob)
               {
                   isCPUfob = false;
                   DeleteCPUID( CPUfobid );
               }
            }
        }
        // Reset flag to false
        //
        checkKeyForExpiredKeys = false;
    }
}

static void SysBase_timer_handler(void * p_context)
{    
    Touch_TimerProc();
    KeyDrive.ioctl(KEY_SCAN,NULL);
    Touch_Proc();
    ble_dahao_timer_proc();
    Access_TimerProc();
    //SendUart_data(wdat,4);
    //Send_Weigand34(&wdat);
    
    //HC595_LedWriteData(0x1000);
    Sys_RestProc();

    Factory_Proc();
    ReLay_Timepro();
    removeExpiredKeys();
    DFUServiceCheck();
}

void Carddet_timer_start(uint16 time)
{    
    app_timer_stop(carddet);
    app_timer_start(carddet, APP_TIMER_TICKS(time, APP_TIMER_PRESCALER), NULL);
    SysSleepFlag = STD_FALSE;
}

/****************************************************************************************************
**Function: dayOfWeek
**Author: RahulR
**Description: This function calculates day of week
**Input: uint8_t y       => Holds year value
**       uint8_t m       => Holds month value
**       uint8_t d       => Holds day of month value
**Output: int day of week
****************************************************************************************************/
static int dayOfWeek(int y, int m, int d)
{
    int xRef[NUM_DAYS_IN_WEEK] = {0, 6, 5, 4, 3, 2, 1};

    return xRef[(d += m < 3 ? y-- : y - 2, 23 * m / 9 + d + 4 + y / 4 - y / 100 + y / 400) % 7];
}

/****************************************************************************************************
**Function: isLeapYear
**Author: RahulR
**Description: This function checks if year is a leap year or not
**Input: uint8_t year       => Holds year value
**Output: _Bool 1 if year is leap year
                0 if year is not leap year
****************************************************************************************************/
static _Bool isLeapYear(uint16_t year)
{
   return( (year % 400 == 0) || ((year % 4 == 0) && (year % 100 != 0)) ) ? 1 : 0;
}

/****************************************************************************************************
**Function: calculateDaysInAMonth
**Author: RahulR
**Description: This function calculates days in a month
**Input: uint8_t month      => Holds month value
**       uint8_t year       => Holds year value
**Output: uint8 Max. number of days in a month
****************************************************************************************************/
static uint8 calculateDaysInAMonth(uint8_t month, uint16_t year)
{
    uint8 numOfDays = 0;

    year = year + 2000;

    switch(month)
    {
        case 1:
        case 3:
        case 5:
        case 7:
        case 8:
        case 10: numOfDays = 31;
                 break;
        case 4:
        case 6:
        case 9:
        case 11: numOfDays = 30;
                 break;
        case 2:  if(true == isLeapYear(year))
                 {
                     numOfDays = 29;
                 }
                 else
                 {
                     numOfDays = 28;
                 }
                 break;
        default:
#ifdef ENABLE_CONSOLE_LOG
                 printf("F1\n\r");
#endif
                 break;
    }

    return numOfDays;
}

/****************************************************************************************************
**Function: decreaseHourValue
**Author: RahulR
**Description: This function is used to decrease time by 1 hour
**Input: Rtc_Type *ptrTime => Hold value of time to be modified
**Output: None
****************************************************************************************************/
static void decreaseHourValue(Rtc_Type *ptrTime)
{
    if(ptrTime->Hour == 0)
    {
        if(1 == ptrTime->DayOfMonth)
        {
            if(1 == ptrTime->Month)
            {
                ptrTime->Year -= 1;
                // Assign month as December
                //
                ptrTime->Month = 12;
            }
            else
            {
                // Assign day of month as last day of previous month
                //
                ptrTime->Month -= 1;
                ptrTime->DayOfMonth = calculateDaysInAMonth(ptrTime->Month - 1, ptrTime->Year);
			}
        }
        else
        {
            ptrTime->DayOfMonth -= 1;
        }
        ptrTime->Hour = 23;
    }
    else
    {
        ptrTime->Hour -= 1;
    }
}

/****************************************************************************************************
**Function: isLastDayOfMonth
**Author: RahulR
**Description: This function checks if day of month is last day for that month
**Input: uint8_t dayOfMonth => Holds day of month value
**Input: uint8_t month      => Holds month value
**Input: uint8_t year       => Holds year value
**
**Output: _Bool => 1 If day of month is last day for that month
**        _Bool => 0 If day of month is not last day for that month
****************************************************************************************************/
static _Bool isLastDayOfMonth(uint8_t dayOfMonth, uint8_t month, uint16_t year)
{
    _Bool isLastDay = false;

    year = year + 2000;

    switch(dayOfMonth)
    {
        case 31:
            isLastDay = true;
            break;
        case 30:
            switch(month)
            {
                case 4:
                case 6:
                case 9:
                case 11:
                    isLastDay = true;
                    break;
            }
            break;
        case 29:
             if(2 == month)
             {
                 isLastDay = true;
             }
             break;
        case 28:
             if(2 == month)
             {
                 if(false == isLeapYear(year))
                 {
                     isLastDay = true;
                 }
             }
             break;
        default:
#ifdef ENABLE_CONSOLE_LOG
             printf("F2\n\r");
#endif
             break;
    }
    return isLastDay;
}

/****************************************************************************************************
**Function: increaseHourValue
**Author: RahulR
**Description: This function is used to increase time by 1 hour
**Input: Rtc_Type *ptrTime => Hold value of time to be modified
**Output: None
****************************************************************************************************/
static void increaseHourValue(Rtc_Type *ptrTime)
{
    if(ptrTime->Hour == 23)
    {
        if(true == isLastDayOfMonth(ptrTime->DayOfMonth, ptrTime->Month, ptrTime->Year))
        {
            ptrTime->DayOfMonth = 1;
            if(12 == ptrTime->Month)
            {
                ptrTime->Year += 1;
                ptrTime->Month = 1;
            }
            else
            {
                ptrTime->Month += 1;
            }
        }
        else
        {
            ptrTime->DayOfMonth += 1;
        }
        ptrTime->Hour = 0;
    }
    else
    {
        ptrTime->Hour += 1;
    }
}

#if 0
/****************************************************************************************************
**Function: setDSTLockTime
**Author: RahulR
**Description: This function is used to set time based on DST configuration
**Input: _Bool dstFlag => 1 indicates time to be decreased by 1 hour
**       _Bool dstFlag => 0 indicates time to be increased by 1 hour
**Output: None
****************************************************************************************************/
static void setDSTLockTime(_Bool dstFlag)
{
    Rtc_Type timeTemp;

    memcpy(&timeTemp, &ProtoAnaly_Rtcinfo, sizeof(Rtc_Type));

    if(true == dstFlag)
    {
        if(false == setDstFlag)
        {
            // Decrease hour by 1
            //
            decreaseHourValue(&timeTemp);
#ifdef ENABLE_CONSOLE_LOG
            printf("Time decreased by 1 hour\n\r");
#endif // ENABLE_CONSOLE_LOG
        }
        Sys_PataCfg.lockDSTFlag = false;
    }
    else
    {
        if(false == setDstFlag)
        {
            // Increase hour by 1
            //
            increaseHourValue(&timeTemp);
#ifdef ENABLE_CONSOLE_LOG
            printf("Time increased by 1 hour\n\r");
#endif // ENABLE_CONSOLE_LOG
        }
        Sys_PataCfg.lockDSTFlag = true;
    }

    // Set new time base on DST config
    //
    Rtc_Ioctl(RTC_CLOCKSET, &timeTemp);
    // Toggle lock DST flag
    //
    Sys_StorePara();
    setDstFlag = false;
}

/****************************************************************************************************
**Function: checkDST
**Author: RahulR
**Description: This function is used to set time based on DST configuration
**Input: None
**Output: None
****************************************************************************************************/
static void checkDST(void)
{
    int32 startDayDST = 0;
    int32 endDayDST = 0;
    _Bool isLockDSTset = false;
    _Bool isBLEDSTSet = false;
#ifdef ENABLE_CONSOLE_LOG
     static int i = 0xbb;
#endif // ENABLE_CONSOLE_LOG

    isBLEDSTSet = Sys_PataCfg.bleDSTFlag;

    // Check if DST feature is enabled through BLE command
    //
    if(true == isBLEDSTSet)
    {
#ifdef ENABLE_CONSOLE_LOG
        if(0xaa == i)
        printf("Enabling DST feature...:%d\n\r", isLockDSTset);
        i = 0xbb;
#endif // ENABLE_CONSOLE_LOG
        // Read time from RTC
        //
        ProtoAnaly_UpdateTime();

        startDayDST = FIRST_DAY + NUM_DAYS_IN_WEEK + dayOfWeek(ProtoAnaly_Rtcinfo.Year, MARCH, FIRST_DAY);
        endDayDST   = FIRST_DAY + dayOfWeek(ProtoAnaly_Rtcinfo.Year, NOVEMBER, FIRST_DAY);

        isLockDSTset = Sys_PataCfg.lockDSTFlag;
        // Check if lock time set with DST
        //
        if(true == isLockDSTset)
        {
            // Lock time set with DST
            // Check if DST window is finished
            //
            if(((ProtoAnaly_Rtcinfo.Month < MARCH)             &&
                (ProtoAnaly_Rtcinfo.Month > NOVEMBER))
              ||
               ((ProtoAnaly_Rtcinfo.Month      == MARCH)       &&
                (ProtoAnaly_Rtcinfo.DayOfMonth < startDayDST))
              ||
               ((ProtoAnaly_Rtcinfo.Month      == NOVEMBER)    &&
                (ProtoAnaly_Rtcinfo.DayOfMonth > endDayDST))
              ||
               ((ProtoAnaly_Rtcinfo.Month      == MARCH)       &&
                (ProtoAnaly_Rtcinfo.DayOfMonth == startDayDST) &&
                (ProtoAnaly_Rtcinfo.Hour       < 2))
              ||
               ((ProtoAnaly_Rtcinfo.Month      == NOVEMBER)    &&
                (ProtoAnaly_Rtcinfo.DayOfMonth == endDayDST)   &&
                (ProtoAnaly_Rtcinfo.Hour       > 2)))
            {
                // Set time with hour time decreased by 1 hour
                //
                setDSTLockTime(true);
            }
        }
        else
        {
            // Lock time not set with DST
            // Check if DST window is started
            //
            if(((ProtoAnaly_Rtcinfo.Month      == MARCH)       &&
                (ProtoAnaly_Rtcinfo.DayOfMonth == startDayDST) &&
                (ProtoAnaly_Rtcinfo.Hour       >= 2))
              ||
               ((ProtoAnaly_Rtcinfo.Month      == MARCH)       &&
                (ProtoAnaly_Rtcinfo.DayOfMonth > startDayDST))
              ||
               ((ProtoAnaly_Rtcinfo.Month > MARCH)             &&
                (ProtoAnaly_Rtcinfo.Month < NOVEMBER))
              ||
               ((ProtoAnaly_Rtcinfo.Month      == NOVEMBER)    &&
                (ProtoAnaly_Rtcinfo.DayOfMonth < endDayDST))
              ||
               ((ProtoAnaly_Rtcinfo.Month       == NOVEMBER)   &&
                 (ProtoAnaly_Rtcinfo.DayOfMonth == endDayDST)  &&
                 (ProtoAnaly_Rtcinfo.Hour       <= 2)))
            {
                // Set time with hour time increased by 1 hour
                //
                setDSTLockTime(false);
            }
        }
    }
    else
    {
#ifdef ENABLE_CONSOLE_LOG
        if(0xbb == i)
        printf("Disabling DST...\n\r");
        i = 0xaa;
#endif // ENABLE_CONSOLE_LOG
        // Read time from RTC
        //
        ProtoAnaly_UpdateTime();

        isLockDSTset = Sys_PataCfg.lockDSTFlag;
        // Check if lock time set with DST
        //
        if(true == isLockDSTset)
        {
            // Set time with hour time decreased by 1 hour
            //
            setDSTLockTime(true);
        }
    }
}
#endif

static void setDSTLockTime(_Bool dstFlag)
{
	Rtc_Type timeTemp;
	memcpy(&timeTemp, &ProtoAnaly_Rtcinfo, sizeof(Rtc_Type));
	unsigned char store_flag=0;

	if(PANEL_WINTER_TIMER == dstFlag)
	{
		if( (Sys_PataCfg.lockTime_SummerTimer_SkippedFlag == SUMMER_TIMER_SKIPED_FLAG) &&
			(Sys_PataCfg.lockTime_WinterTimer_SkippedFlag == WINTER_TIMER_NOSKIPED_FLAG)&&
			( Sys_PataCfg.lockDSTFlag != DEC_TIMER_FLAG )
		  )
		{
			decreaseHourValue(&timeTemp);
			Sys_PataCfg.lockTime_SummerTimer_SkippedFlag = SUMMER_TIMER_NOSKIPED_FLAG;
			Sys_PataCfg.lockTime_WinterTimer_SkippedFlag = WINTER_TIMER_SKIPED_FLAG;
			Sys_PataCfg.lockDSTFlag = DEC_TIMER_FLAG;
			store_flag = 1;
		}
	}
	else if(PANEL_SUMMER_TIMER == dstFlag)
	{
		if( (Sys_PataCfg.lockTime_SummerTimer_SkippedFlag == SUMMER_TIMER_NOSKIPED_FLAG) &&
			(Sys_PataCfg.lockTime_WinterTimer_SkippedFlag == SUMMER_TIMER_SKIPED_FLAG)&&
			( Sys_PataCfg.lockDSTFlag != DEC_TIMER_FLAG )
		  )
		{
			increaseHourValue(&timeTemp);
			Sys_PataCfg.lockTime_SummerTimer_SkippedFlag = SUMMER_TIMER_SKIPED_FLAG;    
			Sys_PataCfg.lockTime_WinterTimer_SkippedFlag = WINTER_TIMER_NOSKIPED_FLAG;	
			store_flag = 1;
		}
	}

	if( store_flag == 1 )
	{
		Rtc_Ioctl(RTC_CLOCKSET, &timeTemp);
        RtcLocalTimebak = 0;
		Sys_StorePara();
	}
}

SummerAndWinterTimeTypedef summer_winter_time_check(void)
{
	int32 startDayDST = 0;
	int32 endDayDST   = 0;
	_Bool isBLEDSTSet  = false;
	SummerAndWinterTimeTypedef state;

	isBLEDSTSet = Sys_PataCfg.bleDSTFlag;
	if(true == isBLEDSTSet)
	{
		ProtoAnaly_UpdateTime();
		if ( Sys_PataCfg.lockDSTFlag == DEC_TIMER_FLAG )		
		{		    
		    if ((ProtoAnaly_Rtcinfo.Hour == 2) && (ProtoAnaly_Rtcinfo.Second >= 2))		    
		    {		       
			    Sys_PataCfg.lockDSTFlag = WINTER_TIMER_FLAG;		
				Sys_StorePara();		  
			}		
		}

		startDayDST = FIRST_DAY + NUM_DAYS_IN_WEEK + dayOfWeek(ProtoAnaly_Rtcinfo.Year, MARCH, FIRST_DAY);
		endDayDST   = FIRST_DAY + dayOfWeek(ProtoAnaly_Rtcinfo.Year, NOVEMBER, FIRST_DAY);

		if( ((ProtoAnaly_Rtcinfo.Month < MARCH) || (ProtoAnaly_Rtcinfo.Month > NOVEMBER)) ||

			((ProtoAnaly_Rtcinfo.Month == MARCH) && (ProtoAnaly_Rtcinfo.DayOfMonth < startDayDST)) ||

			((ProtoAnaly_Rtcinfo.Month == NOVEMBER) && (ProtoAnaly_Rtcinfo.DayOfMonth > endDayDST)) ||

			((ProtoAnaly_Rtcinfo.Month == MARCH) && (ProtoAnaly_Rtcinfo.DayOfMonth == startDayDST) && (ProtoAnaly_Rtcinfo.Hour < 2)) ||

			((ProtoAnaly_Rtcinfo.Month == NOVEMBER) && (ProtoAnaly_Rtcinfo.DayOfMonth == endDayDST) && (ProtoAnaly_Rtcinfo.Hour >= 2))
		)
		{
			state = WINTER_TIMER;
		}
		else
		{
			state = SUMMER_TIMER;
		}
	}
	return(state);
}

static void checkDST(void)
{
	if( Sys_PataCfg.bleDSTFlag == true )
	{
		setDSTLockTime( summer_winter_time_check() );
	}
	else
	{
	    ProtoAnaly_UpdateTime();
	}
}


static void checkADV(void)
{
	static uint8 ADVcnt;
    uint32_t err_code;
    uint8 idtemp[4];
	if ( ++ ADVcnt >= 10 )
	{
		ADVcnt = 0;
	    if((m_ble_adv_timer != 0) || (m_conn_handle != BLE_CONN_HANDLE_INVALID))
	    {
	        return;
	    }
		
	    if( m_adv_mode_current == BLE_ADV_MODE_IDLE )
	    {
            advertising_init();
            err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
            APP_ERROR_CHECK(err_code);    
            m_ble_adv_timer = 500/64;
            memset(idtemp,0xAA,4);
            Access_WriteRecordFlash(idtemp, ProtoAnaly_RtcLocalTime, ADV_ERROR, false);
	    }
	}
}		

static void Carddet_timer_handler(void * p_context)
{    
    if(Sys_CheckCardSleepDet() == 0)
    {
        app_timer_stop(carddet);
        app_timer_start(carddet, APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER), NULL);
    }
    CheckRtc();
    checkDST();
    checkADV();
    FactortAppTest_ResponseApp();
    Ageing_Timepro();
    PcbaTest_pro();
    if(cardopenFlag) return;
    Sys_SpiOpen();
    Access_DetCardProc();
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
// Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
// Create timers.
    (void)app_timer_create(&BleProc, APP_TIMER_MODE_SINGLE_SHOT, BleProc_timer_handler);
    (void)app_timer_create(&SysBase, APP_TIMER_MODE_REPEATED, SysBase_timer_handler);
    (void)app_timer_create(&carddet, APP_TIMER_MODE_SINGLE_SHOT, Carddet_timer_handler);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *            device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t    gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);


    ble_dahao_get_scan_data();
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)m_addl_adv_data,
                                          strlen((const char *)m_addl_adv_data));
    APP_ERROR_CHECK(err_code);
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_tx_power_set(Sys_PataCfg.bleCfg.txPower); 
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for stopping advertising.
 */
static void advertising_stop(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK(err_code);
}

/** Reset prepare */
/**@brief Function for preparing for system reset.
 *
 * @details This function implements @ref dfu_app_reset_prepare_t. It will be called by
 *          @ref dfu_app_handler.c before entering the bootloader/DFU.
 *          This allows the current running application to shut down gracefully.
 */
static void reset_prepare(void)
{
    uint32_t err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Disconnect from peer.
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        // If not connected, the device will be advertising. Hence stop the advertising.
        advertising_stop();
    }

    err_code = ble_conn_params_stop();
    APP_ERROR_CHECK(err_code);

    nrf_delay_ms(500);
}

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_dfu_init_t dfus_init;
    err_code = ble_dahao_add_service(&m_ble_dahao);
    APP_ERROR_CHECK(err_code);
    err_code = ble_dahao_add_characteristics(&m_ble_dahao);
    APP_ERROR_CHECK(err_code);

    // Initialize the Device Firmware Update Service.
    memset(&dfus_init, 0, sizeof(dfus_init));

    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.error_handler = NULL;
    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.revision      = DFU_REVISION;

    err_code = ble_dfu_init(&m_dfus, &dfus_init);
    APP_ERROR_CHECK(err_code);

    dfu_app_reset_prepare_set(reset_prepare);
    dfu_app_dm_appl_instance_set(m_app_handle);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code;// = bsp_indication_set(BSP_INDICATE_IDLE);
  //  APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);
    return ;
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
        m_ble_adv_status = BLE_ADV_FAST;
           m_ble_adv_timer = 500/64;
            break;    
    case BLE_ADV_EVT_SLOW:
           m_ble_adv_status = BLE_ADV_SLOW;
           m_ble_adv_timer = 500/64;
            break;
        case BLE_ADV_EVT_IDLE:
        sleep_mode_enter();
           m_ble_adv_status = BLE_ADV_STOP;
            break;
        default:
            break;
    }
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */

//Á´½Ó»½ÐÑ
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
            {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            BLE_UPDATA_DISCONNECT_TIMER(Sys_PataCfg.bleCfg.connectOff/64);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;
        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
    ble_dahao_on_ble_evt(&m_ble_dahao, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
#ifdef BLE_DFU_APP_SUPPORT
    /** @snippet [Propagating BLE Stack events to DFU Service] */
    ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
    /** @snippet [Propagating BLE Stack events to DFU Service] */
#endif // BLE_DFU_APP_SUPPORT
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
}

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize the SoftDevice handler module.    
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

#ifdef BLE_DFU_APP_SUPPORT
    ble_enable_params.gatts_enable_params.service_changed = 1;
#endif // BLE_DFU_APP_SUPPORT
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for loading application-specific context after establishing a secure connection.
 *
 * @details This function will load the application context and check if the ATT table is marked as
 *          changed. If the ATT table is marked as changed, a Service Changed Indication
 *          is sent to the peer if the Service Changed CCCD is set to indicate.
 *
 * @param[in] p_handle The Device Manager handle that identifies the connection for which the context
 *                     should be loaded.
 */
#define APP_SERVICE_HANDLE_START         0x000C                                     /**< Handle of first application specific service when when service changed characteristic is present. */
#define BLE_HANDLE_MAX                   0xFFFF                                     /**< Max handle value in BLE. */

static void app_context_load(dm_handle_t const * p_handle)
{
    uint32_t                 err_code;
    static uint32_t          context_data;
    dm_application_context_t context;

    context.len    = sizeof(context_data);
    context.p_data = (uint8_t *)&context_data;

    err_code = dm_application_context_get(p_handle, &context);
    if (err_code == NRF_SUCCESS)
    {
        // Send Service Changed Indication if ATT table has changed.
        if ((context_data & (DFU_APP_ATT_TABLE_CHANGED << DFU_APP_ATT_TABLE_POS)) != 0)
        {
            err_code = sd_ble_gatts_service_changed(m_conn_handle, APP_SERVICE_HANDLE_START, BLE_HANDLE_MAX);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != BLE_ERROR_INVALID_CONN_HANDLE) &&
                (err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != BLE_ERROR_NO_TX_PACKETS) &&
                (err_code != NRF_ERROR_BUSY) &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
            {
                APP_ERROR_HANDLER(err_code);
            }
        }

        err_code = dm_application_context_delete(p_handle);
        APP_ERROR_CHECK(err_code);
    }
    else if (err_code == DM_NO_APP_CONTEXT)
    {
        // No context available. Ignore.
    }
    else
    {
        APP_ERROR_HANDLER(err_code);
    }
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            err_code = ble_advertising_restart_without_whitelist();
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}


/**@brief Function for handling the Device Manager events.
 *
 * @param[in] p_evt  Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);


#ifdef BLE_DFU_APP_SUPPORT
    if (p_event->event_id == DM_EVT_LINK_SECURED)
    {
        app_context_load(p_handle);
    }
#endif // BLE_DFU_APP_SUPPORT

    return NRF_SUCCESS;
}


/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.lesc         = SEC_PARAM_LESC;
    register_param.sec_param.keypress     = SEC_PARAM_KEYPRESS;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;
    ble_advdata_manuf_data_t manuf_data;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = false;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_slow_enabled  = BLE_ADV_SLOW_ENABLED;
    if(Sys_CheckBTSleepMode())
    {
     options.ble_adv_fast_interval = APP_ADV_INTERVAL;
         options.ble_adv_fast_timeout  = 15;
         options.ble_adv_slow_interval = APP_ADV_INTERVAL*2;
         options.ble_adv_slow_timeout  = 15;
    }
    else
    {
     options.ble_adv_fast_interval = APP_ADV_INTERVAL;
         options.ble_adv_fast_timeout  = 15;
            
         options.ble_adv_slow_interval = APP_ADV_INTERVAL_ALL;
         options.ble_adv_slow_timeout  = 0;
    }
   
    ble_dahao_get_manual_data();
    manuf_data.company_identifier = 0x0059;
    manuf_data.data.p_data = m_addl_adv_manuf_data;
    manuf_data.data.size = BLE_MANUF_DATA_LEN;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.p_manuf_specific_data   = &manuf_data;
    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);

    APP_ERROR_CHECK(err_code);
}

/****************************************************************************************************
**Function:
    Std_ReturnType power_out_sleep(void)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void power_enter_sleep(void)
{
    m_ble_adv_timer = 0;
}


uint32 rtc1timebak;

static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

Std_ReturnType power_enter_sleepdet(void)
{
    static uint32 cntrtc;
    static uint32 cntwdt;

    if(++cntwdt >= 250000)
    {
        cntwdt = 0;
#if (defined WATCHDOG_EN) && (WATCHDOG_EN == STD_TRUE)
        Sys_wdt_feed();
#endif
    }
 
    if(++cntrtc >= 1000000)
    {
        cntrtc = 0;

        if(Sys_EnableBlueTooth())
        {
            ble_dahao_start_advert(1);
        }

        if(rtc1timebak == NRF_RTC1->CC[0])
        {
            NVIC_SystemReset();
        }
        else
        {
            rtc1timebak = NRF_RTC1->CC[0];
        }
    }
		power_manage();
    return E_OK;
}

/****************************************************************************************************
**Function:
    Std_ReturnType power_out_sleep(void)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
Std_ReturnType power_out_sleep(void)
{
    Vdd_PowerOn();
    return E_OK;
}

#ifdef ENABLE_CONSOLE_LOG
#define UART_TX_BUF_SIZE 1024                       /**< UART TX buffer size. */
#else
#define UART_TX_BUF_SIZE 64                         /**< UART TX buffer size. */
#endif

#define UART_RX_BUF_SIZE 64     

#define STRING_END_CHARACTER      '\0'

#define CHAR_CARRIAGE_RETURN      '\r'
#define CHAR_NEW_LINE             '\n'

#define NUMBER_SYSTEM_BINARY      2
#define NUMBER_SYSTEM_DECIMAL     10
#define NUMBER_SYSTEM_HEXADECIMAL 16

#define PRINT_BUFFER_SIZE         32U

uint8_t   uartBuff[32]={0};  
uint8_t   uartLenx =0;  
uint8_t   uartNodat=0;

void uart_event_handle(app_uart_evt_t * p_event)  
{
    if (p_event->evt_type == APP_UART_DATA_READY)  
    {  
        app_uart_get(&uartBuff[uartLenx++]); 
              uartNodat=0;
    }
}

void uart_init(void)
{
    uint32_t err_code;
    const app_uart_comm_params_t comm_params =
    {
         29,//RX_PIN_NUMBER,
         28,//TX_PIN_NUMBER,
         29,//RTS_PIN_NUMBER,
         28,//CTS_PIN_NUMBER,
         APP_UART_FLOW_CONTROL_DISABLED,
         false,
         UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_event_handle,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);

    APP_ERROR_CHECK(err_code);
}

#if 0
#define S_SWAP(a,b) do { uint8_t t = S[a]; S[a] = S[b]; S[b] = t; } while(0)

int rc4_skip(const uint8_t *key, uint32_t keylen, uint32_t skip,uint8_t *data, uint32_t data_len)
{    
    uint32_t i, j, k;    
    uint8_t S[256], *pos;    
    uint32_t kpos;    
    
    /* Setup RC4 state */    
    for (i = 0; i < 256; i++)
    {
        S[i] = i;
    }
    
    j = 0;    
    kpos = 0;    
    for (i = 0; i < 256; i++) 
    {        
        j = (j + S[i] + key[kpos]) & 0xff;
        
        kpos++;        
        if (kpos >= keylen)
        {
            kpos = 0;
        }
        S_SWAP(i, j);
    }    
    
    /* Skip the start of the stream */    
    i = j = 0;    
    for (k = 0; k < skip; k++) 
    {        
        i = (i + 1) & 0xff;        
        j = (j + S[i]) & 0xff;        
        S_SWAP(i, j);
    }    
    
    /* Apply RC4 to data */    
    pos = data;    
    for (k = 0; k < data_len; k++) 
    {        
        i = (i + 1) & 0xff;        
        j = (j + S[i]) & 0xff;        
        S_SWAP(i, j);        
        *pos++ ^= S[(S[i] + S[j]) & 0xff];    
    }    
    return 0;
}                 
#endif

_Bool isPassageModeActive(void)
{
    _Bool status = false;

    uint8_t  cycledat = 0;
    uint32_t endTime = 0;
    uint32_t startTime = 0;

    ProtoAnaly_UpdateTime();

    if(IS_SET(Sys_PataCfg.configMode, PASSMODE))
    {
        endTime = Sys_PataCfg.passageEndTime;
        startTime = Sys_PataCfg.passageStartTime;

        if(E_OK == Access_ComTime(startTime, endTime))
        {
            cycledat = Sys_PataCfg.weeklyPeriodicity;
            if(0 == (cycledat & 0x80))
            {
                uint8 t = 0;

                if(7 == ProtoAnaly_Rtcinfo.DayOfWeek)
                {
                    t = 0;
                }
                else
                {
                    t = ProtoAnaly_Rtcinfo.DayOfWeek;
                }

                if(0 != (cycledat & (1 << t)))
                {
                    if(E_OK == Access_ComTimeLoop(startTime, endTime))
                    {
                        status = true;
                    }
                }
            }
        }
    }
    return(status);
}

/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    bool erase_bonds;
    uint32_t reset_reason_clr_msk = 0xFFFFFFFF;
    uint8_t rebootKey[4] = {0};
#ifdef ENABLE_CONSOLE_LOG
    ReadIndexDataType readIndex;
#endif // ENABLE_CONSOLE_LOG

    timers_init();
    ble_stack_init();
    device_manager_init(erase_bonds);
    gap_params_init();
    services_init();
    conn_params_init();
    Sys_Init();
    advertising_init();
    
    m_ble_adv_status = BLE_ADV_STOP;
    if(Sys_EnableBlueTooth())
    {
        err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
        m_ble_adv_timer = 1000/64;
    }
    //Battdet_timer_start(1000);
    Carddet_timer_start(500);  
    SysBasetimer_Start(64);
#if (defined WATCHDOG_EN) && (WATCHDOG_EN == STD_TRUE)
    Sys_wdt_start();
#endif
#if (defined TOUCH_ENABLE) && (TOUCH_ENABLE == STD_TRUE)    
    TouchTimer64ms = 2000/64;
#endif

    //rc4_skip(Aes_KeyTable,16,0,Aes_KeyTable,16 );
#ifdef ENABLE_CONSOLE_LOG
    printf("Password: 0x%lx\n\r", Sys_PataCfg.Pswd);
    printf("AES key: ");
    for(int i = 0; i < 16; i++) printf("%.2x", Sys_PataCfg.Aeskey[i]);
    printf("\n\r");
    printf("Log: Reboot\n\r");
    printf("DST: %u\n\r", Sys_PataCfg.bleDSTFlag);
#endif  // ENABLE_CONSOLE_LOG

    reset_reason = 0;
    LogOperationStatus = false;

    sd_power_reset_reason_get(&reset_reason);

    if(0 == reset_reason) reset_reason = 0x05u;
    rebootKey[0] = (reset_reason >> 24) & 0xFF;
    rebootKey[1] = (reset_reason >> 16) & 0xFF;
    rebootKey[2] = (reset_reason >> 8) & 0xFF;
    rebootKey[3] = reset_reason & 0xFF;

#ifdef ENABLE_CONSOLE_LOG
    printf("[0x%x][0x%x][0x%x][0x%x]\n\r", rebootKey[0], rebootKey[2], rebootKey[2], rebootKey[3]);
    printf("Reboot: 0x%x\n\r", reset_reason);
    ble_version_t version;

    sd_ble_version_get(&version);
    printf("Version: %u::%u::%u\n\r", version.company_id, version.subversion_number, version.version_number);

    for ( int j=0;j<SERVE_KEY_PAGEMAX;j++ )
    {
        ServeKeyList_Readflash( j );
        for ( int i=0; i<SERVE_KEY_MAX; i++)
          {
            if (LockKeyServeKeyList.KeyList[i].KeyId != 0xffffffff)
              {
                  LockKeyStatistics.ServKeyCount++;
              }
          }
    }
    printf("LockKeyStatistics.ServKeyCount: %d\n\r", LockKeyStatistics.ServKeyCount);
#endif  // ENABLE_CONSOLE_LOG
    sd_power_reset_reason_clr(reset_reason_clr_msk);

    if(POWER_RESET == reset_reason)
    {
        LogOperationStatus = true;
    }

#if (defined(SUPPORT_RECORD_LOC_STORE)&&(SUPPORT_RECORD_LOC_STORE == STD_TRUE))
    // Update time
    //
    ProtoAnaly_UpdateTime();
    // Record log for reboot
    //
    Access_WriteRecordFlash(rebootKey, ProtoAnaly_RtcLocalTime, 26, LogOperationStatus);
#endif

    for (;;)
    {
        power_enter_sleepdet();
    }
}
